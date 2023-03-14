#include "drake_ros/action/follow_joint_trajectory_server.h"

#include <drake/common/trajectories/piecewise_polynomial.h>

#include "drake_ros/core/serializer.h"
#include "drake_ros/core/serializer_interface.h"

namespace drake_ros_action {

using drake_ros_core::SerializerInterface;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::Value;

namespace {
template <typename DataT>
class Queue {
 public:
  void Put(std::shared_ptr<DataT> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = data;
  }

  std::shared_ptr<DataT> Take() {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::move(data_);
  }

 private:
  // Mutex to synchronize access to the queue.
  std::mutex mutex_;
  // Last received data (i.e. queue of size 1).
  std::shared_ptr<DataT> data_;
};
}  // namespace

struct FollowJointTrajectoryServerSystem::Impl {
  // Queue of serialized goals.
  Queue<FollowJointTrajectoryServerSystem::GoalHandle> queue;
};

FollowJointTrajectoryServerSystem::FollowJointTrajectoryServerSystem(
    const std::string& action_name, const std::vector<std::string>& joint_names,
    drake_ros_core::DrakeRos* ros, double feedback_publish_period)
    : num_joints_(joint_names.size()),
      joint_names_(joint_names),
      impl_(new Impl()) {
  drake::unused(action_name, joint_names, ros, feedback_publish_period);

  this->DeclareVectorInputPort("actual_position", joint_names.size());
  this->DeclareVectorOutputPort(
      "position", joint_names.size(),
      &FollowJointTrajectoryServerSystem::PositionOut);

  rclcpp::Node* node = ros->get_mutable_node();

  goal_handle_index_ =
      this->DeclareAbstractState(Value<std::shared_ptr<GoalHandle>>{nullptr});
  trajectory_index_ = this->DeclareAbstractState(
      Value<PiecewisePolynomial<double>>());

  this->DeclarePerStepUnrestrictedUpdateEvent(
      &FollowJointTrajectoryServerSystem::Monitor);

  // TODO: declare periodic unrestricted update to update tracking state
  // (publish Result from here, too)
  // TODO: declare periodic publish for feedback

  // Initialization event to take initial state of the robot.
  last_position_index_ = this->DeclareDiscreteState(joint_names.size());
  this->DeclareInitializationDiscreteUpdateEvent(
      &FollowJointTrajectoryServerSystem::Initialize);

  // Now create the actual action server interface.
  using namespace std::placeholders;
  this->action_server_ = rclcpp_action::create_server<ActionType>(
      node, action_name,
      std::bind(&FollowJointTrajectoryServerSystem::HandleGoal, this, _1, _2),
      std::bind(&FollowJointTrajectoryServerSystem::HandleCancel, this, _1),
      std::bind(&FollowJointTrajectoryServerSystem::HandleAccepted, this, _1));
}

FollowJointTrajectoryServerSystem::~FollowJointTrajectoryServerSystem() {}

EventStatus FollowJointTrajectoryServerSystem::Initialize(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  Eigen::VectorXd position = this->get_input_port().Eval(context);
  DRAKE_DEMAND(position.size() == num_joints_);
  discrete_state->set_value(last_position_index_, position);
  return EventStatus::Succeeded();
}

drake::systems::EventStatus FollowJointTrajectoryServerSystem::Monitor(
    const drake::systems::Context<double>& context,
    drake::systems::State<double>* state) const {
  auto& goal_handle =
      state->get_mutable_abstract_state<std::shared_ptr<GoalHandle>>(
          goal_handle_index_);
  if (!goal_handle) {
    return EventStatus::Succeeded();
  }
  // TODO(russt): Check current tracking performance.
  const auto& traj = context.get_abstract_state<PiecewisePolynomial<double>>(
      trajectory_index_);
  if (context.get_time() > traj.end_time() && rclcpp::ok()) {
    auto result = std::make_shared<ActionType::Result>();
    result->error_code = result->SUCCESSFUL;
    goal_handle->succeed(std::move(result));
    goal_handle = nullptr;
    drake::log()->info("Goal success!");
  }
  return EventStatus::Succeeded();
}

EventStatus FollowJointTrajectoryServerSystem::PublishFeedback(
    const Context<double>& context) const {
  drake::unused(context);
  return EventStatus::Succeeded();
}

void FollowJointTrajectoryServerSystem::PositionOut(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& traj = context.get_abstract_state<PiecewisePolynomial<double>>(
      trajectory_index_);
  if (traj.empty()) {
    output->SetFromVector(
        context.get_discrete_state(last_position_index_).value());
  } else {
    output->SetFromVector(traj.value(context.get_time()));
  }
}

void FollowJointTrajectoryServerSystem::DoCalcNextUpdateTime(
    const drake::systems::Context<double>& context,
    drake::systems::CompositeEventCollection<double>* events,
    double* time) const {
  // Vvv Copied from LcmSubscriberSystem vvv

  // We do not support events other than our own message timing events.
  LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
  DRAKE_THROW_UNLESS(events->HasEvents() == false);
  DRAKE_THROW_UNLESS(std::isinf(*time));

  std::shared_ptr<GoalHandle> goal_handle = impl_->queue.Take();

  // Do nothing unless we have a new message.
  if (!goal_handle) {
    return;
  }

  // Create a unrestricted event and tie the handler to the corresponding
  // function.
  drake::systems::UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback
      callback = [this, goal_handle{std::move(goal_handle)}](
                     const drake::systems::Context<double>& context,
                     const drake::systems::UnrestrictedUpdateEvent<double>&,
                     drake::systems::State<double>* state) {
        drake::systems::AbstractValues& abstract_state =
            state->get_mutable_abstract_state();
        // Parse the goal into a trajectory.
        const std::shared_ptr<const typename ActionType::Goal> goal =
            goal_handle->get_goal();
        const double current_time = context.get_time();
        const int num_samples = goal->trajectory.points.size();
        Eigen::VectorXd breaks(num_samples);
        Eigen::MatrixXd samples(num_joints_, num_samples);
        // TODO(russt): Smooth commanded trajectory from last command
        for (int i=0; i<num_samples; ++i) {
          breaks[i] = current_time +
                      goal->trajectory.points[i].time_from_start.sec +
                      1e-9 * goal->trajectory.points[i].time_from_start.nanosec;
          for (int j = 0; j < num_joints_; ++j) {
            samples(j, i) = goal->trajectory.points[i].positions[j];
          }
        }
        auto& trajectory_value =
            abstract_state.get_mutable_value(trajectory_index_);
        trajectory_value.set_value(
            PiecewisePolynomial<double>::FirstOrderHold(breaks, samples));
        // Move goal_handle into the context.
        auto& goal_handle_value =
            abstract_state.get_mutable_value(goal_handle_index_);
        // TODO(russt): call goal_handle->abort() if necessary.
        goal_handle_value.set_value(std::move(goal_handle));
        return drake::systems::EventStatus::Succeeded();
      };

  // Schedule an update event at the current time.
  *time = context.get_time();
  drake::systems::EventCollection<
      drake::systems::UnrestrictedUpdateEvent<double>>& uu_events =
      events->get_mutable_unrestricted_update_events();
  uu_events.AddEvent(drake::systems::UnrestrictedUpdateEvent<double>(
      drake::systems::TriggerType::kTimed, callback));
}

rclcpp_action::GoalResponse FollowJointTrajectoryServerSystem::HandleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const ActionType::Goal> goal) {
  drake::log()->info("Received goal");
  // TODO(russt): Check that joint_names match.
  // TODO(russt): Check that times are strictly increasing.
  // TODO(russt): Check that positions are all the correct size.
  if (static_cast<int>(goal->trajectory.joint_names.size()) != num_joints_) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FollowJointTrajectoryServerSystem::HandleCancel(
  const std::shared_ptr<GoalHandle>)
{
  drake::log()->info("Received cancellation request.");
  // TODO(russt): Implement cancel.
  // return rclcpp_action::CancelResponse::ACCEPT;
  return rclcpp_action::CancelResponse::REJECT;
}

void FollowJointTrajectoryServerSystem::HandleAccepted(
    const std::shared_ptr<GoalHandle> goal_handle) {
  drake::log()->info("Accepted goal");
  impl_->queue.Put(goal_handle);
}

}  // namespace drake_ros_action
