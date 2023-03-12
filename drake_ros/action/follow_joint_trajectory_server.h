#pragma once

#include <memory>
#include <string>

#include <drake/systems/framework/leaf_system.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "drake_ros/core/drake_ros.h"

namespace drake_ros_action {

/* TODO(russt): Offer a general type-erased version of an action server system.
 Nevertheless, FollowTrajectoryAction is one of the most commonly used actions,
 especially in conjunction with MoveIt!, and deserves special treatment. */

/** A system that provides a ROS action server for the FollowTrajectory action
defined in http://wiki.ros.org/control_msgs. The system subscribes to the
FollowTrajectory action Request, smoothly blending a new request with any
currently executing requests. It periodically publishes the action Feedback
(only when executing a request), and publishes the action Result upon
completion/termination of a Request.
 */
class FollowJointTrajectoryServerSystem final
    : public drake::systems::LeafSystem<double> {
 public:
  using ActionType = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;

  FollowJointTrajectoryServerSystem(
      const std::string& action_name,
      const std::vector<std::string>& joint_names,
      drake_ros_core::DrakeRos* ros,
      double feedback_publish_period = 0.0);

  ~FollowJointTrajectoryServerSystem() override;

 private:
  void DoCalcNextUpdateTime(const drake::systems::Context<double>&,
                            drake::systems::CompositeEventCollection<double>*,
                            double*) const override;

  void PositionOut(const drake::systems::Context<double>& context,
                   drake::systems::BasicVector<double>* output) const;

  drake::systems::EventStatus Initialize(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* values) const;

  drake::systems::EventStatus Monitor(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  drake::systems::EventStatus PublishFeedback(
      const drake::systems::Context<double>& context) const;

  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ActionType::Goal> goal);

  rclcpp_action::CancelResponse HandleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void HandleAccepted(
      const std::shared_ptr<GoalHandle> goal_handle);

  int num_joints_;
  std::vector<std::string> joint_names_;
  rclcpp_action::Server<ActionType>::SharedPtr action_server_;
  drake::systems::AbstractStateIndex goal_handle_index_;
  drake::systems::AbstractStateIndex trajectory_index_;
  drake::systems::DiscreteStateIndex last_position_index_;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace drake_ros_action
