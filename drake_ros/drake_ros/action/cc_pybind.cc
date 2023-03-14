#include <memory>
#include <unordered_set>

#include <drake/systems/framework/leaf_system.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake_ros/core/drake_ros.h"
#include "drake_ros/action/follow_joint_trajectory_server.h"
#include "drake_ros/drake_ros_pybind.h"

namespace drake_ros {
namespace drake_ros_py DRAKE_ROS_NO_EXPORT {

using drake_ros_action::FollowJointTrajectoryServerSystem;
using drake_ros_core::DrakeRos;
using drake::systems::LeafSystem;

void DefAction(py::module m) {
  m.doc() = "Python bindings for drake_ros.action";

  py::module::import("numpy");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.multibody.plant");
  py::module::import("pydrake.math");
  py::module::import("pydrake.common.eigen_geometry");
  py::module::import("drake_ros.core");

  py::class_<FollowJointTrajectoryServerSystem, LeafSystem<double>>(
      m, "FollowJointTrajectoryServerSystem")
      .def(py::init([](const std::string& action_name,
                       const std::vector<std::string>& joint_names,
                       drake_ros_core::DrakeRos* ros,
                       double feedback_publish_period) {
             return std::make_unique<FollowJointTrajectoryServerSystem>(
                 action_name, joint_names, ros, feedback_publish_period);
           }),
           py::arg("action_name"), py::arg("joint_names"), py::arg("ros"),
           py::arg("feedback_publish_period") = 0.1);

}
// clang-format off
}  // namespace drake_ros_py
// clang-format on
}  // namespace drake_ros
