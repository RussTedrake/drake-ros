#!/usr/bin/env python3
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class IiwaJointTrajectoryClient(Node):
    def __init__(self):
        super().__init__("iiwa_joint_trajectory_client")
        self._action_client = ActionClient(
            self, FollowJointTrajectory, "iiwa_joint_trajectory"
        )

    def send_goal(self):
        plant = MultibodyPlant(0.0)
        iiwa = Parser(plant).AddModelsFromUrl(
            "package://drake/manipulation/models/iiwa_description/urdf/iiwa14_no_collision.urdf"
        )[0]
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
        plant.Finalize()
        joint_indices = plant.GetJointIndices(iiwa)
        joint_names = [
            plant.get_joint(ind).name()
            for ind in joint_indices
            if plant.get_joint(ind).num_positions() == 1
        ]
        # Initial conditions from the ManipulationStation clutter clearing demo:
        pos = np.array([-1.57, 0.1, 0, -1.2, 0, 1.6, 0])

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names

        for t in np.arange(0, 2 * np.pi, 0.1):
            pos[0] = -np.pi / 4 - np.pi / 4 * np.cos(t)
            sec = int(np.floor(t))
            nanosec = int((t - sec) * 1e9)
            goal_msg.trajectory.points.append(
                JointTrajectoryPoint(
                    positions=pos,
                    time_from_start=Duration(sec=sec, nanosec=nanosec),
                )
            )

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.error_code))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = IiwaJointTrajectoryClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
