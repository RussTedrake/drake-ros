#!/usr/bin/env python3
import argparse

import numpy as np

import drake_ros.core
from drake_ros.action import FollowJointTrajectoryServerSystem
from drake_ros.core import RosInterfaceSystem
from drake_ros.viz import RvizVisualizer

from pydrake.examples import ManipulationStation
from pydrake.geometry import Meshcat, MeshcatVisualizer
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--simulation_sec",
        type=float,
        default=float("inf"),
        help="How many seconds to run the simulation",
    )
    args = parser.parse_args()

    builder = DiagramBuilder()

    drake_ros.core.init()
    ros_interface_system = builder.AddSystem(
        RosInterfaceSystem("iiwa_manipulator_node")
    )

    manipulation_station = builder.AddSystem(ManipulationStation())
    manipulation_station.SetupClutterClearingStation()
    manipulation_station.Finalize()

    plant = manipulation_station.get_multibody_plant()
    joint_indices = plant.GetJointIndices(plant.GetModelInstanceByName("iiwa"))
    joint_names = [
        plant.get_joint(ind).name()
        for ind in joint_indices
        if plant.get_joint(ind).num_positions() == 1
    ]

    # Add a FollowJointTrajectory action server.
    follow_traj = builder.AddSystem(
        FollowJointTrajectoryServerSystem(
            "iiwa_joint_trajectory",
            joint_names,
            ros_interface_system.get_ros_interface(),
            0.1,
        )
    )
    builder.Connect(
        follow_traj.get_output_port(),
        manipulation_station.GetInputPort("iiwa_position"),
    )
    builder.Connect(
        manipulation_station.GetOutputPort("iiwa_position_measured"),
        follow_traj.get_input_port(),
    )

    rviz_visualizer = builder.AddSystem(
        RvizVisualizer(ros_interface_system.get_ros_interface())
    )

    rviz_visualizer.RegisterMultibodyPlant(
        manipulation_station.get_multibody_plant()
    )
    rviz_visualizer.ComputeFrameHierarchy()

    # Also add a meshcat visualizer
    meshcat = Meshcat()
    MeshcatVisualizer.AddToBuilder(
        builder, manipulation_station.GetOutputPort("query_object"), meshcat
    )

    builder.Connect(
        manipulation_station.GetOutputPort("query_object"),
        rviz_visualizer.get_graph_query_input_port(),
    )

    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator_context = simulator.get_mutable_context()

    manipulation_station_context = diagram.GetMutableSubsystemContext(
        manipulation_station, simulator_context
    )

    # Fix gripper joints' position.
    manipulation_station.GetInputPort("wsg_position").FixValue(
        manipulation_station_context, np.zeros(1)
    )

    # Step the simulator in 0.1s intervals
    step = 0.1
    while simulator_context.get_time() < args.simulation_sec:
        next_time = min(
            simulator_context.get_time() + step,
            args.simulation_sec,
        )
        simulator.AdvanceTo(next_time)


if __name__ == "__main__":
    main()
