import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    robot_name = "my_robot"
    package_name = robot_name + "_description"
    robot_description = os.path.join(get_package_share_directory(
        package_name), "urdf", robot_name + ".urdf")
    robot_description_config = xacro.process_file(robot_description)

    controller_config = os.path.join(
        get_package_share_directory(
            package_name), "controllers", "controllers.yaml"
    )

    share_dir = get_package_share_directory(robot_name+'_bringup')


    ld = LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),

        #Node(
        #    package="controller_manager",
        #    executable="spawner",
        #    arguments=["velocity_controller", "-c", "/controller_manager"],
        #),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        ),])
    return ld