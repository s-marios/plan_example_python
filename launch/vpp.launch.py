
"""
A launch file for running the motion planning python api tutorial
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, IfElseSubstitution, EqualsSubstitution, TextSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from ros2launch.api.api import print_arguments_of_launch_description

import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_ip = LaunchConfiguration('robot_ip', default='fake')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')


    movefile = IfElseSubstitution(EqualsSubstitution(
            robot_ip, TextSubstitution(text='fake')),
            "lite6_moveit_fake.launch.py",
            "lite6_moveit_realmove.launch.py",
            )


    lite6_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch',  movefile])),
        launch_arguments={
            'robot_ip': robot_ip,
            'hw_ns': hw_ns,
        }.items(),
    )

    spawn_exec_file = DeclareLaunchArgument(
        "object_spawner_exec",
        default_value="object_spawner",
        description="Spawn virtual objects for the robot to pick up",
        choices=["object_spawner", "random_object_spawner"]
        )

    obj_spawn_node = Node(
        name="obj_spawn",
        package="plan_example_python",
        executable=LaunchConfiguration("object_spawner_exec"),
        output="both",
        )


    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="lite6", package_name="plan_example_python"
            )
        .robot_description(file_path="config/lite6.urdf")
        .robot_description_semantic(file_path="config/lite6.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("plan_example_python")
            + "/config/motion_planning_python_api_tutorial.yaml"
            )
        .to_moveit_configs()
        )

    pick_and_place_exec = DeclareLaunchArgument(
        "pick_place",
        default_value="pick_place",
        description="The main pick and place task",
        )

    pick_place_node = Node(
        name="pick_place_moveit_py",
        package="plan_example_python",
        executable=LaunchConfiguration("pick_place"),
        output="both",
        parameters=[moveit_config.to_dict()],
        )

    return LaunchDescription(
        [
            spawn_exec_file,
            obj_spawn_node,
            pick_and_place_exec,
            lite6_launch,
            pick_place_node
            ]
        )
