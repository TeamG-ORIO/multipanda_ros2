"""Hardware launch file.

Intended for use with real Franka arms.  Manipulation and perception
providers should be replaced with real implementations.

Currently the manipulation_provider and perception_providers still
default to their mock/yaml stubs — replace these arguments when the
real backends are available.

Usage:
  ros2 launch orio_task_supervisor hardware_mode.launch.py
  ros2 launch orio_task_supervisor hardware_mode.launch.py \
      grasp_provider:=vision label_provider:=vision \
      execution_mode:=continuous
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_supervisor = get_package_share_directory('orio_task_supervisor')
    pkg_perception = get_package_share_directory('orio_perception_interface')

    args = [
        DeclareLaunchArgument('grasp_provider',        default_value='yaml',
                              description='Grasp pose provider: yaml | vision'),
        DeclareLaunchArgument('label_provider',        default_value='yaml',
                              description='Label pose provider: yaml | vision'),
        DeclareLaunchArgument('manipulation_provider', default_value='mock',
                              description='Manipulation backend (mock until MoveIt integration)'),
        DeclareLaunchArgument('execution_mode',        default_value='discrete',
                              description='Execution mode: discrete | continuous'),
    ]

    supervisor_config = os.path.join(pkg_supervisor, 'config', 'supervisor.yaml')
    perception_config = os.path.join(pkg_perception, 'config', 'mock_poses.yaml')

    task_supervisor_node = Node(
        package='orio_task_supervisor',
        executable='task_supervisor',
        name='task_supervisor',
        parameters=[
            supervisor_config,
            {'execution_mode': LaunchConfiguration('execution_mode')},
        ],
        output='screen',
    )

    manipulation_server_node = Node(
        package='orio_manipulation_interface',
        executable='manipulation_server',
        name='manipulation_server',
        output='screen',
        # TODO: when real MoveIt backend is ready, pass the robot description
        # and controller parameters here.
    )

    grasp_pose_server_node = Node(
        package='orio_perception_interface',
        executable='grasp_pose_server',
        name='grasp_pose_server',
        parameters=[
            perception_config,
            {'grasp_provider': LaunchConfiguration('grasp_provider')},
        ],
        output='screen',
    )

    label_pose_server_node = Node(
        package='orio_perception_interface',
        executable='label_pose_server',
        name='label_pose_server',
        parameters=[
            perception_config,
            {'label_provider': LaunchConfiguration('label_provider')},
        ],
        output='screen',
    )

    return LaunchDescription(args + [
        task_supervisor_node,
        manipulation_server_node,
        grasp_pose_server_node,
        label_pose_server_node,
    ])
