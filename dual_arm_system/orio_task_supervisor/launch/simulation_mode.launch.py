"""Simulation / mock launch file.

All providers use mock/YAML implementations.  No real hardware required.

Usage:
  ros2 launch orio_task_supervisor simulation_mode.launch.py
  ros2 launch orio_task_supervisor simulation_mode.launch.py \
      grasp_provider:=yaml label_provider:=yaml \
      manipulation_provider:=mock execution_mode:=discrete
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_supervisor  = get_package_share_directory('orio_task_supervisor')
    pkg_perception  = get_package_share_directory('orio_perception_interface')
    pkg_config      = get_package_share_directory('orio_task_supervisor')

    # ── Launch arguments ───────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('grasp_provider',        default_value='yaml',
                              description='Grasp pose provider: yaml | vision'),
        DeclareLaunchArgument('label_provider',        default_value='yaml',
                              description='Label pose provider: yaml | vision'),
        DeclareLaunchArgument('manipulation_provider', default_value='mock',
                              description='Manipulation backend: mock (only option for now)'),
        DeclareLaunchArgument('execution_mode',        default_value='discrete',
                              description='Execution mode: discrete | continuous'),
    ]

    supervisor_config  = os.path.join(pkg_supervisor, 'config', 'supervisor.yaml')
    perception_config  = os.path.join(pkg_perception, 'config', 'mock_poses.yaml')

    # ── Nodes ──────────────────────────────────────────────────────────────
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
