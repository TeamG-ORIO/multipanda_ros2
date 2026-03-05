import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    arm_id_1 = 'mj_left'
    arm_id_2 = 'mj_right'
    load_gripper = True

    orio_dual_cart_params = LaunchConfiguration(
        'orio_dual_cart_params',
        default=os.path.join(
            get_package_share_directory('move_to_pose'),
            'config', 'orio_dual_cart_target_poses.yaml'
        )
    )

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'sim', 'orio_dual_panda_arm_sim.urdf.xacro'
    )
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ', franka_xacro_file,
        ' arm_id_1:=', arm_id_1,
        ' arm_id_2:=', arm_id_2,
        ' hand_1:=', str(load_gripper).lower(),
        ' hand_2:=', str(load_gripper).lower(),
    ])
    robot_description = {'robot_description': robot_description_config}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_moveit_config'),
        'srdf', 'dual_panda.srdf.xacro'
    )
    robot_description_semantic_config = Command([
        FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file,
        ' arm_id_1:=', arm_id_1,
        ' arm_id_2:=', arm_id_2,
        ' hand_1:=', str(load_gripper).lower(),
        ' hand_2:=', str(load_gripper).lower(),
    ])
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml('franka_moveit_config', 'config/kinematics.yaml')

    orio_dual_cart_node = Node(
        package='move_to_pose',
        executable='orio_dual_cart_move_to_pose',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            orio_dual_cart_params,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'orio_dual_cart_params',
            default_value=os.path.join(
                get_package_share_directory('move_to_pose'),
                'config', 'orio_dual_cart_target_poses.yaml'
            ),
            description='Path to the YAML file with target poses for both arms',
        ),
        orio_dual_cart_node,
    ])
