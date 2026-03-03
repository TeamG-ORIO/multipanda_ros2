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
    arm_id = LaunchConfiguration('arm_id', default='panda')
    load_gripper = True

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'sim', 'panda_arm_sim.urdf.xacro'
    )
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ', franka_xacro_file,
        ' arm_id:=', arm_id,
        ' hand:=', str(load_gripper).lower(),
    ])
    robot_description = {'robot_description': robot_description_config}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_moveit_config'),
        'srdf', 'panda_arm.srdf.xacro'
    )
    robot_description_semantic_config = Command([
        FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file,
        ' hand:=', str(load_gripper).lower(),
    ])
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml('franka_moveit_config', 'config/kinematics.yaml')

    pick_and_place_params = os.path.join(
        get_package_share_directory('move_to_pose'),
        'config', 'pick_and_place.yaml'
    )

    pick_and_place_node = Node(
        package='move_to_pose',
        executable='pick_and_place',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            pick_and_place_params,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_id',
            default_value='panda',
            description='Arm ID used to resolve the URDF and SRDF',
        ),
        pick_and_place_node,
    ])
