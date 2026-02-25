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

    x   = LaunchConfiguration('x',   default='0.3')
    y   = LaunchConfiguration('y',   default='0.4')
    z   = LaunchConfiguration('z',   default='0.4')
    r00 = LaunchConfiguration('r00', default='1.0')
    r01 = LaunchConfiguration('r01', default='0.0')
    r02 = LaunchConfiguration('r02', default='0.0')
    r10 = LaunchConfiguration('r10', default='0.0')
    r11 = LaunchConfiguration('r11', default='-1.0')
    r12 = LaunchConfiguration('r12', default='0.0')
    r20 = LaunchConfiguration('r20', default='0.0')
    r21 = LaunchConfiguration('r21', default='0.0')
    r22 = LaunchConfiguration('r22', default='-1.0')

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

    move_to_pose_node = Node(
        package='move_to_pose',
        executable='move_to_pose',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {
                'x': x, 'y': y, 'z': z,
                'r00': r00, 'r01': r01, 'r02': r02,
                'r10': r10, 'r11': r11, 'r12': r12,
                'r20': r20, 'r21': r21, 'r22': r22,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('arm_id', default_value='panda'),
        DeclareLaunchArgument('x',   default_value='0.3',  description='Target position x (m)'),
        DeclareLaunchArgument('y',   default_value='0.4',  description='Target position y (m)'),
        DeclareLaunchArgument('z',   default_value='0.4',  description='Target position z (m)'),
        DeclareLaunchArgument('r00', default_value='1.0',  description='Rotation matrix [0,0]'),
        DeclareLaunchArgument('r01', default_value='0.0',  description='Rotation matrix [0,1]'),
        DeclareLaunchArgument('r02', default_value='0.0',  description='Rotation matrix [0,2]'),
        DeclareLaunchArgument('r10', default_value='0.0',  description='Rotation matrix [1,0]'),
        DeclareLaunchArgument('r11', default_value='-1.0', description='Rotation matrix [1,1]'),
        DeclareLaunchArgument('r12', default_value='0.0',  description='Rotation matrix [1,2]'),
        DeclareLaunchArgument('r20', default_value='0.0',  description='Rotation matrix [2,0]'),
        DeclareLaunchArgument('r21', default_value='0.0',  description='Rotation matrix [2,1]'),
        DeclareLaunchArgument('r22', default_value='-1.0', description='Rotation matrix [2,2]'),
        move_to_pose_node,
    ])
