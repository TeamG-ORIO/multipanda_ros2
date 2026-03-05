import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
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
    arm_id      = LaunchConfiguration('arm_id',      default='panda')
    use_cartesian = LaunchConfiguration('use_cartesian', default='false')
    load_gripper = True

    x     = LaunchConfiguration('x',     default='0.3')
    y     = LaunchConfiguration('y',     default='0.4')
    z     = LaunchConfiguration('z',     default='0.2')
    roll  = LaunchConfiguration('roll',  default='180.0')
    pitch = LaunchConfiguration('pitch', default='0.0')
    yaw   = LaunchConfiguration('yaw',   default='0.0')

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

    common_params = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        {
            'x': x, 'y': y, 'z': z,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
        },
        {'use_sim_time': True},
    ]

    move_to_pose_node = Node(
        package='move_to_pose',
        executable='move_to_pose',
        output='screen',
        parameters=common_params,
        condition=UnlessCondition(PythonExpression(["'", use_cartesian, "' == 'true'"])),
    )

    cart_move_to_pose_node = Node(
        package='move_to_pose',
        executable='cart_move_to_pose',
        output='screen',
        parameters=common_params,
        condition=IfCondition(PythonExpression(["'", use_cartesian, "' == 'true'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('arm_id', default_value='panda'),
        DeclareLaunchArgument(
            'use_cartesian',
            default_value='false',
            description='Use Cartesian planner (cart_move_to_pose) instead of the default joint-space planner',
        ),
        DeclareLaunchArgument('x',     default_value='0.3',   description='Target position x (m)'),
        DeclareLaunchArgument('y',     default_value='0.0',   description='Target position y (m)'),
        DeclareLaunchArgument('z',     default_value='0.2',   description='Target position z (m)'),
        DeclareLaunchArgument('roll',  default_value='180.0', description='Roll  angle in degrees (rotation about X)'),
        DeclareLaunchArgument('pitch', default_value='0.0',   description='Pitch angle in degrees (rotation about Y)'),
        DeclareLaunchArgument('yaw',   default_value='0.0',   description='Yaw   angle in degrees (rotation about Z)'),
        move_to_pose_node,
        cart_move_to_pose_node,
    ])
