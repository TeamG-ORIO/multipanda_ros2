#!/usr/bin/env python3
"""
Move the Franka Panda end-effector to a target Cartesian pose using MoveIt 2.

Usage (after sourcing the workspace):
  ros2 run franka_simple_publishers move_to_pose \
      --ros-args \
      -p x:=0.4 -p y:=0.0 -p z:=0.5 \
      -p qx:=0.0 -p qy:=0.0 -p qz:=0.0 -p qw:=1.0 \
      -p arm_group:=panda_arm \
      -p ee_link:=panda_hand_tcp \
      -p planning_time:=5.0

All parameters have defaults so the node can be run without any arguments.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


class MoveToPose(Node):

    def __init__(self):
        super().__init__('move_to_pose')

        # Declare parameters with sensible defaults
        self.declare_parameter('x', 0.4)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.5)
        self.declare_parameter('qx', 0.0)
        self.declare_parameter('qy', 0.7071068)
        self.declare_parameter('qz', 0.0)
        self.declare_parameter('qw', 0.7071068)
        self.declare_parameter('arm_group', 'panda_arm')
        self.declare_parameter('ee_link', 'panda_hand_tcp')
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('reference_frame', 'panda_link0')

    def run(self):
        x   = self.get_parameter('x').value
        y   = self.get_parameter('y').value
        z   = self.get_parameter('z').value
        qx  = self.get_parameter('qx').value
        qy  = self.get_parameter('qy').value
        qz  = self.get_parameter('qz').value
        qw  = self.get_parameter('qw').value
        arm_group       = self.get_parameter('arm_group').value
        ee_link         = self.get_parameter('ee_link').value
        planning_time   = self.get_parameter('planning_time').value
        reference_frame = self.get_parameter('reference_frame').value

        self.get_logger().info(
            f'Target pose → position: ({x:.3f}, {y:.3f}, {z:.3f})  '
            f'orientation quat: ({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})'
        )
        self.get_logger().info(
            f'Group: {arm_group}  |  EE link: {ee_link}  |  '
            f'Frame: {reference_frame}  |  Planning time: {planning_time}s'
        )

        # Initialise MoveItPy (reuses the node's executor)
        moveit = MoveItPy(node_name='move_to_pose')
        arm = moveit.get_planning_component(arm_group)

        # Build target pose
        target = PoseStamped()
        target.header.frame_id = reference_frame
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw

        # Plan
        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=target, pose_link=ee_link)

        self.get_logger().info('Planning...')
        plan_result = arm.plan()

        if not plan_result:
            self.get_logger().error('Planning failed — no solution found.')
            return False

        self.get_logger().info('Plan found. Executing...')
        moveit.execute(plan_result.trajectory, controllers=[])
        self.get_logger().info('Execution complete.')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPose()

    try:
        success = node.run()
        if not success:
            raise SystemExit(1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
