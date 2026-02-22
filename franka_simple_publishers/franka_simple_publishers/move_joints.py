#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import LoadController, SwitchController, ConfigureController
import time


class JointMover(Node):
    """
    A simple node to move robot joints in MuJoCo simulation.
    Loads and activates the joint_impedance_example_controller.
    """

    def __init__(self):
        super().__init__('joint_mover')
        
        # Create service clients
        self.load_controller_client = self.create_client(
            LoadController, 
            '/controller_manager/load_controller'
        )
        self.configure_controller_client = self.create_client(
            ConfigureController,
            '/controller_manager/configure_controller'
        )
        self.switch_controller_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )
        
        self.get_logger().info('Joint Mover initialized')
        self.get_logger().info('Loading and activating joint_impedance_example_controller...')
        
        # Wait for services
        while not self.load_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for load_controller service...')
        
        while not self.configure_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for configure_controller service...')
        
        while not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for switch_controller service...')
        
        # Load and activate controller
        self.setup_controller()

    def setup_controller(self):
        """Load, configure and activate the joint_impedance_example_controller."""
        # First, load the controller
        load_req = LoadController.Request()
        load_req.name = 'joint_impedance_example_controller'
        
        self.get_logger().info('Loading controller...')
        load_future = self.load_controller_client.call_async(load_req)
        rclpy.spin_until_future_complete(self, load_future)
        
        if load_future.result() is not None:
            self.get_logger().info('✓ Controller loaded successfully')
        else:
            self.get_logger().error('Failed to load controller')
            return
        
        # Configure the controller
        config_req = ConfigureController.Request()
        config_req.name = 'joint_impedance_example_controller'
        
        self.get_logger().info('Configuring controller...')
        config_future = self.configure_controller_client.call_async(config_req)
        rclpy.spin_until_future_complete(self, config_future)
        
        if config_future.result() is not None:
            self.get_logger().info('✓ Controller configured successfully')
        else:
            self.get_logger().error('Failed to configure controller')
            return
        
        # Then, switch to the controller
        switch_req = SwitchController.Request()
        switch_req.activate_controllers = ['joint_impedance_example_controller']
        switch_req.deactivate_controllers = []
        switch_req.strictness = 2  # STRICT
        switch_req.start_asap = True
        
        self.get_logger().info('Activating controller...')
        switch_future = self.switch_controller_client.call_async(switch_req)
        rclpy.spin_until_future_complete(self, switch_future)
        
        result = switch_future.result()
        if result is not None and result.ok:
            self.get_logger().info('✓ Controller activated! Robot is moving!')
        else:
            self.get_logger().error(f'Failed to activate controller: {result}')



def main(args=None):
    rclpy.init(args=args)
    mover = JointMover()
    
    try:
        print("\n" + "="*60)
        print("MuJoCo Robot Motion - Joint Impedance Controller")
        print("="*60)
        print("\nThe controller is now active!")
        print("Watch the MuJoCo window to see joints 3 and 4 oscillate.\n")
        
        print("Running for 15 seconds...")
        print("(Press Ctrl+C to stop)\n")
        
        # Keep the node running while the controller moves the robot
        for i in range(15):
            time.sleep(1)
            remaining = 15 - i - 1
            if remaining > 0:
                print(f"  Motion in progress... {remaining}s remaining")
        
        print("\n✓ Demonstration complete!")
        print("="*60 + "\n")
        
    except KeyboardInterrupt:
        print("\n\nMotion stopped by user")
    finally:
        mover.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
