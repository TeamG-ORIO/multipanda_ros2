import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        # Subscribing to the topics confirmed in your setup
        self.depth_sub = self.create_subscription(Image, '/mujoco_server/cameras/cam_table_1/depth/image_raw', self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(Image, '/mujoco_server/cameras/cam_table_1/rgb/image_raw', self.rgb_callback, 10)
        self.latest_rgb = None

    def rgb_callback(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        if self.latest_rgb is None: return
        
        # Convert 32FC1 to numpy array
        depth_data = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        
        # Save RGB
        cv2.imwrite('workspace_rgb.jpg', self.latest_rgb)
        
        # Save Depth as raw float data (Best for info preservation)
        np.save('workspace_depth.npy', depth_data)
        
        # Save Depth as a viewable 16-bit PNG (scaled to millimeters)
        depth_mm = (depth_data * 1000).astype(np.uint16)
        cv2.imwrite('workspace_depth_16bit.png', depth_mm)
        
        self.get_logger().info('Saved RGB and Depth info!')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ImageSaver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()