"""Label pose service server.

Selects the active provider at startup via the 'label_provider' parameter.

Launch argument:
  label_provider:=yaml    (default)
  label_provider:=vision  (not yet implemented – raises a clear error)
"""

import rclpy
from rclpy.node import Node

from orio_dual_arm_msgs.srv import GetLabelPose

from orio_perception_interface.providers.yaml_provider import YamlLabelPoseProvider
from orio_perception_interface.providers.vision_provider import VisionLabelPoseProvider

_PROVIDERS = {
    'yaml':   YamlLabelPoseProvider,
    'vision': VisionLabelPoseProvider,
}


class LabelPoseServer(Node):
    def __init__(self):
        super().__init__('label_pose_server')

        self.declare_parameter('label_provider', 'yaml')

        # Flat parameters for mock label pose (position + orientation in degrees)
        self.declare_parameter('mock_label_pose.position',    [0.30, -0.30, 0.20])
        self.declare_parameter('mock_label_pose.orientation', [180.0, 0.0, 0.0])
        self.declare_parameter('mock_pre_label_pose.position',    [0.30, -0.30, 0.35])
        self.declare_parameter('mock_pre_label_pose.orientation', [180.0, 0.0, 0.0])

        provider_name = self.get_parameter('label_provider').value
        if provider_name not in _PROVIDERS:
            self.get_logger().fatal(
                f"Unknown label_provider '{provider_name}'. "
                f"Valid options: {list(_PROVIDERS.keys())}")
            raise ValueError(f"Unknown label_provider: {provider_name}")

        self._provider = _PROVIDERS[provider_name](self)
        self.get_logger().info(f'Label pose server using provider: {provider_name}')

        self._srv = self.create_service(
            GetLabelPose,
            'perception/get_label_pose',
            self._handle_request)

    def _handle_request(
        self,
        request: GetLabelPose.Request,
        response: GetLabelPose.Response,
    ) -> GetLabelPose.Response:
        return self._provider.get_label_pose(request, response)


def main(args=None):
    rclpy.init(args=args)
    node = LabelPoseServer()
    rclpy.spin(node)
    rclpy.shutdown()
