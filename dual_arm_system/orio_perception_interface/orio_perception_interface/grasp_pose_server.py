"""Grasp pose service server.

Selects the active provider at startup via the 'grasp_provider' parameter.

Launch argument:
  grasp_provider:=yaml    (default)
  grasp_provider:=vision  (not yet implemented – raises a clear error)
"""

import rclpy
from rclpy.node import Node

from orio_dual_arm_msgs.srv import GetGraspPose

from orio_perception_interface.providers.yaml_provider import YamlGraspPoseProvider
from orio_perception_interface.providers.vision_provider import VisionGraspPoseProvider

_PROVIDERS = {
    'yaml':   YamlGraspPoseProvider,
    'vision': VisionGraspPoseProvider,
}


class GraspPoseServer(Node):
    def __init__(self):
        super().__init__('grasp_pose_server')

        self.declare_parameter('grasp_provider', 'yaml')

        # Flat parameters for mock grasp pose (position + orientation in degrees)
        self.declare_parameter('mock_grasp_pose.position',    [0.45, 0.0, 0.20])
        self.declare_parameter('mock_grasp_pose.orientation', [180.0, 0.0, 0.0])
        self.declare_parameter('mock_pre_grasp_pose.position',    [0.45, 0.0, 0.35])
        self.declare_parameter('mock_pre_grasp_pose.orientation', [180.0, 0.0, 0.0])

        provider_name = self.get_parameter('grasp_provider').value
        if provider_name not in _PROVIDERS:
            self.get_logger().fatal(
                f"Unknown grasp_provider '{provider_name}'. "
                f"Valid options: {list(_PROVIDERS.keys())}")
            raise ValueError(f"Unknown grasp_provider: {provider_name}")

        self._provider = _PROVIDERS[provider_name](self)
        self.get_logger().info(f'Grasp pose server using provider: {provider_name}')

        self._srv = self.create_service(
            GetGraspPose,
            'perception/get_grasp_pose',
            self._handle_request)

    def _handle_request(
        self,
        request: GetGraspPose.Request,
        response: GetGraspPose.Response,
    ) -> GetGraspPose.Response:
        return self._provider.get_grasp_pose(request, response)


def main(args=None):
    rclpy.init(args=args)
    node = GraspPoseServer()
    rclpy.spin(node)
    rclpy.shutdown()
