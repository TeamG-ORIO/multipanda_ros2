"""Stub for a future vision-based perception provider.

This file exists as a placeholder so that the launch argument
  grasp_provider:=vision
  label_provider:=vision
produces a clear error rather than a silent failure.
"""

from orio_dual_arm_msgs.srv import GetGraspPose, GetLabelPose
from orio_perception_interface.providers.base_provider import (
    GraspPoseProvider,
    LabelPoseProvider,
)
import rclpy


class VisionGraspPoseProvider(GraspPoseProvider):
    def __init__(self, node):
        self._node = node

    def get_grasp_pose(
        self,
        request: GetGraspPose.Request,
        response: GetGraspPose.Response,
    ) -> GetGraspPose.Response:
        self._node.get_logger().error(
            'VisionGraspPoseProvider is not yet implemented. '
            'Use grasp_provider:=yaml for mock operation.')
        raise NotImplementedError('Vision grasp provider not implemented.')


class VisionLabelPoseProvider(LabelPoseProvider):
    def __init__(self, node):
        self._node = node

    def get_label_pose(
        self,
        request: GetLabelPose.Request,
        response: GetLabelPose.Response,
    ) -> GetLabelPose.Response:
        self._node.get_logger().error(
            'VisionLabelPoseProvider is not yet implemented. '
            'Use label_provider:=yaml for mock operation.')
        raise NotImplementedError('Vision label provider not implemented.')
