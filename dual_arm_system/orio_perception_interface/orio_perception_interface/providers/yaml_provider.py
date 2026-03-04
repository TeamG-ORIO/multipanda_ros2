"""YAML-based mock perception provider.

Reads static poses from flat ROS2 parameters declared as:

  mock_grasp_pose.position:    [x, y, z]
  mock_grasp_pose.orientation: [roll_deg, pitch_deg, yaw_deg]
  mock_pre_grasp_pose.position:    [x, y, z]
  mock_pre_grasp_pose.orientation: [roll_deg, pitch_deg, yaw_deg]

  mock_label_pose.position:    [x, y, z]
  mock_label_pose.orientation: [roll_deg, pitch_deg, yaw_deg]
  mock_pre_label_pose.position:    [x, y, z]
  mock_pre_label_pose.orientation: [roll_deg, pitch_deg, yaw_deg]

Orientations are Euler angles in degrees (ZYX convention) and are
converted to quaternions before being placed in the message.
"""

from geometry_msgs.msg import Pose, Point

from orio_dual_arm_msgs.srv import GetGraspPose, GetLabelPose
from orio_perception_interface.utils import euler_deg_to_quaternion
from orio_perception_interface.providers.base_provider import (
    GraspPoseProvider,
    LabelPoseProvider,
)


def _build_pose(node, prefix: str) -> Pose:
    """Read <prefix>.position and <prefix>.orientation params and build a Pose."""
    pos = node.get_parameter(f'{prefix}.position').value
    ori = node.get_parameter(f'{prefix}.orientation').value
    pose = Pose()
    pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
    pose.orientation = euler_deg_to_quaternion(float(ori[0]), float(ori[1]), float(ori[2]))
    return pose


class YamlGraspPoseProvider(GraspPoseProvider):
    def __init__(self, node):
        self._node = node

    def get_grasp_pose(
        self,
        request: GetGraspPose.Request,
        response: GetGraspPose.Response,
    ) -> GetGraspPose.Response:
        try:
            response.result.pose           = _build_pose(self._node, 'mock_grasp_pose')
            response.result.pre_grasp_pose = _build_pose(self._node, 'mock_pre_grasp_pose')
            response.result.confidence     = 1.0
            response.result.success        = True
            response.result.error_message  = ''
        except Exception as exc:  # noqa: BLE001
            msg = f'YamlGraspPoseProvider error: {exc}'
            self._node.get_logger().error(msg)
            response.result.success       = False
            response.result.error_message = msg
        return response


class YamlLabelPoseProvider(LabelPoseProvider):
    def __init__(self, node):
        self._node = node

    def get_label_pose(
        self,
        request: GetLabelPose.Request,
        response: GetLabelPose.Response,
    ) -> GetLabelPose.Response:
        try:
            response.result.pose           = _build_pose(self._node, 'mock_label_pose')
            response.result.pre_grasp_pose = _build_pose(self._node, 'mock_pre_label_pose')
            response.result.confidence     = 1.0
            response.result.success        = True
            response.result.error_message  = ''
        except Exception as exc:  # noqa: BLE001
            msg = f'YamlLabelPoseProvider error: {exc}'
            self._node.get_logger().error(msg)
            response.result.success       = False
            response.result.error_message = msg
        return response
