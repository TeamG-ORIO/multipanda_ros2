"""Abstract base class for perception providers."""

from abc import ABC, abstractmethod
from orio_dual_arm_msgs.srv import GetGraspPose, GetLabelPose


class GraspPoseProvider(ABC):
    """Interface every grasp-pose backend must implement."""

    @abstractmethod
    def get_grasp_pose(
        self,
        request: GetGraspPose.Request,
        response: GetGraspPose.Response,
    ) -> GetGraspPose.Response:
        ...


class LabelPoseProvider(ABC):
    """Interface every label-pose backend must implement."""

    @abstractmethod
    def get_label_pose(
        self,
        request: GetLabelPose.Request,
        response: GetLabelPose.Response,
    ) -> GetLabelPose.Response:
        ...
