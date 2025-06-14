from dimos.multiprocess.actors.base import deploy_actor
from dimos.multiprocess.actors.camera import CameraActor
from dimos.multiprocess.actors.frame import LatencyActor
from dimos.multiprocess.actors.recognition import FaceRecognitionActor
from dimos.multiprocess.actors.video import VideoActor

__all__ = ["CameraActor", "VideoActor", "LatencyActor", "FaceRecognitionActor", "deploy_actor"]
