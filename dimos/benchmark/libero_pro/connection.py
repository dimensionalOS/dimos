"""Ordinary DimOS Module adapting the LIBERO policy gRPC interface."""

from __future__ import annotations

import threading
import time
from typing import Any

import grpc  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray

from dimos.benchmark.libero_pro.proto import libero_pro_pb2 as pb2, libero_pro_pb2_grpc as pb2_grpc
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.memory2.module import Recorder
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.simulation.engines.mujoco_shm import ManipShmWriter, shm_key_from_path

CAMERA_OPTICAL_FRAMES = {
    "agentview": "agentview_optical",
    "robot0_eye_in_hand": "eye_in_hand_optical",
}


class LiberoConnectionConfig(ModuleConfig):
    endpoint: str
    discovery_address: str
    startup_timeout_s: float = 30.0


class LiberoConnection(Module):
    """Publish permitted Panda observations and forward ordinary joint commands."""

    config: LiberoConnectionConfig
    joint_state: Out[JointState]
    tf: Out[TFMessage]
    agentview_color_image: Out[Image]
    agentview_depth_image: Out[Image]
    agentview_camera_info: Out[CameraInfo]
    eye_in_hand_color_image: Out[Image]
    eye_in_hand_depth_image: Out[Image]
    eye_in_hand_camera_info: Out[CameraInfo]

    def __init__(self, endpoint: str, discovery_address: str, **kwargs: Any) -> None:
        super().__init__(endpoint=endpoint, discovery_address=discovery_address, **kwargs)
        self._channel: grpc.Channel | None = None
        self._stub: pb2_grpc.PolicyInterfaceStub | None = None
        self._shm: ManipShmWriter | None = None
        self._stop = threading.Event()
        self._watch_thread: threading.Thread | None = None
        self._command_thread: threading.Thread | None = None
        self._sequence = 0
        self._latest_gripper = 0.04
        self._arm_target: NDArray[np.float64] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._channel = grpc.insecure_channel(self.config.endpoint)
        self._stub = pb2_grpc.PolicyInterfaceStub(self._channel)
        grpc.channel_ready_future(self._channel).result(timeout=self.config.startup_timeout_s)
        self._stub.GetHealth(pb2.Empty(), timeout=self.config.startup_timeout_s)
        self._shm = ManipShmWriter(shm_key_from_path(self.config.discovery_address))
        self._watch_thread = threading.Thread(target=self._watch, daemon=True)
        self._command_thread = threading.Thread(target=self._pump_commands, daemon=True)
        self._watch_thread.start()
        self._command_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._channel is not None:
            self._channel.close()
        for thread in (self._watch_thread, self._command_thread):
            if thread is not None:
                thread.join(timeout=2)
        if self._shm is not None:
            self._shm.cleanup()
            self._shm = None
        super().stop()

    def _watch(self) -> None:
        assert self._stub is not None
        try:
            for snapshot in self._stub.WatchState(pb2.WatchRequest()):
                if self._stop.is_set():
                    return
                names = [f"panda/joint{i + 1}" for i in range(7)] + ["panda/gripper"]
                positions = [*snapshot.joint_position, snapshot.gripper_position]
                if self._arm_target is None:
                    self._arm_target = np.asarray(snapshot.joint_position, dtype=np.float64)
                velocities = [*snapshot.joint_velocity, 0.0]
                self.joint_state.publish(
                    JointState(
                        ts=snapshot.timestamp_s,
                        frame_id="panda",
                        name=names,
                        position=positions,
                        velocity=velocities,
                        effort=[0.0] * 8,
                    )
                )
                if self._shm is not None:
                    self._shm.write_joint_state(
                        list(snapshot.joint_position), list(snapshot.joint_velocity), [0.0] * 7
                    )
                    self._shm.write_gripper_state(snapshot.gripper_position)
                    self._shm.signal_ready(num_joints=8)
                transforms = []
                for frame in snapshot.cameras:
                    image, depth, info, transform = _decode_camera(frame, snapshot.timestamp_s)
                    transforms.append(transform)
                    if frame.camera == "agentview":
                        self.agentview_color_image.publish(image)
                        self.agentview_depth_image.publish(depth)
                        self.agentview_camera_info.publish(info)
                    elif frame.camera == "robot0_eye_in_hand":
                        self.eye_in_hand_color_image.publish(image)
                        self.eye_in_hand_depth_image.publish(depth)
                        self.eye_in_hand_camera_info.publish(info)
                if transforms:
                    self.tf.publish(TFMessage(*transforms))
        except grpc.RpcError:
            if self._stop.is_set():
                return
            raise

    def _pump_commands(self) -> None:
        while not self._stop.wait(0.005):
            if self._shm is None or self._stub is None:
                continue
            arm = self._shm.read_position_command(7)
            gripper = self._shm.read_gripper_command()
            if arm is None and gripper is None:
                continue
            if gripper is not None:
                self._latest_gripper = float(gripper)
            arm = self._resolve_arm_target(arm)
            if arm is None:
                continue
            self._sequence += 1
            self._stub.SetJointTargets(
                pb2.JointTargets(
                    joint_position=list(arm),
                    gripper_position=self._latest_gripper,
                    sequence=self._sequence,
                ),
                timeout=2,
            )
            time.sleep(0)

    def _resolve_arm_target(
        self,
        arm: NDArray[np.float64] | None,
    ) -> NDArray[np.float64] | None:
        """Retain the commanded arm target across gripper-only updates."""
        if arm is not None:
            self._arm_target = np.asarray(arm, dtype=np.float64).copy()
        if self._arm_target is None:
            return None
        return self._arm_target.copy()


class LiberoRecorder(Recorder):
    joint_state: In[JointState]
    agentview_color_image: In[Image]
    agentview_depth_image: In[Image]
    agentview_camera_info: In[CameraInfo]
    eye_in_hand_color_image: In[Image]
    eye_in_hand_depth_image: In[Image]
    eye_in_hand_camera_info: In[CameraInfo]


def _decode_camera(
    frame: pb2.CameraFrame,
    timestamp_s: float,
) -> tuple[Image, Image, CameraInfo, Transform]:
    if frame.camera not in CAMERA_OPTICAL_FRAMES:
        raise ValueError(f"Unknown LIBERO camera: {frame.camera}")
    if len(frame.intrinsic) != 9:
        raise ValueError("camera intrinsic must contain a 3x3 matrix")
    if len(frame.camera_to_robot_base) != 16:
        raise ValueError("camera pose must contain a 4x4 matrix")
    frame_id = CAMERA_OPTICAL_FRAMES[frame.camera]
    color = np.frombuffer(frame.rgb, dtype=np.uint8).reshape(frame.height, frame.width, 3)
    depth = np.frombuffer(frame.depth_meters, dtype=np.float32).reshape(frame.height, frame.width)
    intrinsic = list(frame.intrinsic)
    info = CameraInfo(
        height=frame.height,
        width=frame.width,
        distortion_model="plumb_bob",
        D=[0.0] * 5,
        K=intrinsic,
        P=[
            intrinsic[0],
            intrinsic[1],
            intrinsic[2],
            0.0,
            intrinsic[3],
            intrinsic[4],
            intrinsic[5],
            0.0,
            intrinsic[6],
            intrinsic[7],
            intrinsic[8],
            0.0,
        ],
        frame_id=frame_id,
        ts=timestamp_s,
    )
    transform = Transform.from_matrix(
        np.asarray(frame.camera_to_robot_base, dtype=np.float64).reshape(4, 4),
        ts=timestamp_s,
        frame_id="world",
        child_frame_id=frame_id,
    )
    return (
        Image(
            data=color,
            format=ImageFormat.RGB,
            frame_id=frame_id,
            ts=timestamp_s,
        ),
        Image(
            data=depth,
            format=ImageFormat.DEPTH,
            frame_id=frame_id,
            ts=timestamp_s,
        ),
        info,
        transform,
    )
