"""Tests for public LIBERO observation conversion."""

import grpc
import numpy as np

from dimos.benchmark.libero_pro.connection import LiberoConnection, _decode_camera
from dimos.benchmark.libero_pro.proto import libero_pro_pb2 as pb2
from dimos.msgs.sensor_msgs.Image import ImageFormat


def test_camera_messages_keep_the_shared_simulator_timestamp() -> None:
    pixels = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    depth = np.array([[0.5, 0.75], [1.0, 1.25]], dtype=np.float32)
    intrinsic = [100.0, 0.0, 1.0, 0.0, 101.0, 1.0, 0.0, 0.0, 1.0]
    camera_to_robot_base = np.eye(4)
    camera_to_robot_base[:3, 3] = [0.1, 0.2, 0.3]
    frame = pb2.CameraFrame(
        camera="agentview",
        width=2,
        height=2,
        rgb=pixels.tobytes(),
        depth_meters=depth.tobytes(),
        intrinsic=intrinsic,
        camera_to_robot_base=camera_to_robot_base.reshape(-1),
    )

    image, depth_image, info, transform = _decode_camera(frame, 123.5)

    assert image.ts == 123.5
    assert image.frame_id == "agentview_optical"
    assert image.format is ImageFormat.RGB
    assert np.array_equal(image.data, pixels)
    assert depth_image.ts == 123.5
    assert depth_image.frame_id == "agentview_optical"
    assert depth_image.format is ImageFormat.DEPTH
    assert np.array_equal(depth_image.data, depth)
    assert info.ts == 123.5
    assert info.frame_id == "agentview_optical"
    assert info.K == intrinsic
    assert transform.ts == 123.5
    assert transform.frame_id == "world"
    assert transform.child_frame_id == "agentview_optical"
    assert np.allclose(transform.to_matrix(), camera_to_robot_base)


def test_policy_snapshot_has_no_privileged_evaluation_fields() -> None:
    fields = set(pb2.RobotSnapshot.DESCRIPTOR.fields_by_name)

    assert fields == {
        "tick",
        "timestamp_s",
        "joint_position",
        "joint_velocity",
        "gripper_position",
        "cameras",
    }
    assert fields.isdisjoint({"reward", "success", "goal_predicates", "object_poses"})


def test_watch_stream_cancellation_is_clean_during_stop() -> None:
    class CancelledStub:
        def WatchState(self, _request: pb2.WatchRequest):  # type: ignore[no-untyped-def]
            raise grpc.RpcError("channel closed")

    connection = LiberoConnection(endpoint="unused", discovery_address="unused")
    try:
        connection._stub = CancelledStub()  # type: ignore[assignment]
        connection._stop.set()
        connection._watch()
    finally:
        connection.stop()


def test_gripper_only_update_retains_last_commanded_arm_target() -> None:
    connection = LiberoConnection(endpoint="unused", discovery_address="unused")
    try:
        commanded = np.arange(7, dtype=np.float64)

        assert np.array_equal(connection._resolve_arm_target(commanded), commanded)

        commanded[:] = -1.0
        assert np.array_equal(connection._resolve_arm_target(None), np.arange(7))
    finally:
        connection.stop()
