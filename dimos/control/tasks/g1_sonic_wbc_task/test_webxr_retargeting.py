# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import (
    DEFAULT_ANGLES_ONNX,
    WRIST_ONNX_INDICES,
)
from dimos.control.tasks.g1_sonic_wbc_task.streamed_motion import StreamedMotionMerger
from dimos.control.tasks.g1_sonic_wbc_task.webxr_retargeting import (
    SMPL_WEBXR_JOINTS,
    WEBXR_TO_SONIC,
    IncompleteBodyPoseError,
    PoseStreamError,
    WebXRSonicPoseStream,
    WebXRSonicRetargeter,
    _interpolate_quaternion_wxyz,
)
from dimos.teleop.webxr.body_tracking import BodyJointPose, BodyTrackingSnapshot


def _webxr_quaternion_for_sonic_rotation(rotation: Rotation) -> tuple[float, ...]:
    matrix = WEBXR_TO_SONIC.T @ rotation.as_matrix() @ WEBXR_TO_SONIC
    return tuple(float(value) for value in Rotation.from_matrix(matrix).as_quat())


def _snapshot(
    *,
    capture_time_s: float = 10.0,
    rotations: dict[str, Rotation] | None = None,
    position_scale: float = 1.0,
    omitted: frozenset[str] = frozenset(),
) -> BodyTrackingSnapshot:
    rotations = rotations or {}
    joints = {
        name: BodyJointPose(
            position=(
                position_scale * float(index),
                position_scale * float(2 * index),
                position_scale * float(-3 * index),
            ),
            orientation=_webxr_quaternion_for_sonic_rotation(
                rotations.get(name, Rotation.identity())
            ),
        )
        for index, name in enumerate(SMPL_WEBXR_JOINTS)
        if name not in omitted
    }
    return BodyTrackingSnapshot(
        type="body_tracking_snapshot",
        capture_time_s=capture_time_s,
        frame_id="local-floor",
        joints=joints,
    )


def test_retarget_uses_tracked_root_local_skeleton() -> None:
    frame = WebXRSonicRetargeter().retarget(_snapshot(), frame_index=7, t_now=1.0).fields

    assert frame["frame_index"].tolist() == [7]
    assert frame["joint_pos"].shape == (1, 29)
    assert frame["joint_vel"].shape == (1, 29)
    assert frame["body_quat_w"].tolist() == [[1.0, 0.0, 0.0, 0.0]]
    assert frame["smpl_joints"].shape == (1, 24, 3)
    assert frame["smpl_pose"].shape == (1, 21, 3)
    np.testing.assert_allclose(frame["smpl_joints"][0, 0], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(frame["smpl_joints"][0, 1], [3.0, -1.0, 2.0])
    np.testing.assert_allclose(frame["smpl_pose"], 0.0, atol=1e-7)
    np.testing.assert_allclose(frame["joint_pos"][0], DEFAULT_ANGLES_ONNX)

    merged = StreamedMotionMerger().merge(frame, current_playback_frame=0)
    assert merged.error is None
    assert merged.motion is not None
    assert merged.motion.encode_mode == 2


def test_retarget_preserves_operator_bone_lengths() -> None:
    retargeter = WebXRSonicRetargeter()

    normal = retargeter.retarget(_snapshot(position_scale=1.0), frame_index=0, t_now=1.0)
    tall = retargeter.retarget(_snapshot(position_scale=2.0), frame_index=1, t_now=1.1)

    np.testing.assert_allclose(tall.fields["smpl_joints"], 2.0 * normal.fields["smpl_joints"])


def test_retarget_derives_direct_wrist_targets_and_velocity() -> None:
    retargeter = WebXRSonicRetargeter()
    retargeter.retarget(_snapshot(), frame_index=0, t_now=1.0)
    rotations = {
        "left-hand-wrist": Rotation.from_euler("x", 0.3),
        "right-hand-wrist": Rotation.from_euler("x", 0.3),
    }

    frame = retargeter.retarget(_snapshot(rotations=rotations), frame_index=1, t_now=1.1).fields

    assert frame["joint_pos"][0, WRIST_ONNX_INDICES[0]] == pytest.approx(0.3)
    assert frame["joint_pos"][0, WRIST_ONNX_INDICES[1]] == pytest.approx(0.3)
    assert frame["joint_vel"][0, WRIST_ONNX_INDICES[0]] == pytest.approx(3.0)
    assert frame["joint_vel"][0, WRIST_ONNX_INDICES[1]] == pytest.approx(3.0)
    expected = DEFAULT_ANGLES_ONNX.copy()
    expected[WRIST_ONNX_INDICES[:2]] = 0.3
    np.testing.assert_allclose(frame["joint_pos"][0], expected, atol=1e-7)


def test_pose_stream_waits_for_two_resampled_frames() -> None:
    stream = WebXRSonicPoseStream()

    for index in range(2):
        rotations = {"left-hand-wrist": Rotation.from_euler("x", 0.3)} if index == 1 else None
        stream.push(
            _snapshot(capture_time_s=10.0 + 0.02 * index, rotations=rotations),
            t_now=1.0 + 0.02 * index,
        )

    assert stream.ready is True
    assert stream.buffered_frames == 2
    assert stream.generation == 2
    fields = stream.fields()
    assert fields["frame_index"].tolist() == [0, 1]
    assert fields["smpl_joints"].shape == (2, 24, 3)
    assert fields["smpl_pose"].shape == (2, 21, 3)
    np.testing.assert_allclose(fields["smpl_joints"][:, 0], 0.0)
    np.testing.assert_allclose(fields["smpl_joints"][:, 1], [[3.0, -1.0, 2.0]] * 2)
    expected_joint_pos = np.stack([DEFAULT_ANGLES_ONNX, DEFAULT_ANGLES_ONNX])
    expected_joint_pos[1, WRIST_ONNX_INDICES[0]] = 0.3
    np.testing.assert_allclose(fields["joint_pos"], expected_joint_pos, atol=1e-7)
    assert fields["joint_vel"][1, WRIST_ONNX_INDICES[0]] == pytest.approx(15.0)


def test_pose_stream_interpolates_root_by_shortest_quaternion_path() -> None:
    left_xyzw = Rotation.from_euler("z", 170, degrees=True).as_quat()
    right_xyzw = Rotation.from_euler("z", -170, degrees=True).as_quat()
    left_wxyz = np.array([left_xyzw[3], *left_xyzw[:3]], dtype=np.float32)
    right_wxyz = np.array([right_xyzw[3], *right_xyzw[:3]], dtype=np.float32)

    middle_wxyz = _interpolate_quaternion_wxyz(left_wxyz, right_wxyz, 0.5)
    middle_xyzw = np.array([*middle_wxyz[1:], middle_wxyz[0]])
    middle_yaw = Rotation.from_quat(middle_xyzw).as_euler("xyz", degrees=True)[2]
    assert abs(middle_yaw) == pytest.approx(180.0)


def test_pose_stream_rejects_non_monotonic_capture_time_and_resets() -> None:
    stream = WebXRSonicPoseStream()
    stream.push(_snapshot(capture_time_s=10.0), t_now=1.0)

    with pytest.raises(PoseStreamError, match="did not increase"):
        stream.push(_snapshot(capture_time_s=10.0), t_now=1.01)

    assert stream.ready is False
    assert stream.buffered_frames == 0


def test_retarget_rejects_incomplete_body_frame() -> None:
    snapshot = _snapshot(omitted=frozenset({"head"}))

    with pytest.raises(IncompleteBodyPoseError, match="head"):
        WebXRSonicRetargeter().retarget(snapshot, frame_index=0, t_now=1.0)


def test_retarget_rejects_invalid_orientation() -> None:
    snapshot = _snapshot()
    assert snapshot.joints is not None
    joints = dict(snapshot.joints)
    joints["head"] = BodyJointPose(
        position=joints["head"].position,
        orientation=(0.0, 0.0, 0.0, 0.0),
    )
    invalid = snapshot.model_copy(update={"joints": joints})

    with pytest.raises(IncompleteBodyPoseError, match="head"):
        WebXRSonicRetargeter().retarget(invalid, frame_index=0, t_now=1.0)
