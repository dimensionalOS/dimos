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
    WRIST_ONNX_INDICES,
)
from dimos.control.tasks.g1_sonic_wbc_task.streamed_motion import StreamedMotionMerger
from dimos.control.tasks.g1_sonic_wbc_task.webxr_retargeting import (
    SMPL_WEBXR_JOINTS,
    IncompleteBodyPoseError,
    PoseStreamError,
    PoseStreamGapError,
    WebXRSonicPoseStream,
    WebXRSonicRetargeter,
    _interpolate_quaternion_wxyz,
)
from dimos.msgs.visualization_msgs.SonicPoseReference import SMPL_PARENTS
from dimos.teleop.webxr.body_tracking import BodyJointPose, BodyTrackingSnapshot

_NVIDIA_PICO_ROTATION_OFFSET = Rotation.from_euler("y", 180.0, degrees=True)


def _webxr_global_rotations(
    local_rotations: dict[str, Rotation],
) -> dict[str, Rotation]:
    nvidia_global: list[Rotation] = []
    result: dict[str, Rotation] = {}
    for index, name in enumerate(SMPL_WEBXR_JOINTS[:22]):
        local = local_rotations.get(name, Rotation.identity())
        parent = SMPL_PARENTS[index]
        global_rotation = local if parent == -1 else nvidia_global[parent] * local
        nvidia_global.append(global_rotation)
        result[name] = global_rotation * _NVIDIA_PICO_ROTATION_OFFSET.inv()

    result[SMPL_WEBXR_JOINTS[22]] = nvidia_global[20] * _NVIDIA_PICO_ROTATION_OFFSET.inv()
    result[SMPL_WEBXR_JOINTS[23]] = nvidia_global[21] * _NVIDIA_PICO_ROTATION_OFFSET.inv()
    return result


def _snapshot(
    *,
    capture_time_s: float = 10.0,
    rotations: dict[str, Rotation] | None = None,
    position_scale: float = 1.0,
    omitted: frozenset[str] = frozenset(),
) -> BodyTrackingSnapshot:
    global_rotations = _webxr_global_rotations(rotations or {})
    joints = {
        name: BodyJointPose(
            position=(
                position_scale * float(index),
                position_scale * float(2 * index),
                position_scale * float(-3 * index),
            ),
            orientation=tuple(float(value) for value in global_rotations[name].as_quat()),
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


def test_retarget_matches_nvidia_neutral_reference() -> None:
    frame = WebXRSonicRetargeter().retarget(_snapshot(), frame_index=7).fields

    assert frame["frame_index"].tolist() == [7]
    assert frame["joint_pos"].shape == (1, 29)
    assert frame["joint_vel"].shape == (1, 29)
    np.testing.assert_allclose(
        frame["body_quat_w"],
        [[0.70710677, 0.0, 0.0, -0.70710677]],
        atol=1e-7,
    )
    assert frame["smpl_joints"].shape == (1, 24, 3)
    assert frame["smpl_pose"].shape == (1, 21, 3)
    np.testing.assert_allclose(
        frame["smpl_joints"][0, [0, 1, 10, 15, 20, 21, 22, 23]],
        [
            [0.35140750, 0.00312328, 0.01203655],
            [0.32540634, 0.06131265, -0.08072688],
            [0.40235060, 0.11981189, -0.92053664],
            [0.33541873, 0.01109689, 0.63163406],
            [0.27868444, 0.67018986, 0.39975783],
            [0.27843621, -0.67221105, 0.40285343],
            [0.34479648, 0.74836445, 0.37759736],
            [0.34479663, -0.74836504, 0.37759790],
        ],
        atol=1e-6,
    )
    np.testing.assert_allclose(frame["smpl_pose"], 0.0, atol=1e-7)
    np.testing.assert_allclose(frame["joint_pos"], 0.0, atol=1e-7)
    np.testing.assert_allclose(frame["joint_vel"], 0.0, atol=1e-7)

    merged = StreamedMotionMerger().merge(frame, current_playback_frame=0)
    assert merged.error is None
    assert merged.motion is not None
    assert merged.motion.encode_mode == 2


def test_retarget_uses_canonical_bone_lengths() -> None:
    retargeter = WebXRSonicRetargeter()

    normal = retargeter.retarget(_snapshot(position_scale=1.0), frame_index=0)
    tall = retargeter.retarget(_snapshot(position_scale=2.0), frame_index=1)

    np.testing.assert_allclose(tall.fields["smpl_joints"], normal.fields["smpl_joints"])


def test_retarget_matches_nvidia_asymmetric_pose_reference() -> None:
    rotations = {
        "hips": Rotation.from_rotvec([0.1, -0.2, 0.3]),
        "left-upper-leg": Rotation.from_rotvec([0.12, -0.08, 0.03]),
        "left-lower-leg": Rotation.from_rotvec([-0.3, 0.05, 0.1]),
        "right-arm-upper": Rotation.from_rotvec([0.2, -0.15, 0.35]),
        "left-arm-lower": Rotation.from_rotvec([0.25, 0.4, -0.1]),
        "right-arm-lower": Rotation.from_rotvec([-0.3, 0.2, 0.45]),
        "left-hand-wrist": Rotation.from_rotvec([0.15, -0.25, 0.3]),
        "right-hand-wrist": Rotation.from_rotvec([-0.2, 0.1, -0.35]),
    }

    frame = WebXRSonicRetargeter().retarget(_snapshot(rotations=rotations), frame_index=4).fields

    np.testing.assert_allclose(
        frame["body_quat_w"],
        [[0.6244695, 0.14059779, -0.0702990, -0.7650673]],
        atol=1e-6,
    )
    np.testing.assert_allclose(
        frame["smpl_joints"][0, [0, 4, 10, 18, 20, 21, 22, 23]],
        [
            [0.34062776, 0.08019388, 0.03440223],
            [0.26495391, 0.20674747, -0.43171456],
            [0.43539619, 0.25942671, -0.86555368],
            [0.27037677, 0.49527410, 0.39890230],
            [0.17327875, 0.72870469, 0.41006434],
            [0.21995851, -0.48212010, 0.16907625],
            [0.21780342, 0.81661332, 0.37452894],
            [0.29264113, -0.55064464, 0.13970664],
        ],
        atol=1e-6,
    )
    np.testing.assert_allclose(
        frame["smpl_pose"][0, [0, 3, 16, 17, 18, 19, 20]],
        [
            [0.12, -0.08, 0.03],
            [-0.3, 0.05, 0.1],
            [0.2, -0.15, 0.35],
            [0.25, 0.4, -0.1],
            [-0.3, 0.2, 0.45],
            [0.15, -0.25, 0.3],
            [-0.2, 0.1, -0.35],
        ],
        atol=1e-6,
    )


def test_retarget_matches_nvidia_elbow_swing_and_wrist_mapping() -> None:
    rotations = {
        "left-arm-lower": Rotation.from_rotvec([0.25, 0.4, -0.1]),
        "right-arm-lower": Rotation.from_rotvec([-0.3, 0.2, 0.45]),
        "left-hand-wrist": Rotation.from_rotvec([0.15, -0.25, 0.3]),
        "right-hand-wrist": Rotation.from_rotvec([-0.2, 0.1, -0.35]),
    }

    frame = WebXRSonicRetargeter().retarget(_snapshot(rotations=rotations), frame_index=1).fields

    np.testing.assert_allclose(
        frame["joint_pos"][0, WRIST_ONNX_INDICES],
        [
            0.45141233,
            0.51398351,
            -0.22242762,
            -0.13203273,
            0.27607877,
            0.06898271,
        ],
        atol=1e-6,
    )
    np.testing.assert_allclose(frame["joint_vel"], 0.0, atol=1e-7)


def test_pose_stream_waits_for_ten_chronological_resampled_frames() -> None:
    stream = WebXRSonicPoseStream()

    for index in range(9):
        stream.push(_snapshot(capture_time_s=10.0 + 0.02 * index, position_scale=1.0 + 0.1 * index))

    assert stream.ready is False
    assert stream.buffered_frames == 9

    stream.push(_snapshot(capture_time_s=10.18, position_scale=1.9))

    assert stream.ready is True
    assert stream.buffered_frames == 10
    assert stream.generation == 10
    fields = stream.fields()
    assert fields["frame_index"].tolist() == list(range(10))
    assert fields["smpl_joints"].shape == (10, 24, 3)
    assert fields["smpl_pose"].shape == (10, 21, 3)
    expected_joints = np.repeat(fields["smpl_joints"][0:1], 10, axis=0)
    np.testing.assert_allclose(fields["smpl_joints"], expected_joints, atol=1e-7)

    stream.push(_snapshot(capture_time_s=10.20, position_scale=2.0))

    assert stream.buffered_frames == 10
    assert stream.generation == 11
    assert stream.fields()["frame_index"].tolist() == list(range(1, 11))


def test_low_latency_pose_stream_uses_four_frame_rolling_window() -> None:
    stream = WebXRSonicPoseStream(sonic_pipeline="sonic-low-latency")

    stream.push(_snapshot(capture_time_s=10.0))

    assert stream.ready is False
    assert stream.buffered_frames == 0

    stream.push(_snapshot(capture_time_s=10.02))

    assert stream.ready is False
    assert stream.sonic_pipeline == "sonic-low-latency"
    assert stream.window_frames == 4

    stream.push(_snapshot(capture_time_s=10.04))
    stream.push(_snapshot(capture_time_s=10.06))

    assert stream.ready is True
    assert stream.buffered_frames == 4
    assert stream.fields()["frame_index"].tolist() == [0, 1, 2, 3]

    stream.push(_snapshot(capture_time_s=10.08))

    assert stream.buffered_frames == 4
    assert stream.fields()["frame_index"].tolist() == [1, 2, 3, 4]


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
    stream.push(_snapshot(capture_time_s=10.0))

    with pytest.raises(PoseStreamError, match="did not increase"):
        stream.push(_snapshot(capture_time_s=10.0))

    assert stream.ready is False
    assert stream.buffered_frames == 0


def test_pose_stream_capture_gap_reprimes_a_fresh_window() -> None:
    stream = WebXRSonicPoseStream()
    stream.push(_snapshot(capture_time_s=10.0))

    with pytest.raises(PoseStreamGapError, match="gap exceeded 150 ms"):
        stream.push(_snapshot(capture_time_s=10.2))

    assert stream.ready is False
    assert stream.buffered_frames == 0

    for index in range(1, 10):
        stream.push(_snapshot(capture_time_s=10.2 + 0.02 * index))

    assert stream.ready is True
    assert stream.fields()["frame_index"].tolist() == list(range(10))


def test_retarget_rejects_incomplete_body_frame() -> None:
    snapshot = _snapshot(omitted=frozenset({"head"}))

    with pytest.raises(IncompleteBodyPoseError, match="head"):
        WebXRSonicRetargeter().retarget(snapshot, frame_index=0)


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
        WebXRSonicRetargeter().retarget(invalid, frame_index=0)
