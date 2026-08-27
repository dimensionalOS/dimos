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
    WebXRSonicRetargeter,
)
from dimos.teleop.webxr.body_tracking import BodyJointPose, BodyTrackingSnapshot


def _webxr_quaternion_for_sonic_rotation(rotation: Rotation) -> tuple[float, ...]:
    matrix = WEBXR_TO_SONIC.T @ rotation.as_matrix() @ WEBXR_TO_SONIC
    return tuple(float(value) for value in Rotation.from_matrix(matrix).as_quat())


def _snapshot(
    *,
    overrides: dict[str, Rotation] | None = None,
    omitted: frozenset[str] = frozenset(),
) -> BodyTrackingSnapshot:
    rotations = overrides or {}
    joints = {
        name: BodyJointPose(
            position=(float(index), float(2 * index), float(-3 * index)),
            orientation=_webxr_quaternion_for_sonic_rotation(
                rotations.get(name, Rotation.identity())
            ),
        )
        for index, name in enumerate(SMPL_WEBXR_JOINTS)
        if name not in omitted
    }
    return BodyTrackingSnapshot(
        type="body_tracking_snapshot",
        capture_time_s=10.0,
        frame_id="local-floor",
        joints=joints,
    )


def test_retarget_produces_protocol_v3_full_body_frame() -> None:
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


def test_retarget_derives_wrist_targets_and_velocity() -> None:
    retargeter = WebXRSonicRetargeter()
    retargeter.retarget(_snapshot(), frame_index=0, t_now=1.0)
    moved = _snapshot(overrides={"left-hand-wrist": Rotation.from_euler("x", 0.3)})

    frame = retargeter.retarget(moved, frame_index=1, t_now=1.1).fields

    assert frame["joint_pos"][0, WRIST_ONNX_INDICES[0]] == pytest.approx(0.3)
    assert frame["joint_vel"][0, WRIST_ONNX_INDICES[0]] == pytest.approx(3.0)
    np.testing.assert_allclose(frame["joint_pos"][0, WRIST_ONNX_INDICES[1:]], 0.0)


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
