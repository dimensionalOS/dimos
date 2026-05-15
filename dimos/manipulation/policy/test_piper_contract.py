# Copyright 2026 Dimensional Inc.
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

"""Unit tests for `PiperRobotContract`."""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from dimos.manipulation.policy.contract import GripperBinarization
from dimos.manipulation.policy.contracts.piper import PiperRobotContract
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState

# Don't import from `dimos.manipulation.data_collection.piper_blueprint_config` here — that
# transitively pulls in the Piper teleop blueprint and mujoco, deps the
# contract package (and therefore its tests) deliberately avoid. Instead use
# the same source the contract itself uses.
JOINTS = list(PiperRobotContract().state_joint_names)
GRIPPER_OPEN = 0.08
GRIPPER_CLOSED = 0.0


def _make_image(h: int = 480, w: int = 640) -> Image:
    return Image.from_numpy(np.zeros((h, w, 3), dtype=np.uint8), format=ImageFormat.RGB)


def _make_jointstate(positions: dict[str, float]) -> JointState:
    names = [f"arm/{n}" for n in positions]
    return JointState(name=names, position=list(positions.values()))


def _make_row(
    *,
    state_positions: dict[str, float],
    action_positions: dict[str, float],
    image: np.ndarray | None = None,
) -> dict[str, Any]:
    if image is None:
        image = np.zeros((480, 640, 3), dtype=np.uint8)
    row: dict[str, Any] = {"/observation/camera/usb": image}
    for j, v in state_positions.items():
        row[f"/observation/state/{j}"] = float(v)
    for j, v in action_positions.items():
        row[f"/action/{j}"] = float(v)
    return row


# ── Schema ───────────────────────────────────────────────────────────────


def test_joint_ordering_is_arm_joints_then_gripper():
    c = PiperRobotContract()
    assert list(c.state_joint_names) == [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "gripper",
    ]
    assert list(c.action_joint_names) == list(c.state_joint_names)
    assert c.gripper_joint == "gripper"
    assert c.cameras == {"usb": (480, 640, 3)}


def test_joint_ordering_matches_data_collection_helper_when_available():
    """Cross-check the contract joint ordering against the recorder's helper.

    The contract derives names from the robot catalog; the recorder uses
    `data_collection_vis.piper_data_collection_joint_short_names()`. They MUST
    agree, but the recorder helper transitively imports mujoco — skip the
    cross-check when mujoco isn't installed (the catalog-derived list still
    governs in that env).
    """
    pytest.importorskip("mujoco")
    from dimos.manipulation.data_collection.piper_blueprint_config import (
        piper_data_collection_joint_short_names,
    )

    assert list(PiperRobotContract().state_joint_names) == list(
        piper_data_collection_joint_short_names()
    )


def test_features_shape_and_keys():
    c = PiperRobotContract()
    feats = c.features()
    assert set(feats) == {"observation.images.usb", "observation.state", "action"}
    assert feats["observation.images.usb"]["shape"] == (480, 640, 3)
    assert feats["observation.state"]["shape"] == (7,)
    assert feats["action"]["shape"] == (7,)
    assert feats["observation.state"]["names"] == JOINTS
    assert feats["action"]["names"] == JOINTS


def test_rerun_entities_lists_camera_state_action():
    c = PiperRobotContract()
    entities = list(c.rerun_entities())
    assert "/observation/camera/usb" in entities
    for j in JOINTS:
        assert f"/observation/state/{j}" in entities
        assert f"/action/{j}" in entities


# ── from_rerun_row: ordering and shapes ──────────────────────────────────


def test_from_rerun_row_packs_state_action_in_joint_order():
    c = PiperRobotContract()
    state = {j: 0.1 * (i + 1) for i, j in enumerate(JOINTS)}
    action = {j: 0.2 * (i + 1) for i, j in enumerate(JOINTS)}
    # Set gripper raw values that will normalize cleanly.
    state["gripper"] = 0.04  # → 0.04 / 0.08 = 0.5 fraction-open
    action["gripper"] = 0.07  # → 0.07 / 0.08 = 0.875 fraction-open

    frame = c.from_rerun_row(_make_row(state_positions=state, action_positions=action))

    assert frame["observation.state"].shape == (7,)
    assert frame["observation.state"].dtype == np.float32
    assert frame["action"].shape == (7,)
    assert frame["action"].dtype == np.float32
    assert frame["observation.images.usb"].shape == (480, 640, 3)
    assert frame["observation.images.usb"].dtype == np.uint8

    # Non-gripper slots in observation.state are passed through raw.
    for i, j in enumerate(JOINTS[:-1]):
        np.testing.assert_allclose(frame["observation.state"][i], state[j], rtol=1e-6)
    # Non-gripper slots in action are passed through raw.
    for i, j in enumerate(JOINTS[:-1]):
        np.testing.assert_allclose(frame["action"][i], action[j], rtol=1e-6)


# ── Gripper binarization ─────────────────────────────────────────────────


def test_gripper_action_above_threshold_is_one():
    c = PiperRobotContract()  # threshold default 0.7
    state = {j: 0.0 for j in JOINTS}
    action = {j: 0.0 for j in JOINTS}
    # 0.06 / 0.08 = 0.75 > 0.7 → 1.0
    action["gripper"] = 0.06
    frame = c.from_rerun_row(_make_row(state_positions=state, action_positions=action))
    assert frame["action"][-1] == 1.0


def test_gripper_action_below_threshold_is_zero():
    c = PiperRobotContract()
    state = {j: 0.0 for j in JOINTS}
    action = {j: 0.0 for j in JOINTS}
    # 0.05 / 0.08 = 0.625 < 0.7 → 0.0
    action["gripper"] = 0.05
    frame = c.from_rerun_row(_make_row(state_positions=state, action_positions=action))
    assert frame["action"][-1] == 0.0


def test_gripper_threshold_is_exclusive_on_open_side():
    # A normalized fraction equal to the threshold maps to 0.0 (closed).
    c = PiperRobotContract(
        gripper_binarization=GripperBinarization(open_pos=1.0, closed_pos=0.0, threshold=0.5),
    )
    state = {j: 0.0 for j in JOINTS}
    action = {j: 0.0 for j in JOINTS}
    action["gripper"] = 0.5  # exactly threshold → closed
    frame = c.from_rerun_row(_make_row(state_positions=state, action_positions=action))
    assert frame["action"][-1] == 0.0


def test_state_gripper_is_rescaled_not_thresholded():
    c = PiperRobotContract()  # binarize default ON
    state = {j: 0.0 for j in JOINTS}
    state["gripper"] = 0.04  # → 0.04 / 0.08 = 0.5 (not thresholded)
    action = {j: 0.0 for j in JOINTS}
    frame = c.from_rerun_row(_make_row(state_positions=state, action_positions=action))
    np.testing.assert_allclose(frame["observation.state"][-1], 0.04 / 0.08, rtol=1e-5)


def test_no_binarize_preserves_raw_in_both_state_and_action():
    c = PiperRobotContract(
        gripper_binarization=GripperBinarization(
            open_pos=0.08, closed_pos=0.0, enabled=False, threshold=0.7
        ),
    )
    state = {j: 0.0 for j in JOINTS}
    action = {j: 0.0 for j in JOINTS}
    state["gripper"] = 0.42
    action["gripper"] = 0.42
    frame = c.from_rerun_row(_make_row(state_positions=state, action_positions=action))
    np.testing.assert_allclose(frame["observation.state"][-1], 0.42, rtol=1e-6)
    np.testing.assert_allclose(frame["action"][-1], 0.42, rtol=1e-6)


# ── from_messages mirrors from_rerun_row ─────────────────────────────────


def test_from_messages_keys_shapes_dtypes_match_from_rerun_row():
    c = PiperRobotContract()
    state_pos = {j: 0.1 * i for i, j in enumerate(JOINTS)}
    action_pos = {j: 0.2 * i for i, j in enumerate(JOINTS)}
    state_pos["gripper"] = 0.6
    action_pos["gripper"] = 0.6

    rerun_frame = c.from_rerun_row(
        _make_row(state_positions=state_pos, action_positions=action_pos)
    )
    msg_frame = c.from_messages(
        images={"usb": _make_image()},
        state=_make_jointstate(state_pos),
        action=_make_jointstate(action_pos),
    )

    # Shared keys: same dtype + shape.
    for key in ("observation.images.usb", "observation.state", "action"):
        assert rerun_frame[key].shape == msg_frame[key].shape, key
        assert rerun_frame[key].dtype == msg_frame[key].dtype, key
    # Same numerical values for state + action.
    np.testing.assert_allclose(rerun_frame["observation.state"], msg_frame["observation.state"])
    np.testing.assert_allclose(rerun_frame["action"], msg_frame["action"])


def test_from_messages_action_none_omits_action_key():
    c = PiperRobotContract()
    state_pos = {j: 0.0 for j in JOINTS}
    frame = c.from_messages(
        images={"usb": _make_image()},
        state=_make_jointstate(state_pos),
        action=None,
        task="pick the block",
    )
    assert "action" not in frame
    assert frame["task"] == "pick the block"
    assert "observation.images.usb" in frame
    assert "observation.state" in frame


def test_from_messages_missing_camera_raises():
    c = PiperRobotContract()
    state_pos = {j: 0.0 for j in JOINTS}
    with pytest.raises(KeyError, match="usb"):
        c.from_messages(images={}, state=_make_jointstate(state_pos))


def test_from_messages_missing_joint_raises():
    c = PiperRobotContract()
    state_pos = {j: 0.0 for j in JOINTS if j != "joint3"}
    with pytest.raises(KeyError, match="joint3"):
        c.from_messages(images={"usb": _make_image()}, state=_make_jointstate(state_pos))


# ── to_command ───────────────────────────────────────────────────────────


def test_to_command_binarized_gripper_one_maps_to_open_pos():
    c = PiperRobotContract()
    vec = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 1.0], dtype=np.float32)
    cmd = c.to_command(vec)
    assert cmd.position[-1] == GRIPPER_OPEN
    assert cmd.name[-1] == "arm/gripper"
    assert cmd.name[:-1] == [f"arm/{j}" for j in JOINTS[:-1]]


def test_to_command_binarized_gripper_zero_maps_to_closed_pos():
    c = PiperRobotContract()
    vec = np.array([0.0] * 6 + [0.0], dtype=np.float32)
    cmd = c.to_command(vec)
    assert cmd.position[-1] == GRIPPER_CLOSED


def test_to_command_no_binarize_passes_gripper_through():
    c = PiperRobotContract(
        gripper_binarization=GripperBinarization(open_pos=0.08, closed_pos=0.0, enabled=False),
    )
    vec = np.array([0.0] * 6 + [0.42], dtype=np.float32)
    cmd = c.to_command(vec)
    assert cmd.position[-1] == pytest.approx(0.42)


def test_to_command_wrong_shape_raises_value_error():
    c = PiperRobotContract()
    with pytest.raises(ValueError, match="7"):
        c.to_command(np.zeros(6, dtype=np.float32))
    with pytest.raises(ValueError, match="7"):
        c.to_command(np.zeros((7, 1), dtype=np.float32))
