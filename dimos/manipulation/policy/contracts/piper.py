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

"""Piper concrete `RobotContract`.

Maps the schema written by `RerunDataRecorder` for the Piper teleop blueprint
(`/observation/camera/usb`, `/observation/state/<joint>`, `/action/<joint>`)
to the LeRobot frame dict shape (`observation.images.usb`, `observation.state`,
`action`).

The joint name list is derived from
`dimos.manipulation.data_collection.piper_blueprint_config.piper_data_collection_joint_short_names()`
so renaming a joint in the robot model propagates here automatically.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.policy.contract import GripperBinarization
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.Image import Image


_CAMERA_KEY = "usb"
_CAMERA_ENTITY_PATH = "/observation/camera/usb"
_CAMERA_SHAPE = (480, 640, 3)
_OBSERVATION_STATE_PREFIX = "/observation/state"
_ACTION_PREFIX = "/action"
_JOINT_PREFIX = "arm"


def _piper_joint_names() -> tuple[str, ...]:
    # Source: `dimos.robot.catalog.piper.piper(...).joint_names` plus the
    # gripper joint. We import the catalog factory rather than the teleop
    # `data_collection_vis` helper because the latter transitively pulls in
    # the teleop blueprint and mujoco — heavy deps the converter / policy
    # package shouldn't require.
    from dimos.robot.catalog.piper import piper

    cfg = piper()
    arm_short = [name.split("/")[-1] for name in (cfg.joint_names or [])]
    gripper_short = "gripper"
    if cfg.gripper and cfg.gripper.joints:
        gripper_short = cfg.gripper.joints[0].split("/")[-1]
    return (*arm_short, gripper_short)


@dataclass
class PiperRobotContract:
    """Piper teleop schema → LeRobot frame translator."""

    cameras: Mapping[str, tuple[int, int, int]] = field(
        default_factory=lambda: {_CAMERA_KEY: _CAMERA_SHAPE},
    )
    state_joint_names: Sequence[str] = field(default_factory=_piper_joint_names)
    action_joint_names: Sequence[str] = field(default_factory=_piper_joint_names)
    gripper_joint: str | None = "gripper"
    gripper_binarization: GripperBinarization = field(
        # `open_pos` matches `dimos.robot.catalog.piper.piper().gripper.open_position`
        # — the value the recorder writes to `/action/gripper`. Note this is NOT
        # the same as the `0.85` `gripper_open_pos` in the teleop blueprint,
        # which is a trigger-to-position scale applied before the value reaches
        # this layer.
        default_factory=lambda: GripperBinarization(open_pos=0.08, closed_pos=0.0),
    )

    # ── LeRobot dataset feature schema ────────────────────────────────────

    def features(self) -> dict[str, dict[str, Any]]:
        feats: dict[str, dict[str, Any]] = {}
        for cam_key, (h, w, c) in self.cameras.items():
            feats[f"observation.images.{cam_key}"] = {
                "dtype": "video",
                "shape": (h, w, c),
                "names": ["height", "width", "channels"],
            }
        feats["observation.state"] = {
            "dtype": "float32",
            "shape": (len(self.state_joint_names),),
            "names": list(self.state_joint_names),
        }
        feats["action"] = {
            "dtype": "float32",
            "shape": (len(self.action_joint_names),),
            "names": list(self.action_joint_names),
        }
        return feats

    # ── rrd entity-path selection ─────────────────────────────────────────

    def rerun_entities(self) -> Sequence[str]:
        paths = [_CAMERA_ENTITY_PATH]
        paths.extend(f"{_OBSERVATION_STATE_PREFIX}/{j}" for j in self.state_joint_names)
        paths.extend(f"{_ACTION_PREFIX}/{j}" for j in self.action_joint_names)
        return paths

    # ── rrd row → frame dict ──────────────────────────────────────────────

    def from_rerun_row(self, row: Mapping[str, Any]) -> dict[str, Any]:
        """Build a LeRobot frame from a row dict whose keys are entity paths.

        The row dict is the per-tick view a CLI converter has after extracting
        latest-non-null values from a `rerun.dataframe.view().select()` Arrow
        batch. Values:

        - `row["/observation/camera/usb"]` → a uint8 ndarray of the
          contract-declared shape (or convertible to one).
        - `row["/observation/state/<joint>"]` → a float scalar.
        - `row["/action/<joint>"]` → a float scalar.
        """
        frame: dict[str, Any] = {}
        for cam_key in self.cameras:
            entity = (
                _CAMERA_ENTITY_PATH if cam_key == _CAMERA_KEY else f"/observation/camera/{cam_key}"
            )
            frame[f"observation.images.{cam_key}"] = self._extract_image(row, entity, cam_key)

        state_raw = self._pack_scalars(row, _OBSERVATION_STATE_PREFIX, self.state_joint_names)
        action_raw = self._pack_scalars(row, _ACTION_PREFIX, self.action_joint_names)

        frame["observation.state"] = self._apply_state_gripper_rescale(state_raw)
        frame["action"] = self._apply_action_gripper_binarize(action_raw)
        return frame

    # ── live messages → frame dict ────────────────────────────────────────

    def from_messages(
        self,
        images: Mapping[str, Image],
        state: JointState,
        *,
        action: JointState | None = None,
        task: str = "",
    ) -> dict[str, Any]:
        frame: dict[str, Any] = {}

        missing_cams = [k for k in self.cameras if k not in images]
        if missing_cams:
            raise KeyError(f"PiperRobotContract.from_messages: missing camera(s) {missing_cams}")
        for cam_key in self.cameras:
            arr = self._image_msg_to_array(images[cam_key], cam_key)
            frame[f"observation.images.{cam_key}"] = arr

        state_raw = self._jointstate_to_vector(state, self.state_joint_names)
        frame["observation.state"] = self._apply_state_gripper_rescale(state_raw)

        if action is not None:
            action_raw = self._jointstate_to_vector(action, self.action_joint_names)
            frame["action"] = self._apply_action_gripper_binarize(action_raw)

        if task:
            frame["task"] = task
        return frame

    # ── action vector → coordinator JointState ────────────────────────────

    def to_command(self, action_vec: np.ndarray) -> JointState:
        expected = len(self.action_joint_names)
        if action_vec.shape != (expected,):
            raise ValueError(
                f"PiperRobotContract.to_command: expected action vector of shape "
                f"({expected},), got {tuple(action_vec.shape)}"
            )
        positions = action_vec.astype(float).tolist()
        if self.gripper_binarization.enabled and self.gripper_joint is not None:
            gi = self._gripper_index(self.action_joint_names)
            positions[gi] = self._binary_to_position(positions[gi])
        names = [f"{_JOINT_PREFIX}/{n}" for n in self.action_joint_names]
        return JointState(name=names, position=positions)

    # ── helpers ───────────────────────────────────────────────────────────

    def _gripper_index(self, joints: Sequence[str]) -> int:
        if self.gripper_joint is None:
            raise ValueError("PiperRobotContract has no gripper_joint configured")
        return joints.index(self.gripper_joint)

    def _pack_scalars(
        self, row: Mapping[str, Any], prefix: str, joints: Sequence[str]
    ) -> np.ndarray:
        vals: list[float] = []
        missing: list[str] = []
        for j in joints:
            key = f"{prefix}/{j}"
            v = row.get(key)
            if v is None:
                missing.append(key)
                vals.append(0.0)
            else:
                vals.append(float(self._scalar_value(v)))
        if missing:
            raise KeyError(
                f"PiperRobotContract.from_rerun_row: missing scalar(s) in row: {missing}"
            )
        return np.asarray(vals, dtype=np.float32)

    @staticmethod
    def _scalar_value(v: Any) -> float:
        # rerun.dataframe.view().select() yields per-cell values that may be
        # numpy scalars, 1-element arrays, Python floats, or pyarrow scalars
        # depending on the column's archetype. Reduce to a Python float.
        if hasattr(v, "as_py"):
            v = v.as_py()
        if isinstance(v, (list, tuple)):
            v = v[0]
        arr = np.asarray(v).reshape(-1)
        if arr.size != 1:
            raise ValueError(f"expected scalar, got array of size {arr.size}")
        return float(arr[0])

    def _extract_image(self, row: Mapping[str, Any], entity: str, cam_key: str) -> np.ndarray:
        v = row.get(entity)
        if v is None:
            raise KeyError(f"PiperRobotContract.from_rerun_row: missing image at {entity}")
        h, w, c = self.cameras[cam_key]
        if hasattr(v, "as_py"):
            v = v.as_py()
        if isinstance(v, (bytes, bytearray, memoryview)):
            arr = np.frombuffer(bytes(v), dtype=np.uint8).reshape(h, w, c)
        else:
            arr = np.asarray(v, dtype=np.uint8)
            if arr.shape != (h, w, c):
                arr = arr.reshape(h, w, c)
        return arr

    @staticmethod
    def _image_msg_to_array(image: Image, cam_key: str) -> np.ndarray:
        # `Image.from_numpy(...)` and friends store the array on `.data`.
        data = getattr(image, "data", None)
        if data is None:
            raise ValueError(f"PiperRobotContract.from_messages: image '{cam_key}' has no .data")
        return np.ascontiguousarray(data, dtype=np.uint8)

    def _jointstate_to_vector(self, joint_state: JointState, joints: Sequence[str]) -> np.ndarray:
        # Tolerate full-namespace names in the message (e.g. "arm/joint1") by
        # matching on the short suffix.
        index_by_short: dict[str, int] = {}
        for i, full in enumerate(joint_state.name):
            short = full.split("/")[-1]
            index_by_short[short] = i

        out: list[float] = []
        missing: list[str] = []
        for j in joints:
            idx = index_by_short.get(j)
            if idx is None:
                missing.append(j)
                out.append(0.0)
            else:
                out.append(float(joint_state.position[idx]))
        if missing:
            raise KeyError(f"PiperRobotContract.from_messages: JointState missing joints {missing}")
        return np.asarray(out, dtype=np.float32)

    def _apply_state_gripper_rescale(self, state_raw: np.ndarray) -> np.ndarray:
        out = state_raw.astype(np.float32, copy=True)
        if self.gripper_binarization.enabled and self.gripper_joint is not None:
            gi = self._gripper_index(self.state_joint_names)
            out[gi] = self._normalize_gripper(float(out[gi]))
        return out

    def _apply_action_gripper_binarize(self, action_raw: np.ndarray) -> np.ndarray:
        out = action_raw.astype(np.float32, copy=True)
        if self.gripper_binarization.enabled and self.gripper_joint is not None:
            gi = self._gripper_index(self.action_joint_names)
            out[gi] = self._binarize_gripper(float(out[gi]))
        return out

    def _normalize_gripper(self, raw: float) -> float:
        cfg = self.gripper_binarization
        span = cfg.open_pos - cfg.closed_pos
        if span == 0.0:
            return 0.0
        return float((raw - cfg.closed_pos) / span)

    def _binarize_gripper(self, raw: float) -> float:
        return 1.0 if self._normalize_gripper(raw) > self.gripper_binarization.threshold else 0.0

    def _binary_to_position(self, binary: float) -> float:
        cfg = self.gripper_binarization
        return cfg.open_pos if binary >= 0.5 else cfg.closed_pos


__all__ = ["PiperRobotContract"]
