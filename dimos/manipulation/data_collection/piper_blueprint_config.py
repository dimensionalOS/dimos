#!/usr/bin/env python3
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

"""Rerun visualization wiring for the Piper data collection blueprint.

Provides:

- ``joint_state_to_rerun_scalars`` — the JointState → per-joint Rerun scalar
  override consumed by both the live viewer and the on-disk recorder.
- ``piper_data_collection_rerun_blueprint`` — the preset Rerun layout (camera
  on the left, per-joint plots on the right).
- ``piper_data_collection_rerun_config`` — the single source-of-truth config
  dict consumed by both ``vis_module("rerun", rerun_config=...)`` and
  ``RerunDataRecorder.blueprint(...)``. Sharing this dict is the structural
  guarantee that the viewer and the recorder cannot drift apart.
- Session / episode path / id / metadata factories used by the blueprint to
  drive the standalone ``RerunDataRecorder``.

Joint name source is shared with the override and the preset so they cannot
drift. Recorded entity paths follow a LeRobot-aligned schema —
``/observation/camera/usb``, ``/observation/state/<joint>``, ``/action/<joint>``
— so ``rerun.dataframe`` / ``rerun.experimental.dataloader`` can select
observations and actions by entity-path prefix.

The recorder consumes camera, joint state, and desired action through
typed ``In[T]`` stream slots (see ``recorder.py``). The
``visual_override`` / ``topic_to_entity`` keys produced by
``piper_data_collection_rerun_config()`` are consumed only by the bridge
(live viewer); the recorder ignores them.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timezone
from functools import partial
import os
from pathlib import Path
from typing import Any, Literal

import rerun as rr
import rerun.blueprint as rrb

from dimos.control.blueprints.teleop import piper_teleop_robot_model_config
from dimos.control.components import make_gripper_joints
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.data import get_data_dir
from dimos.visualization.rerun.bridge import RerunMulti

JointRole = Literal["measured", "commanded"]

# LeRobot-aligned entity-path layout. The recorder and the viewer both log
# under these prefixes; downstream readers can select observation vs. action by
# entity-path prefix without consulting per-session metadata.
_OBSERVATION_STATE_PREFIX = "/observation/state"
_ACTION_PREFIX = "/action"

# Camera image is logged at this path by the recorder's typed `image`
# stream slot. The leaf segment (`_CAMERA_KEY`) is passed to the recorder
# via `RerunDataRecorderConfig.camera_key` and to the policy module via
# `PolicyModuleConfig.camera_key`; the bridge layout preset below targets
# the full path.
_CAMERA_KEY = "usb"
_CAMERA_ENTITY_PATH = f"/observation/camera/{_CAMERA_KEY}"

# Visual-override pattern keys. With the LeRobot retarget the recorder /
# bridge use entity_prefix="" so these match the bare LCM topic names.
_JOINT_STATE_ENTITY_PATH = "/coordinator/joint_state"
_DESIRED_JOINT_ACTION_ENTITY_PATH = "/coordinator/desired_joint_action"


def piper_data_collection_joint_short_names() -> list[str]:
    """Ordered short joint names plotted by the data collection visualization.

    Arm joints come first (from the Piper robot model), gripper last. The
    "short" name is what the JointState-to-scalar override emits as the leaf
    entity-path segment, so this list must match what the override produces.
    """
    arm_short = [name.split("/")[-1] for name in piper_teleop_robot_model_config().joint_names]
    gripper_short = make_gripper_joints("arm")[0].split("/")[-1]
    return [*arm_short, gripper_short]


def _entity_path(role: JointRole, short_name: str) -> str:
    if role == "measured":
        return f"{_OBSERVATION_STATE_PREFIX}/{short_name}"
    return f"{_ACTION_PREFIX}/{short_name}"


def _convert_joint_state(role: JointRole, msg: JointState) -> RerunMulti:
    """Module-level converter (picklable so worker subprocesses can deploy it)."""
    out: RerunMulti = []
    for joint_name, position in zip(msg.name, msg.position, strict=False):
        short = joint_name.split("/")[-1]
        out.append((_entity_path(role, short), rr.Scalars([position])))
    return out


def joint_state_to_rerun_scalars(role: JointRole) -> Callable[[JointState], RerunMulti]:
    """Build a visual_override converter that emits per-joint scalars.

    The returned callable takes a `JointState` and produces a list of
    `(entity_path, rr.Scalars)` tuples — one per joint in the incoming message.
    Joints not present in the message are skipped (never raises). Joint names
    are short-cut to their last `/`-segment to match the preset layout.

    Returns a ``functools.partial`` (not a closure) so the result is picklable
    and can survive `multiprocessing` deployment to worker subprocesses.
    """
    return partial(_convert_joint_state, role)


def _joint_plot(short_name: str) -> rrb.TimeSeriesView:
    """Build the per-joint plot pairing measured and commanded series."""
    return rrb.TimeSeriesView(
        name=short_name,
        contents=[
            _entity_path("measured", short_name),
            _entity_path("commanded", short_name),
        ],
    )


def piper_data_collection_rerun_blueprint() -> rrb.Blueprint:
    """Default Rerun layout for `teleop_quest_piper_data_collection`.

    Camera on the left, one TimeSeriesView per joint stacked on the right with
    measured + commanded overlaid. Layout is locked (`auto_layout=False`,
    `auto_views=False`) so panels do not rearrange as entities arrive.
    """
    joint_plots = [_joint_plot(name) for name in piper_data_collection_joint_short_names()]
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(
                origin=_CAMERA_ENTITY_PATH,
                name="USB camera",
            ),
            rrb.Vertical(*joint_plots),
            column_shares=[2, 1],
        ),
        auto_layout=False,
        auto_views=False,
        collapse_panels=True,
    )


# ── Session / episode helpers ────────────────────────────────────────────────


def default_session_name() -> str:
    """Generate a session directory name as a UTC ``YYYYMMDDTHHMMSSZ`` timestamp."""
    return datetime.now(tz=timezone.utc).strftime("%Y%m%dT%H%M%SZ")


@dataclass
class _EpisodePathFactory:
    """Stateful per-episode path factory (kept as a top-level dataclass so it
    is picklable for multiprocessing deployment of the data collection
    blueprint)."""

    session_name: str | None = None
    counter: int = 0

    def __call__(self) -> Path:
        if self.session_name is None:
            self.session_name = default_session_name()
        self.counter += 1
        return (
            get_data_dir()
            / "piper_data_collection"
            / self.session_name
            / f"episode_{self.counter:03d}.rrd"
        )


def default_episode_path_factory(session_name: str | None = None) -> Callable[[], Path]:
    """Return a stateful factory producing per-episode `.rrd` paths.

    Each call yields ``{data_dir}/piper_data_collection/<session>/episode_<NNN>.rrd``
    with a monotonically increasing 1-indexed counter. When ``session_name`` is
    ``None`` the factory generates one on its first call and reuses it for the
    rest of the session.
    """
    return _EpisodePathFactory(session_name=session_name)


def default_recording_id_factory(path: Path) -> str:
    """Default ``recording_id`` derivation from a per-episode `.rrd` path.

    ``…/<session>/episode_<NNN>.rrd`` → ``<session>/episode_<NNN>``.
    """
    return f"{path.parent.name}/{path.stem}"


def _build_piper_episode_metadata(session_id: str, operator: str | None, n: int) -> dict[str, str]:
    out: dict[str, str] = {
        "episode_id": f"{session_id}/episode_{n:03d}",
        "episode_index": str(n),
        "session_id": session_id,
    }
    if operator is not None:
        out["operator"] = operator
    return out


def piper_episode_metadata(
    session_id: str,
    operator: str | None,
) -> Callable[[int], dict[str, str]]:
    """Build a per-episode metadata callable for ``RerunDataRecorder``.

    Given a 1-indexed episode number ``n``, the returned callable yields::

        {
          "episode_id": f"{session_id}/episode_{n:03d}",
          "episode_index": str(n),
          "session_id": session_id,
          "operator": operator,                       # omitted when None
        }

    Returned as a ``functools.partial`` over a module-level function so it
    pickles cleanly for multiprocessing deployment.
    """
    return partial(_build_piper_episode_metadata, session_id, operator)


def _piper_data_collection_topic_to_entity(topic: Any) -> str:
    """Topic→entity_path callback consumed by the bridge (live viewer).

    Maps every pubsub topic to its bare topic name (LCM ``#Type`` suffix
    stripped), so visual_override keys match the topic name directly. The
    recorder no longer goes through this callback — all of its inputs
    arrive through typed ``In[T]`` slots.
    """
    name = getattr(topic, "name", None) or str(topic)
    return name.split("#")[0]


def piper_data_collection_rerun_config(
    *,
    session_name: str | None = None,
    operator: str | None = None,
    record_path_factory: Callable[[], Path] | None = None,
) -> dict[str, Any]:
    """Single source-of-truth config dict for the viewer + the recorder.

    The same dict is passed to ``vis_module("rerun", rerun_config=...)`` and
    ``RerunDataRecorder.blueprint(...)`` from the data collection blueprint.
    Shared object identity is the structural drift guard between the two
    sinks — see design.md Decision 5.

    Recorder-only fields (``record_path_factory``, ``recording_id_factory``,
    ``episode_metadata``, ``app_id``) are silently ignored by the bridge.

    ``operator`` defaults to the ``DIMOS_OPERATOR`` env var (or absent).
    ``record_path_factory`` defaults to the shipped per-episode path factory.
    """
    if record_path_factory is None:
        record_path_factory = default_episode_path_factory(session_name)
    if operator is None:
        operator = os.environ.get("DIMOS_OPERATOR")

    # session_id is the directory the path factory will write into. Use the
    # session_name argument if provided; otherwise we cannot know it until the
    # factory runs, so we hand the closure a placeholder that is rewritten on
    # first episode (the factory mutates state; we read it after the first call).
    session_id = session_name or "session"
    episode_metadata = piper_episode_metadata(session_id, operator)

    return {
        "visual_override": {
            _JOINT_STATE_ENTITY_PATH: joint_state_to_rerun_scalars("measured"),
            _DESIRED_JOINT_ACTION_ENTITY_PATH: joint_state_to_rerun_scalars("commanded"),
        },
        "entity_prefix": "",
        "topic_to_entity": _piper_data_collection_topic_to_entity,
        "blueprint": piper_data_collection_rerun_blueprint,
        # Recorder-only fields below; the bridge ignores them.
        "camera_key": _CAMERA_KEY,
        "record_path_factory": record_path_factory,
        "recording_id_factory": default_recording_id_factory,
        "episode_metadata": episode_metadata,
    }
