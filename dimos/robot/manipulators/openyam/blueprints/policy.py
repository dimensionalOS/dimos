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

"""OpenYAM history policy driven by Quest input against a MuJoCo plant.

Two runnable blueprints, one stack:

``openyam-policy-sim``
    A Quest/WebXR headset publishes the right controller pose and the deadman
    grip; the coordinator runs :class:`OpenYamPolicyTask` at the trained 30 Hz
    against the *trained* MuJoCo plant through the ``sim_mujoco`` adapter.

``openyam-policy-sim-replay``
    The same stack with scripted controller input
    (:class:`~dimos.teleop.webxr.replay.WebXRReplayModule`) instead of a
    headset, for machines without a Quest. It is labelled input replay.

Neither blueprint can construct a CAN/Damiao connection: the only hardware
component is a ``sim_mujoco`` adapter that attaches to the blueprint's
:class:`MujocoSimModule` over shared memory. Artifacts are not committed -
pass ``--model-dir`` / ``--manifest-path`` (or the ``DIMOS_OPENYAM_POLICY_*``
environment variables) and the plant comes from the manifest's
``metadata.plant_model``.

The stack pins its own zenoh domain **and** its own multicast scout group
(:data:`ISOLATION_DOMAIN`, :data:`ISOLATION_SCOUT_ADDR`). Both are needed:
stream keys are domain-scoped (`dimos-domains/<domain>/...`) but coordinator RPC
keys are not (`dimos/rpc/<name>`), so a second DimOS stack on the host - even a
different robot's - answers this run's RPCs unless discovery is separated too.
That is not hypothetical: a long-running ``r1pro-vr-teleop`` instance on the
default bus fed foreign teleop commands into this blueprint and drove the arm
onto a joint limit.

The coordinator tick rate is 60 Hz: an exact 2x the trained 30 Hz policy rate,
and comfortably above this host's measured per-tick cost (~10 ms), so the
task's deadline cadence really lands every other tick. Asking for 120 Hz makes
the tick loop overrun its period and deliver ~100 Hz, which degenerates the
actor cadence to a 40 ms / 25 Hz period.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import os
from pathlib import Path
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.spec import JointLimits
from dimos.robot.manipulators.common.blueprints import coordinator
from dimos.robot.manipulators.openyam.config import OPENYAM_HOME_JOINTS
from dimos.robot.manipulators.openyam.policy.contract import JOINT_ORDER, load_manifest
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.teleop.webxr.extensions import ArmTeleopModule
from dimos.teleop.webxr.replay import WebXRReplayModule

#: Task name operators see in ``state_snapshot`` and e-stop/release calls.
POLICY_TASK_NAME = "openyam_policy"
#: Hardware id; the task's joints are ``arm/<joint>`` in the trained order.
HARDWARE_ID = "arm"
#: 60 = 2 x 30: the trained 30 Hz cadence lands every second coordinator tick.
COORDINATOR_TICK_RATE = 60.0
#: Run the actor on every second tick instead of on a deadline. The deadline
#: logic quantizes to coordinator ticks anyway, so with a 16.7 ms tick a 33.3 ms
#: deadline lands on tick 2 or tick 3 depending on sub-millisecond jitter, which
#: shows up as a 33 ms / 50 ms actor interval. Decimation is exact.
ACTOR_EVERY_N_TICKS = 2
#: Streams of this stack live under ``dimos-domains/<domain>/``.
ISOLATION_DOMAIN = "openyam-policy-sim"
#: Private multicast group for discovery, so RPCs never reach another DimOS run.
ISOLATION_SCOUT_ADDR = "224.0.0.231:7446"

DEFAULT_MODEL_DIR = Path(
    os.environ.get(
        "DIMOS_OPENYAM_POLICY_DIR",
        str(Path("~/.local/share/dimos/openyam_policy").expanduser()),
    )
)
DEFAULT_MANIFEST_NAME = "openyam_policy.manifest.json"


def policy_joint_names() -> list[str]:
    """The task's joint names: the robot prefix plus the checkpoint order."""
    return [f"{HARDWARE_ID}/{name}" for name in JOINT_ORDER]


def resolve_manifest_path(
    model_dir: str | Path | None = None, manifest_path: str | Path | None = None
) -> Path:
    """Locate the manifest, defaulting to the model dir's canonical name."""
    if manifest_path is not None:
        return Path(manifest_path)
    directory = Path(model_dir) if model_dir is not None else DEFAULT_MODEL_DIR
    return directory / DEFAULT_MANIFEST_NAME


def resolve_plant_path(
    model_dir: str | Path | None = None,
    manifest_path: str | Path | None = None,
    *,
    strict: bool = True,
) -> Path:
    """The plant the actor was trained in, pinned by the manifest.

    Falls back to ``DIMOS_OPENYAM_POLICY_PLANT`` and then to ``plant.xml`` next
    to the manifest only when a deployed manifest does not pin one, and never
    invents a path: a missing plant is an error, because the goal TCP frame is
    defined by that model. ``strict=False`` is for module import, where reading
    artifacts must not be required; the task itself always re-reads the
    manifest and fails if it pins a different plant.
    """
    manifest_file = resolve_manifest_path(model_dir, manifest_path)
    plant: str | None = None
    if manifest_file.is_file():
        plant = load_manifest(manifest_file).metadata.get("plant_model")
    elif strict:
        raise FileNotFoundError(f"policy manifest not found: {manifest_file}")
    if not plant:
        plant = os.environ.get("DIMOS_OPENYAM_POLICY_PLANT")
    if not plant and not strict:
        plant = str(manifest_file.parent / "plant.xml")
    if not plant:
        raise ValueError(
            "the policy manifest does not pin metadata.plant_model and "
            "DIMOS_OPENYAM_POLICY_PLANT is unset"
        )
    path = Path(plant)
    if strict and not path.is_file():
        raise FileNotFoundError(f"trained plant model not found: {path}")
    return path


def policy_hardware(plant: Path) -> HardwareComponent:
    """Six trained arm joints on the simulated plant; never a CAN adapter."""
    lower = [-2.61799, 0.0, 0.0, -1.65806, -1.5708, -2.35619]
    upper = [3.14159, 3.66519, 4.01426, 1.65806, 1.5708, 1.8326]
    return HardwareComponent(
        hardware_id=HARDWARE_ID,
        hardware_type=HardwareType.MANIPULATOR,
        joints=policy_joint_names(),
        adapter_type="sim_mujoco",
        address=str(plant),
        limits=JointLimits(
            position_lower=lower,
            position_upper=upper,
            velocity_max=[2.0] * len(JOINT_ORDER),
        ),
    )


def policy_task_config(
    *,
    model_dir: Path,
    manifest_path: Path,
    reference_source: str = "teleop",
    clip_path: Path | None = None,
    clip_anchor_index: int | None = None,
) -> TaskConfig:
    """The coordinator task entry for the trained policy."""
    params: dict[str, Any] = {
        "model_dir": str(model_dir),
        "manifest_path": str(manifest_path),
        "reference_source": reference_source,
    }
    if reference_source == "clip":
        if clip_path is None:
            raise ValueError("reference_source='clip' requires a reference clip path")
        params["clip_path"] = str(clip_path)
        params["clip_anchor_index"] = clip_anchor_index
    return TaskConfig(
        name=POLICY_TASK_NAME,
        type="openyam_policy",
        joint_names=policy_joint_names(),
        priority=40,
        params={
            **params,
            # Two coordinator ticks per actor tick: a true 30 Hz actor cadence.
            "every_n_ticks": ACTOR_EVERY_N_TICKS,
            "feedback_timeout_s": 0.2,
            "intent_timeout_s": 0.5,
            # A controller sample older than a quarter second closes the session.
            "input_timeout_s": 0.25,
        },
    )


def sim_module(
    plant: Path,
    *,
    headless: bool = True,
    initial_positions: Sequence[float] | None = None,
    initial_velocities: Sequence[float] | None = None,
) -> Blueprint:
    """The trained plant in-process; the arm adapter attaches to it over SHM."""
    return MujocoSimModule.blueprint(
        address=str(plant),
        headless=headless,
        dof=len(JOINT_ORDER),
        reset_joint_positions=(
            list(initial_positions) if initial_positions is not None else list(OPENYAM_HOME_JOINTS)
        ),
        reset_joint_velocities=(
            list(initial_velocities) if initial_velocities is not None else None
        ),
        enable_color=False,
        enable_depth=False,
        enable_pointcloud=False,
    )


def transport_isolation(
    *, domain: str = ISOLATION_DOMAIN, scout_addr: str = ISOLATION_SCOUT_ADDR
) -> dict[str, str]:
    """GlobalConfig overrides that keep this stack off every other DimOS run.

    Domain scoping covers the typed streams; the private multicast group keeps
    this run's workers discovering only each other, which is what stops a
    foreign coordinator from answering ``dimos/rpc/<name>`` keys.

    Args:
        domain: Zenoh key namespace for this stack's streams.
        scout_addr: Multicast group/port used only by this stack.

    Returns:
        Keyword arguments for :meth:`Blueprint.global_config`.
    """
    return {"zenoh_domain": domain, "zenoh_scout_addr": scout_addr}


def _policy_sim(
    *,
    model_dir: str | Path | None = None,
    manifest_path: str | Path | None = None,
    headless: bool = True,
    replay: bool,
    strict: bool = True,
    initial_positions: Sequence[float] | None = None,
    initial_velocities: Sequence[float] | None = None,
    replay_config: Mapping[str, Any] | None = None,
    isolate: bool = True,
    reference_source: str = "teleop",
    clip_path: str | Path | None = None,
    clip_anchor_index: int | None = None,
) -> Blueprint:
    resolved_manifest = resolve_manifest_path(model_dir, manifest_path)
    resolved_model_dir = Path(model_dir) if model_dir is not None else resolved_manifest.parent
    plant = resolve_plant_path(resolved_model_dir, resolved_manifest, strict=strict)

    modules: list[Any] = [
        sim_module(
            plant,
            headless=headless,
            initial_positions=initial_positions,
            initial_velocities=initial_velocities,
        ),
        coordinator(
            hardware=[policy_hardware(plant)],
            tasks=[
                policy_task_config(
                    model_dir=resolved_model_dir,
                    manifest_path=resolved_manifest,
                    reference_source=reference_source,
                    clip_path=Path(clip_path) if clip_path is not None else None,
                    clip_anchor_index=clip_anchor_index,
                )
            ],
            tick_rate=COORDINATOR_TICK_RATE,
            cls=TeleopControlCoordinator,
            instance_name="ControlCoordinator",
        ),
    ]
    remaps: list[tuple[Any, str, str]] = []
    if reference_source == "clip":
        # The recorded reference needs no controller: the actor is driven from
        # the frozen clip, which is how the integrated chain is compared with
        # the original training rollout.
        if replay:
            raise ValueError("reference_source='clip' does not take a WebXR replay input")
    elif replay:
        modules.insert(0, WebXRReplayModule.blueprint(**(dict(replay_config or {}))))
        remaps.append((WebXRReplayModule, "right_controller_output", "right_cartesian_command"))
    else:
        modules.insert(0, ArmTeleopModule.blueprint(zero_button="B"))
        remaps.append((ArmTeleopModule, "right_controller_output", "right_cartesian_command"))

    blueprint = autoconnect(*modules)
    if remaps:
        blueprint = blueprint.remappings(remaps)
    if isolate:
        blueprint = blueprint.global_config(**transport_isolation())
    return blueprint


def openyam_policy_sim(
    *,
    model_dir: str | Path | None = None,
    manifest_path: str | Path | None = None,
    headless: bool = True,
    initial_positions: Sequence[float] | None = None,
    isolate: bool = True,
) -> Blueprint:
    """Quest controller -> policy -> trained MuJoCo plant, no hardware path.

    Args:
        model_dir: Directory with the exported ONNX actor, normalizer and manifest.
        manifest_path: Manifest pinning the artifacts, plant and contract.
        headless: Run MuJoCo without its interactive viewer.
        initial_positions: Plant reset pose; defaults to the OpenYAM home posture.
        isolate: Pin the private zenoh domain/scout group (see
            :func:`transport_isolation`). Only turn it off to join a stack that
            already runs on this exact bus.
    """
    return _policy_sim(
        model_dir=model_dir,
        manifest_path=manifest_path,
        headless=headless,
        replay=False,
        initial_positions=initial_positions,
        isolate=isolate,
    )


def openyam_policy_sim_replay(
    *,
    model_dir: str | Path | None = None,
    manifest_path: str | Path | None = None,
    headless: bool = True,
    initial_positions: Sequence[float] | None = None,
    replay_config: Mapping[str, Any] | None = None,
    isolate: bool = True,
) -> Blueprint:
    """The same stack with scripted WebXR input, labelled input replay.

    Args:
        model_dir: Directory with the exported ONNX actor, normalizer and manifest.
        manifest_path: Manifest pinning the artifacts, plant and contract.
        headless: Run MuJoCo without its interactive viewer.
        initial_positions: Plant reset pose; defaults to the OpenYAM home posture.
        replay_config: Script knobs for ``WebXRReplayConfig`` (stale input,
            re-anchor, freeze times) used by the safety runs.
        isolate: Pin the private zenoh domain/scout group.
    """
    return _policy_sim(
        model_dir=model_dir,
        manifest_path=manifest_path,
        headless=headless,
        replay=True,
        initial_positions=initial_positions,
        replay_config=replay_config,
        isolate=isolate,
    )


def openyam_policy_sim_clip(
    *,
    clip_path: str | Path,
    clip_anchor_index: int,
    model_dir: str | Path | None = None,
    manifest_path: str | Path | None = None,
    headless: bool = True,
    initial_positions: Sequence[float] | None = None,
    initial_velocities: Sequence[float] | None = None,
    isolate: bool = True,
) -> Blueprint:
    """Same stack driven by the recorded reference clip the training used.

    This is the comparison variant: the reference generator is a frozen
    :class:`~dimos.robot.manipulators.openyam.policy.reference.ClipReference`
    (``q_ref``, ``dq_ref`` and the recorded TCP goal at the same 30 Hz tick)
    and no WebXR input is constructed. It exists to run the integrated
    DimOS chain from the same clip, plant and initial state as the original
    training rollout, so the two can be compared - it is not a deployment
    input path and does not replace the Quest blueprint.
    """
    return _policy_sim(
        model_dir=model_dir,
        manifest_path=manifest_path,
        headless=headless,
        replay=False,
        initial_positions=initial_positions,
        initial_velocities=initial_velocities,
        isolate=isolate,
        reference_source="clip",
        clip_path=clip_path,
        clip_anchor_index=clip_anchor_index,
    )


#: Quest headset -> policy -> MuJoCo. Inert until the deadman is pressed.
openyam_policy_sim_blueprint = _policy_sim(replay=False, strict=False)
#: Scripted WebXR input -> policy -> MuJoCo, for machines without a headset.
openyam_policy_sim_replay_blueprint = _policy_sim(replay=True, strict=False)
