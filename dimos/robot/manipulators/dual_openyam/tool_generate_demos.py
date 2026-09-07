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

"""Generate and physically verify simulated bottle demonstrations. Run with --help."""

import argparse
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import time
from typing import Any, TypeVar, cast
import xml.etree.ElementTree as ET

import numpy as np

from dimos.agents.skill_result import SkillResult
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.dataprep.core import (
    EpisodeExtractor,
    EpisodeQualityReport,
    OutputConfig,
    extract_episodes,
    inspect_episode_quality,
)
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.robot.manipulators.dual_openyam.blueprints.sim_learning import (
    build_dual_openyam_sim_collection,
)
from dimos.robot.manipulators.dual_openyam.blueprints.simulation import dual_openyam_sim_pick_place
from dimos.robot.manipulators.dual_openyam.learning import (
    DUAL_OPENYAM_LEROBOT_IO,
    DUAL_OPENYAM_SIM_CAPTURE_FPS,
    DUAL_OPENYAM_SIM_TASK,
)
from dimos.robot.manipulators.dual_openyam.sim import (
    DUAL_OPENYAM_SCENE_PATH,
    dual_openyam_sim_model_config,
)
from dimos.robot.manipulators.dual_openyam.sim_demo import (
    SimDemoSkills,
    bounds,
    inside_bin,
    planning_group_for_y,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

Error = TypeVar("Error", bound=str)


def require(result: SkillResult[Error]) -> SkillResult[Error]:
    if not result.success:
        raise RuntimeError(f"{result.error_code}: {result.message}")
    return result


@dataclass(frozen=True)
class EpisodeSetup:
    target: str
    group: str
    initial_z: float
    object_height: float


@contextmanager
def recording_episode(monitor: EpisodeMonitorModule | None) -> Iterator[None]:
    """Save only a completed verified take; discard failures and interruptions."""
    if monitor is None:
        yield
        return
    if monitor.get_status().state != "idle":
        raise RuntimeError("An episode is already recording")
    try:
        if monitor.command("start").state != "recording":
            raise RuntimeError("Recorder did not start an episode")
        yield
        if monitor.command("save").state != "idle":
            raise RuntimeError("Recorder did not finish the episode")
    except BaseException:
        monitor.command("discard")
        raise


def prepare_episode(
    sim: MujocoSimModule,
    pick: PickAndPlaceModule,
    target: str,
    rng: np.random.Generator,
    jitter: float,
) -> EpisodeSetup:
    pose = sim.get_body_poses([target])[target]
    pose[0] += float(rng.uniform(-jitter, jitter))
    pose[1] += float(rng.uniform(-jitter, jitter))
    if not sim.set_body_pose(target, pose[:3], pose[3:]):
        raise RuntimeError("Could not randomize bottle")
    time.sleep(0.5)
    start = sim.get_body_poses([target])[target]
    group = planning_group_for_y(start[1])
    require(pick.scan_objects([target]))
    object_lower, object_upper = bounds(sim.sample_body_surface(target, 4096))
    return EpisodeSetup(target, group, start[2], float(object_upper[2] - object_lower[2]))


def run_episode(
    sim: MujocoSimModule,
    pick: PickAndPlaceModule,
    skills: ManipulationSkills,
    setup: EpisodeSetup,
    hold_seconds: float,
) -> dict[str, Any]:
    target, group = setup.target, setup.group
    pick_result = require(pick.pick_object(target, planning_group=group))
    deadline = time.monotonic() + hold_seconds
    lift = float("inf")
    while True:
        current = sim.get_body_poses([target])[target]
        lift = min(lift, current[2] - setup.initial_z)
        if lift < 0.05:
            raise RuntimeError(f"Bottle did not stay lifted: {lift:.4f} m")
        if time.monotonic() >= deadline:
            break
        time.sleep(0.1)
    bin_points = sim.sample_body_surface("bin_container", 8192)
    bin_lower, bin_upper = bounds(bin_points)
    center = (bin_lower + bin_upper) / 2
    release_z = float(bin_upper[2] + setup.object_height / 2 + 0.015)
    require(pick.place_at(float(center[0]), float(center[1]), release_z, planning_group=group))
    require(skills.go_home(planning_group=group))
    time.sleep(1.0)
    final_points = sim.sample_body_surface(target, 8192)
    if not inside_bin(final_points, sim.sample_body_surface("bin_container", 8192)):
        lower, upper = bounds(final_points)
        raise RuntimeError(
            f"Bottle outside bin after release: {lower.tolist()} .. {upper.tolist()}"
        )
    return {
        "object": target,
        "group": group,
        "candidate_rank": pick_result.metadata["rank"],
        "minimum_lift_m": lift,
        "final_pose": sim.get_body_poses([target])[target],
    }


def planning_model_sha256(xml: str) -> str:
    """Hash the expanded URDF and mesh bytes independently of checkout location."""
    root = ET.fromstring(xml)
    for mesh in root.iter("mesh"):
        path = Path(mesh.attrib["filename"].removeprefix("file://"))
        mesh.set("filename", hashlib.sha256(path.read_bytes()).hexdigest())
    canonical = ET.canonicalize(ET.tostring(root, encoding="unicode"), strip_text=True)
    return hashlib.sha256(canonical.encode()).hexdigest()


def recording_manifest() -> dict[str, Any]:
    profile = DUAL_OPENYAM_LEROBOT_IO
    return {
        "scene_sha256": hashlib.sha256(DUAL_OPENYAM_SCENE_PATH.read_bytes()).hexdigest(),
        "planning_model_sha256": planning_model_sha256(
            dual_openyam_sim_model_config().model.load().xml
        ),
        "profile": profile.name,
        "io_contract": profile.model_dump(mode="json"),
        "task": DUAL_OPENYAM_SIM_TASK,
        "camera_rate_hz": DUAL_OPENYAM_SIM_CAPTURE_FPS,
        "dataset_rate_hz": profile.sync.rate_hz,
        "joint_names": list(profile.action.demonstration.joints),
    }


def quality_of_saved_episode(path: Path, after_ts: float) -> EpisodeQualityReport:
    """Wait for durable recorder coverage, then apply the same gates as dataset export."""
    config = DUAL_OPENYAM_LEROBOT_IO.dataprep_config(
        source=str(path), output=OutputConfig(path=path.with_suffix(""))
    )
    features = {**config.observation, **config.action}
    store = SqliteStore(path=str(path), must_exist=True)
    deadline = time.monotonic() + 10.0
    try:
        while time.monotonic() < deadline:
            episodes = extract_episodes(store, EpisodeExtractor())
            recent = [
                episode for episode in episodes if episode.start_ts >= after_ts and episode.success
            ]
            if recent:
                episode = recent[-1]
                latest: list[Observation[Any] | None] = [
                    store.stream(feature.stream).last() for feature in features.values()
                ]
                if all(value is not None and value.ts >= episode.end_ts for value in latest):
                    return inspect_episode_quality(
                        store, episode, features, config.sync, config.quality
                    )
            time.sleep(0.05)
        raise RuntimeError("Recorder did not persist a complete saved episode within 10 seconds")
    finally:
        store.stop()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--episodes",
        type=int,
        default=100,
        help="Number of new physically successful, timing-valid episodes",
    )
    parser.add_argument("--max-attempts", type=int, default=None)
    parser.add_argument(
        "--recording", type=Path, help="New SQLite recording; omit for verification only"
    )
    parser.add_argument(
        "--report", type=Path, required=True, help="Write one JSON object per attempt"
    )
    parser.add_argument(
        "--resume", action="store_true", help="Continue a recording with a matching scene manifest"
    )
    parser.add_argument(
        "--n-workers",
        type=int,
        default=None,
        help="Default: one worker per collection module; four for verification",
    )
    parser.add_argument("--arm", choices=["right", "left", "both"], default="right")
    parser.add_argument(
        "--bimanual", action="store_true", help="Verify both picks after each reset; no recording"
    )
    parser.add_argument(
        "--jitter", type=float, default=0.015, help="Uniform XY spawn perturbation in meters"
    )
    parser.add_argument(
        "--warmup-episodes", type=int, default=1, help="Unrecorded cycles before collection"
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument(
        "--zenoh-scout-addr", default="", help="Use a distinct discovery bus for concurrent runs"
    )
    args = parser.parse_args()
    if args.episodes < 1 or args.jitter < 0 or args.hold_seconds < 0 or args.warmup_episodes < 0:
        parser.error("episodes must be positive; jitter and hold time must be nonnegative")
    if args.bimanual and args.recording:
        parser.error("--bimanual is for verification; collect one target per episode")
    if args.n_workers is not None and args.n_workers < 1:
        parser.error("n-workers must be positive")
    if args.resume and (not args.recording or not args.recording.is_file()):
        parser.error("--resume requires an existing recording")
    if args.recording and args.recording.exists() and not args.resume:
        parser.error("recording already exists; --resume explicitly continues it")
    if args.recording:
        manifest = recording_manifest()
        manifest_path = args.recording.with_suffix(".scene.json")
        if args.resume:
            if not manifest_path.is_file() or json.loads(manifest_path.read_text()) != manifest:
                parser.error("recording scene/profile manifest is missing or incompatible")
        else:
            manifest_path.parent.mkdir(parents=True, exist_ok=True)
            with manifest_path.open("x") as stream:
                json.dump(manifest, stream, indent=2)
    if args.report.exists():
        parser.error("report already exists")
    base = (
        build_dual_openyam_sim_collection(
            recording=args.recording, task=DUAL_OPENYAM_SIM_TASK, resume=args.resume
        )
        if args.recording
        else dual_openyam_sim_pick_place
    )
    workers = args.n_workers or (len(base.blueprints) if args.recording else 4)
    base = base.global_config(
        viewer="none", n_workers=workers, zenoh_scout_addr=args.zenoh_scout_addr
    )
    rng = np.random.default_rng(args.seed)
    coordinator = None
    saved = 0
    attempts = args.max_attempts or 2 * args.episodes
    args.report.parent.mkdir(parents=True, exist_ok=True)
    try:
        coordinator = ModuleCoordinator.build(base)
        sim = cast("MujocoSimModule", coordinator.get_instance("MujocoSimModule"))
        pick = cast("PickAndPlaceModule", coordinator.get_instance(PickAndPlaceModule))
        skills = cast("ManipulationSkills", coordinator.get_instance(ManipulationSkills))
        demo = cast("SimDemoSkills", coordinator.get_instance(SimDemoSkills))
        monitor = (
            cast("EpisodeMonitorModule", coordinator.get_instance(EpisodeMonitorModule))
            if args.recording
            else None
        )
        time.sleep(2.0)
        if args.recording:
            for _ in range(args.warmup_episodes):
                require(demo.reset_scene())
                setup = prepare_episode(sim, pick, "bottle_1", rng, 0.0)
                run_episode(sim, pick, skills, setup, 0.0)
        with args.report.open("x") as report:
            for attempt in range(attempts):
                start = time.monotonic()
                row: dict[str, Any] = {"attempt": attempt, "success": False}
                try:
                    require(demo.reset_scene())
                    time.sleep(0.5)
                    target = (
                        "bottle_4"
                        if args.arm == "left" or (args.arm == "both" and attempt % 2)
                        else "bottle_1"
                    )
                    targets = ["bottle_1", "bottle_4"] if args.bimanual else [target]
                    outcomes = []
                    for selected in targets:
                        setup = prepare_episode(sim, pick, selected, rng, args.jitter)
                        recording_started = time.time()
                        with recording_episode(monitor):
                            outcomes.append(
                                run_episode(sim, pick, skills, setup, args.hold_seconds)
                            )
                    if args.bimanual:
                        for selected in targets:
                            if not inside_bin(
                                sim.sample_body_surface(selected, 8192),
                                sim.sample_body_surface("bin_container", 8192),
                            ):
                                raise RuntimeError(
                                    f"{selected} left the bin during the second placement"
                                )
                        row["objects"] = outcomes
                    else:
                        row.update(outcomes[0])
                    row["physical_success"] = True
                    if args.recording:
                        quality = quality_of_saved_episode(args.recording, recording_started)
                        row["quality"] = quality.model_dump()
                        if not quality.valid:
                            raise RuntimeError("; ".join(quality.rejection_reasons))
                    saved += 1
                    row["success"] = True
                except Exception as exc:
                    row["error"] = str(exc)
                row["duration_s"] = time.monotonic() - start
                report.write(json.dumps(row) + "\n")
                report.flush()
                print(json.dumps(row), flush=True)
                if saved >= args.episodes:
                    break
    finally:
        if coordinator:
            coordinator.stop()
    print(json.dumps({"saved": saved, "requested": args.episodes}), flush=True)
    return 0 if saved == args.episodes else 1


if __name__ == "__main__":
    raise SystemExit(main())
