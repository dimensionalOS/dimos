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

"""Reproducible state-distribution experiments; no production sim2 changes.

Run from the experiment worktree with ``python -m
dimos.sim2.experiments.demo_state_scaling --help``. Source dependencies are
unpinned only on disk: prepare() checks their exact commits before importing.
Four pipelines share one compiled MuJoCo world and the same sampled state:
Pim's actual snapshot, an experimental compact CBOR sample, Jeff's actual
WorldChange, and Jeff's WorldChange with its comparison caches cleared.
The last pair isolates change suppression from representation/codec costs.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import cProfile
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import importlib
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import pstats
import random
import subprocess
import sys
import time
import tracemalloc
from types import SimpleNamespace
from typing import Any
import xml.etree.ElementTree as ET

import dimos_lcm
import mujoco
import numpy as np
import psutil

from dimos.protocol.encode import cbor
from dimos.robot.assets.model import RobotModel
import dimos.sim2
from dimos.sim2.mujoco import MujocoBackend, compile_world
from dimos.sim2.types import Drive, Instance, World

PIM_SHA = "ea95c4486d2ad28135a6ca7d3427c501bea53730"
JEFF_SHA = "05c7a719bc9d727ed7c0c2832ec045ae369e6850"
LCM_SHA = "7d3f7cf2611fbb8c911f3fcd99fa14a370381eb5"
STRATEGIES = ("pim_full", "compact_full", "jeff_full", "jeff_delta")
ROOT = Path(__file__).resolve().parents[3]
OUTPUT = ROOT / "outputs/state-scaling"


def command(*args: str) -> str:
    return subprocess.check_output(args, text=True).strip()


def prepare() -> SimpleNamespace:
    """Load unmodified pinned Jeff modules beside non-overlapping Pim modules."""
    dependencies = OUTPUT / "deps"
    jeff = dependencies / "jeff/dimos/sim2"
    generated = dependencies / "dimos-lcm/generated/python_lcm_msgs/lcm_msgs"
    if not jeff.is_dir() or not generated.is_dir():
        raise RuntimeError("prepare pinned dependencies using the experiment README")
    if command("git", "-C", str(dependencies / "dimos-lcm"), "rev-parse", "HEAD") != LCM_SHA:
        raise RuntimeError("world_msgs source revision mismatch")
    for file in (
        "world_change.py",
        "mujoco_backend.py",
        "world.py",
        "backend.py",
        "actions.py",
        "spec.py",
    ):
        expected = subprocess.check_output(
            ["git", "show", f"{JEFF_SHA}:dimos/sim2/{file}"], cwd=ROOT
        )
        if (jeff / file).read_bytes() != expected:
            raise RuntimeError(f"Jeff source is modified: {file}")
    for file in ("mujoco.py", "types.py", "runtime.py", "module.py", "sensors.py"):
        expected = subprocess.check_output(
            ["git", "show", f"{PIM_SHA}:dimos/sim2/{file}"], cwd=ROOT
        )
        if (ROOT / "dimos/sim2" / file).read_bytes() != expected:
            raise RuntimeError(f"Pim source is modified: {file}")
    for package, path in ((dimos.sim2, jeff), (dimos_lcm, generated)):
        if str(path) not in package.__path__:
            package.__path__.append(str(path))
    return SimpleNamespace(
        backend=importlib.import_module("dimos.sim2.mujoco_backend"),
        world=importlib.import_module("dimos.sim2.world_change"),
        messages=importlib.import_module("dimos_lcm.world_msgs"),
    )


def provenance() -> dict[str, Any]:
    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "cpu": command("sysctl", "-n", "machdep.cpu.brand_string"),
        "memory_bytes": psutil.virtual_memory().total,
        "power": command("pmset", "-g", "batt"),
        "power_configuration": command("pmset", "-g", "custom"),
        "load": psutil.getloadavg(),
        "versions": {
            p: importlib.metadata.version(p)
            for p in ("mujoco", "numpy", "cbor2", "eclipse-zenoh", "pytest")
        },
        "pim": PIM_SHA,
        "jeff": JEFF_SHA,
        "world_msgs": LCM_SHA,
        "harness_commit": command("git", "-C", str(ROOT), "rev-parse", "HEAD"),
        "harness_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }


def box_link(root: ET.Element, name: str, size: tuple[float, float, float], mass: float) -> None:
    link = ET.SubElement(root, "link", name=name)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value=str(mass))
    x, y, z = size
    ET.SubElement(
        inertial,
        "inertia",
        ixx=str(mass * (y * y + z * z) / 12),
        iyy=str(mass * (x * x + z * z) / 12),
        izz=str(mass * (x * x + y * y) / 12),
        ixy="0",
        ixz="0",
        iyz="0",
    )
    for kind in ("visual", "collision"):
        geom = ET.SubElement(ET.SubElement(link, kind), "geometry")
        ET.SubElement(geom, "box", size=" ".join(map(str, size)))


def scene(directory: Path, props: int, robots: int = 1, joints: int = 29) -> World:
    """Scalable primitive collision world. Synthetic chains, not G1 policy tests."""
    directory.mkdir(parents=True, exist_ok=True)
    root = ET.Element("robot", name="articulated_chain")
    box_link(root, "base", (0.08, 0.08, 0.08), 1.0)
    for i in range(joints):
        box_link(root, f"link{i}", (0.025, 0.025, 0.025), 0.05)
        joint = ET.SubElement(root, "joint", name=f"joint{i}", type="revolute")
        ET.SubElement(joint, "parent", link="base" if i == 0 else f"link{i - 1}")
        ET.SubElement(joint, "child", link=f"link{i}")
        ET.SubElement(joint, "origin", xyz=".045 0 0")
        ET.SubElement(joint, "axis", xyz="0 0 1")
        ET.SubElement(joint, "limit", lower="-1", upper="1", effort="5", velocity="5")
        ET.SubElement(joint, "dynamics", damping=".01")
    robot_file = directory / "chain.urdf"
    ET.ElementTree(root).write(robot_file, encoding="unicode")
    model = RobotModel.from_file(robot_file)
    for i in range(4):
        model = model.with_fixed_frame(f"sensor{i}", f"link{joints - 1}", xyz=(0, 0, 0.02 * i))
    instances = [
        Instance(
            f"robot{i}",
            model.with_prefix(f"robot{i}/"),
            xyz=(0, 2 * i, 0.2),
            drives=tuple(Drive(f"robot{i}/joint{j}", kp=1, kd=0.1) for j in range(joints)),
        )
        for i in range(robots)
    ]
    prop = ET.Element("robot", name="prop")
    box_link(prop, "box", (0.1, 0.1, 0.1), 0.2)
    prop_file = directory / "prop.urdf"
    ET.ElementTree(prop).write(prop_file, encoding="unicode")
    prop_model = RobotModel.from_file(prop_file)
    width = max(1, math.ceil(math.sqrt(props)))
    for i in range(props):
        instances.append(
            Instance(
                f"prop{i}",
                prop_model.with_prefix(f"prop{i}/"),
                xyz=(0.18 * (i % width), 2 * robots + 0.18 * (i // width), 0.055),
                floating=True,
            )
        )
    return World(tuple(instances))


@dataclass
class WorldPair:
    pim: MujocoBackend
    jeff: Any
    modules: SimpleNamespace
    world: World
    root_ids: np.ndarray
    root_rows: np.ndarray
    free_q: np.ndarray
    free_v: np.ndarray
    initial_qpos: np.ndarray
    initial_ctrl: np.ndarray
    names: list[str]

    @classmethod
    def build(cls, directory: Path, props: int, robots: int = 1, joints: int = 29) -> WorldPair:
        modules = prepare()
        world = scene(directory, props, robots, joints)
        mjcf = directory / "shared.xml"
        # MjModel compiled from MjSpec has no mj_saveLastXML source. Observe the
        # real compiler's return to retain its spec, without replacing any code.
        # This startup-only probe is never active in a timed region.
        previous_profiler = sys.getprofile()

        def retain_spec(frame: Any, event: str, value: Any) -> None:
            if event == "return" and frame.f_code is compile_world.__code__:
                exported = ET.fromstring(frame.f_locals["spec"].to_xml())
                # MuJoCo 3.10 emits anonymous empty child defaults for attached
                # URDFs; those do nothing but cannot be read back as MJCF.
                for parent in exported.iter():
                    for child in list(parent):
                        if child.tag == "default" and not child.attrib and not len(child):
                            parent.remove(child)
                mjcf.write_text(ET.tostring(exported, encoding="unicode"))

        try:
            sys.setprofile(retain_spec)
            pim = MujocoBackend(world)
        finally:
            sys.setprofile(previous_profiler)
        jeff = modules.backend.MujocoBackend(
            SimpleNamespace(scene=mjcf, speed=1.0, controller_timeout=0.5), {}, directory
        )
        # Prove the shared model layout and physical parameters survive export/reload.
        for field in (
            "body_parentid",
            "body_mass",
            "body_inertia",
            "body_pos",
            "body_quat",
            "jnt_type",
            "jnt_qposadr",
            "jnt_dofadr",
            "geom_size",
            "geom_pos",
            "geom_type",
            "geom_friction",
            "dof_damping",
            "actuator_gainprm",
            "actuator_biasprm",
            "qpos0",
        ):
            np.testing.assert_allclose(
                getattr(pim.model, field), getattr(jeff.model, field), rtol=1e-5, atol=1e-8
            )
        assert pim.model.names == jeff.model.names
        # Extraction experiment only: both unmodified capture methods read the EXACT same MjData/model.
        # Each backend was constructed normally; no mock objects or substituted capture methods.
        jeff.model, jeff.data = pim.model, pim.data
        roots = np.asarray(list(jeff._roots.values()), dtype=int)
        free = np.flatnonzero(pim.model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
        result = cls(
            pim,
            jeff,
            modules,
            world,
            roots,
            roots - 1,
            pim.model.jnt_qposadr[free],
            pim.model.jnt_dofadr[free],
            pim.data.qpos.copy(),
            pim.data.ctrl.copy(),
            list(jeff._roots),
        )
        result.reset()
        return result

    def reset(self) -> None:
        mujoco.mj_resetData(self.pim.model, self.pim.data)
        self.pim.data.qpos[:] = self.initial_qpos
        self.pim.data.ctrl[:] = self.initial_ctrl
        mujoco.mj_forward(self.pim.model, self.pim.data)
        self.jeff.world = self.modules.world.WorldWriter()
        self.jeff._announced = False
        self.jeff._sent.clear()
        self.jeff._sent_joints.clear()

    def pose_sequence(self, frame: int, fraction: float, period: float = 0.02) -> None:
        """Controlled kinematic activity, NOT a physical trajectory or solver benchmark."""
        data, backend = self.pim.data, self.pim
        data.time = frame * period
        phase = 2 * math.pi * data.time
        data.qpos[backend._q] = 0.02 * np.sin(phase + np.arange(len(backend._q)) * 0.1)
        data.qvel[backend._v] = (
            0.02 * 2 * math.pi * np.cos(phase + np.arange(len(backend._v)) * 0.1)
        )
        count = math.ceil(len(self.free_q) * fraction)
        # Even resting props are dynamic bodies. Their values remain identical in this controlled test.
        data.qpos[self.free_q] = self.initial_qpos[self.free_q]
        data.qvel[self.free_v] = 0
        if count:
            phases = phase + np.arange(count) * 0.017
            data.qpos[self.free_q[:count]] += 0.01 * np.sin(phases)
            data.qvel[self.free_v[:count]] = 0.01 * 2 * math.pi * np.cos(phases)
        mujoco.mj_forward(backend.model, data)


class Pipeline:
    def __init__(self, pair: WorldPair, kind: str) -> None:
        self.pair, self.kind = pair, kind
        self.mirror = pair.modules.world.WorldMirror()
        self.tick = 0

    def capture(self) -> Any:
        p = self.pair
        self.tick += 1
        if self.kind == "pim_full":
            return p.pim.snapshot("experiment", round(p.pim.data.time / p.pim.dt))
        if self.kind == "compact_full":
            poses = np.column_stack(
                (p.pim.data.xpos[p.root_ids], p.pim.data.xquat[p.root_ids][:, [1, 2, 3, 0]])
            )
            twists = np.empty((len(p.root_ids), 6))
            velocity = np.empty(6)
            for i, body in enumerate(p.root_ids):
                mujoco.mj_objectVelocity(
                    p.pim.model, p.pim.data, mujoco.mjtObj.mjOBJ_XBODY, int(body), velocity, 0
                )
                twists[i, :3], twists[i, 3:] = velocity[3:], velocity[:3]
            return {
                "epoch": "experiment",
                "tick": round(p.pim.data.time / p.pim.dt),
                "sim_time": float(p.pim.data.time),
                "wall_time": time.time(),
                "revision": p.pim.layout.revision,
                "poses": poses,
                "twists": twists,
                "position": p.pim.data.qpos[p.pim._q].copy(),
                "velocity": p.pim.data.qvel[p.pim._v].copy(),
                "effort": p.pim.data.qfrc_actuator[p.pim._v].copy(),
            }
        if self.kind == "jeff_full":
            p.jeff._sent.clear()
            p.jeff._sent_joints.clear()
        return p.jeff.frame()

    def encode(self, value: Any) -> bytes:
        return (
            cbor.encode(value) if self.kind in ("pim_full", "compact_full") else value.lcm_encode()
        )

    def consume(self, payload: bytes) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return root poses, scalar joint rows, and root velocities for the SAME consumer workload."""
        p = self.pair
        return consume_state(
            self.kind,
            payload,
            self.mirror,
            p.root_rows,
            p.names,
            [name for name in p.names if p.jeff._joints[name][0]],
            p.modules,
        )


def consume_state(
    kind: str,
    payload: bytes,
    mirror: Any,
    root_rows: np.ndarray,
    names: list[str],
    joint_roots: list[str],
    modules: SimpleNamespace,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The identical receiver projection for inline and cross-process experiments."""
    if kind == "pim_full":
        state = cbor.decode(payload).state
        return (
            state.poses[root_rows],
            np.column_stack((state.position, state.velocity, state.effort)),
            state.twists[root_rows],
        )
    if kind == "compact_full":
        state = cbor.decode(payload)
        return (
            state["poses"],
            np.column_stack((state["position"], state["velocity"], state["effort"])),
            state["twists"],
        )
    mirror.apply(modules.messages.WorldChange.lcm_decode(payload))
    entities = [mirror.entity(name) for name in names]
    poses = np.asarray([entity.pose for entity in entities])
    velocities = np.asarray([entity.velocity for entity in entities])
    groups = [mirror.joints(name)[1] for name in joint_roots]
    return poses, np.concatenate(groups) if groups else np.empty((0, 3)), velocities


def summary(values: list[float]) -> dict[str, float]:
    array = np.asarray(values)
    return {
        "median": float(np.median(array)),
        "p95": float(np.quantile(array, 0.95)),
        "p99": float(np.quantile(array, 0.99)),
        "max": float(np.max(array)),
        "n": len(values),
    }


def reference(pair: WorldPair) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    full = pair.pim.snapshot("experiment", 0).state
    return (
        full.poses[pair.root_rows],
        np.column_stack((full.position, full.velocity, full.effort)),
        full.twists[pair.root_rows],
    )


def max_error(actual: tuple[np.ndarray, ...], expected: tuple[np.ndarray, ...]) -> float:
    return max(
        float(np.max(np.abs(a - b), initial=0)) for a, b in zip(actual, expected, strict=True)
    )


def one_replay(
    pair: WorldPair, kind: str, fraction: float, frames: int, repeat: int
) -> dict[str, Any]:
    pair.reset()
    pipe = Pipeline(pair, kind)
    timings: dict[str, list[float]] = defaultdict(list)
    error = 0.0
    samples = []
    initial_bytes = 0
    for i in range(frames + 12):
        pair.pose_sequence(i, fraction)
        expected = reference(pair)  # oracle deliberately outside all timed regions
        start = time.perf_counter_ns()
        packet = pipe.capture()
        captured = time.perf_counter_ns()
        payload = pipe.encode(packet)
        encoded = time.perf_counter_ns()
        result = pipe.consume(payload)
        consumed = time.perf_counter_ns()
        if i == 0:
            initial_bytes = len(payload)
        if i >= 12:
            timings["capture_us"].append((captured - start) / 1000)
            timings["encode_us"].append((encoded - captured) / 1000)
            timings["consume_us"].append((consumed - encoded) / 1000)
            timings["total_us"].append((consumed - start) / 1000)
            timings["bytes"].append(len(payload))
            error = max(error, max_error(result, expected))
            samples.append(
                [
                    i,
                    (captured - start) / 1000,
                    (encoded - captured) / 1000,
                    (consumed - encoded) / 1000,
                    len(payload),
                ]
            )
    return {
        "kind": kind,
        "props": len(pair.free_q),
        "robots": len(pair.world.instances) - len(pair.free_q),
        "fraction": fraction,
        "repeat": repeat,
        "frames": frames,
        "initial_bytes": initial_bytes,
        "error": error,
        "metrics": {k: summary(v) for k, v in timings.items()},
        "samples": samples,
    }


def allocations(pair: WorldPair, kind: str, fraction: float) -> dict[str, Any]:
    pair.reset()
    pipe = Pipeline(pair, kind)
    pipe.consume(pipe.encode(pipe.capture()))
    tracemalloc.start()
    before, _ = tracemalloc.get_traced_memory()
    tracemalloc.reset_peak()
    for i in range(20):
        pair.pose_sequence(i, fraction)
        pipe.consume(pipe.encode(pipe.capture()))
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return {
        "kind": kind,
        "props": len(pair.free_q),
        "fraction": fraction,
        "peak_traced_bytes_above_start": peak - before,
        "retained_bytes": current - before,
        "note": "Separate instrumented run; traced live Python/NumPy memory, not cumulative allocation or copy traffic.",
    }


def profile(pair: WorldPair, kind: str, directory: Path) -> None:
    pair.reset()
    pipe = Pipeline(pair, kind)
    pipe.consume(pipe.encode(pipe.capture()))
    profiler = cProfile.Profile()
    events = []
    for frame in range(40):
        pair.pose_sequence(frame, 0.01)
        profiler.enable()
        start = time.perf_counter_ns() / 1000
        packet = pipe.capture()
        a = time.perf_counter_ns() / 1000
        payload = pipe.encode(packet)
        b = time.perf_counter_ns() / 1000
        pipe.consume(payload)
        c = time.perf_counter_ns() / 1000
        profiler.disable()
        for name, t0, t1 in (("capture", start, a), ("encode", a, b), ("consume", b, c)):
            events.append(
                {"name": name, "cat": kind, "ph": "X", "ts": t0, "dur": t1 - t0, "pid": 1, "tid": 1}
            )
    with (directory / f"{kind}.profile.txt").open("w") as stream:
        pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats("cumulative").print_stats(35)
    (directory / f"{kind}.trace.json").write_text(json.dumps({"traceEvents": events}))


def physical_run(
    pair: WorldPair, kind: str, frames: int, repeat: int, hz: int = 50
) -> dict[str, Any]:
    """Common fixed-step execution, measuring exact kernels instead of different schedulers."""
    pair.reset()
    model, data = pair.pim.model, pair.pim.data
    for _ in range(250):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    pipe = Pipeline(pair, kind)
    pipe.consume(pipe.encode(pipe.capture()))
    timings: dict[str, list[float]] = defaultdict(list)
    steps_per_sample = round(1 / (hz * pair.pim.dt))
    maximum_contact = 0
    state_error = 0.0
    cpu_before = time.process_time()
    began = time.perf_counter()
    checksum = 0.0
    for i in range(frames):
        t0 = time.perf_counter_ns()
        for _ in range(steps_per_sample):
            data.ctrl[:] = 0.03 * np.sin(2 * np.pi * data.time + np.arange(model.nu) * 0.1)
            if len(pair.free_q):
                # Side force above static friction, toggled; ongoing floor contacts.
                count = max(1, len(pair.free_q) // 10)
                bodies = model.jnt_bodyid[model.jnt_type == mujoco.mjtJoint.mjJNT_FREE][:count]
                data.xfrc_applied[bodies, 0] = 2.0 * np.sin(2 * np.pi * data.time)
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)  # same coherent-state contract for all four arms
            maximum_contact = max(maximum_contact, data.ncon)
        t1 = time.perf_counter_ns()
        if kind != "physics_only":
            result = pipe.consume(pipe.encode(pipe.capture()))
        t2 = time.perf_counter_ns()
        if kind != "physics_only" and i % 10 == 0:
            state_error = max(state_error, max_error(result, reference(pair)))
        timings["physics_us"].append((t1 - t0) / 1000)
        timings["distribution_us"].append((t2 - t1) / 1000)
        timings["cycle_us"].append((t2 - t0) / 1000)
        checksum += float(np.sum(data.qpos))
    wall = time.perf_counter() - began
    return {
        "kind": kind,
        "props": len(pair.free_q),
        "repeat": repeat,
        "frames": frames,
        "hz": hz,
        "sim_seconds": frames / hz,
        "wall_seconds": wall,
        "cpu_seconds": time.process_time() - cpu_before,
        "rtf": (frames / hz) / (sum(timings["cycle_us"]) / 1e6),
        "wall_rtf_including_validation": (frames / hz) / wall,
        "peak_contacts": maximum_contact,
        "state_error": state_error,
        "trajectory_checksum": checksum,
        "metrics": {k: summary(v) for k, v in timings.items()},
    }


def failure_cases(pair: WorldPair) -> list[dict[str, Any]]:
    """Real writer/codec/mirror: late join, missed settled update, old frame, reset."""
    rows = []
    for kind in ("pim_full", "compact_full", "jeff_delta"):
        pair.reset()
        publisher, reader = Pipeline(pair, kind), Pipeline(pair, kind)
        pair.pose_sequence(0, 0)
        initial = publisher.encode(publisher.capture())
        reader.consume(initial)
        # Explicitly set a real free body's new pose once, then leave it still.
        pair.pim.data.qpos[pair.free_q[0]] += 0.1
        mujoco.mj_forward(pair.pim.model, pair.pim.data)
        moved = publisher.encode(publisher.capture())
        later = publisher.encode(publisher.capture())
        expected = reference(pair)
        final = reader.consume(later)  # one dropped message, NOT stochastic network loss
        rows.append(
            {
                "kind": kind,
                "case": "missed_final_update",
                "pass": max_error(final, expected) < 1e-6,
                "error": max_error(final, expected),
            }
        )
        late = Pipeline(pair, kind)
        try:
            final = late.consume(later)
            ready = max_error(final, expected) < 1e-6
            detail = "correct current state" if ready else "incomplete/wrong state"
        except (AttributeError, ValueError, TypeError, KeyError) as exc:
            ready, detail = False, type(exc).__name__ + ": " + str(exc)
        rows.append({"kind": kind, "case": "late_join", "pass": ready, "detail": detail})
        reader.consume(moved)
        reader.consume(initial)
        # Dense payloads alone don't reject old packets: explicitly report that limitation too.
        if kind == "jeff_delta":
            rows.append(
                {
                    "kind": kind,
                    "case": "older_frame_rejected",
                    "pass": reader.mirror.frame > 1,
                    "detail": f"mirror frame after older message: {reader.mirror.frame}",
                }
            )
    return rows


def run_suite(args: argparse.Namespace) -> None:
    directory = OUTPUT / args.name
    directory.mkdir(parents=True, exist_ok=False)
    record: dict[str, Any] = {
        "provenance": provenance(),
        "protocol": vars(args),
        "replay": [],
        "physics": [],
        "memory": [],
        "failures": [],
    }
    path = directory / "results.json"

    def save() -> None:
        path.write_text(json.dumps(record, indent=2, allow_nan=False))

    save()
    rng = random.Random(20261102)
    for props in args.props:
        t0 = time.perf_counter()
        pair = WorldPair.build(directory / f"scene-{props}", props, args.robots, args.joints)
        print(
            json.dumps(
                {
                    "stage": "built",
                    "props": props,
                    "bodies": pair.pim.model.nbody,
                    "seconds": time.perf_counter() - t0,
                }
            ),
            flush=True,
        )
        for fraction in args.fractions:
            frames = max(16, min(args.frames, 12000 // max(1, props)))
            for repeat in range(args.repeats):
                kinds = list(STRATEGIES)
                rng.shuffle(kinds)
                for kind in kinds:
                    result = one_replay(pair, kind, fraction, frames, repeat)
                    record["replay"].append(result)
                    print(
                        json.dumps(
                            {
                                "stage": "replay",
                                "props": props,
                                "fraction": fraction,
                                "repeat": repeat,
                                "kind": kind,
                                "us": round(result["metrics"]["total_us"]["median"], 1),
                                "bytes": result["metrics"]["bytes"]["median"],
                                "error": result["error"],
                            }
                        ),
                        flush=True,
                    )
                save()
        if args.physics:
            for repeat in range(args.repeats):
                kinds = [*STRATEGIES, "physics_only"]
                rng.shuffle(kinds)
                for kind in kinds:
                    result = physical_run(pair, kind, args.physics_frames, repeat)
                    record["physics"].append(result)
                    print(
                        json.dumps(
                            {
                                "stage": "physics",
                                "props": props,
                                "kind": kind,
                                "repeat": repeat,
                                "rtf": result["rtf"],
                                "contacts": result["peak_contacts"],
                            }
                        ),
                        flush=True,
                    )
                save()
        if props == args.props[-1]:
            for kind in STRATEGIES:
                record["memory"].append(allocations(pair, kind, 0.01))
                profile(pair, kind, directory)
        if props and not record["failures"]:
            record["failures"] = failure_cases(pair)
        record["power_end"] = command("pmset", "-g", "batt")
        save()
    print(str(path), flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default=datetime.now().strftime("run-%Y%m%d-%H%M%S"))
    parser.add_argument("--props", nargs="+", type=int, default=[0, 32, 128, 512, 2048])
    parser.add_argument("--fractions", nargs="+", type=float, default=[0, 0.01, 0.1, 1])
    parser.add_argument("--robots", type=int, default=1)
    parser.add_argument("--joints", type=int, default=29)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--physics", action="store_true")
    parser.add_argument("--physics-frames", type=int, default=60)
    run_suite(parser.parse_args())


if __name__ == "__main__":
    main()
