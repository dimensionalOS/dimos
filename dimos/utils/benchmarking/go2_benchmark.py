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

"""Tool 2 of the Go2 tuning deliverable: the operating-point benchmark.

Consumes the config artifact from ``go2_characterization``, runs the
HARDCODED baseline P-controller with the derived feedforward + velocity
profile across a speed ladder on a fixed real-space-constrained path set,
scores each (path, speed), and writes back the operating-point map +
tolerance->max-safe-speed inversion (artifact section 5).

    uv run python -m dimos.utils.benchmarking.go2_benchmark \\
        --config reports/go2_config_sim_mujoco_<date>_<sha>.json

The sim harness (the baseline controller driven through a real
``ControlCoordinator`` + the FOPDT ``Go2SimTwistBaseAdapter``) is inlined
below — it is small, baseline-only, and used by nothing else.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
from typing import TYPE_CHECKING, Protocol

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.task import ControlMode, JointCommandOutput
from dimos.control.tasks.baseline_path_follower_task import (
    BaselinePathFollowerTask,
    BaselinePathFollowerTaskConfig,
)
from dimos.control.tasks.feedforward_gain_compensator import FeedforwardGainConfig
from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.utils.benchmarking.go2_tuning import (
    Go2TuningConfig,
    OperatingPoint,
    OperatingPointMap,
    invert_tolerance,
)
from dimos.utils.benchmarking.paths import circle, single_corner, square, straight_line
from dimos.utils.benchmarking.scoring import ExecutedTrajectory, TrajectoryTick, score_run
from dimos.utils.benchmarking.velocity_profile import PathSpeedCap, VelocityProfileConfig

if TYPE_CHECKING:
    from dimos.hardware.drive_trains.go2_sim.adapter import Go2SimTwistBaseAdapter

# Go2 hardware control rate.
GO2_TICK_RATE_HZ = 10.0
_base_joints = make_twist_base_joints("base")
_ARRIVED_STATES = frozenset({"arrived", "completed"})
_FAILED_STATES = frozenset({"aborted"})


# --- inlined baseline sim harness (was runner.py + sim_blueprint.py) -----


class _PathFollowerLike(Protocol):
    def start_path(self, path: NavPath, current_odom: PoseStamped) -> bool: ...
    def update_odom(self, odom: PoseStamped) -> None: ...
    def compute(self, state) -> object: ...
    def get_state(self) -> str: ...


class _VelocityProfileProxyTask:
    """Curvature velocity-profile cap (the DERIVE consumer seam). Caps
    commanded ``|vx|`` to the profile at the robot's path index, scaling
    ``wz`` to preserve geometry; pure pass-through otherwise. No
    control-law change."""

    def __init__(self, inner: _PathFollowerLike, cap: PathSpeedCap) -> None:
        self._inner = inner
        self._cap = cap
        self._xy = (0.0, 0.0)

    @property
    def name(self) -> str:
        return self._inner.name

    def claim(self):
        return self._inner.claim()

    def is_active(self) -> bool:
        return self._inner.is_active()

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        self._inner.on_preempted(by_task, joints)

    def on_buttons(self, *a, **k):
        return self._inner.on_buttons(*a, **k)

    def on_cartesian_command(self, *a, **k):
        return self._inner.on_cartesian_command(*a, **k)

    def set_target_by_name(self, *a, **k):
        return self._inner.set_target_by_name(*a, **k)

    def set_velocities_by_name(self, *a, **k):
        return self._inner.set_velocities_by_name(*a, **k)

    def get_state(self) -> str:
        return self._inner.get_state()

    def update_odom(self, odom: PoseStamped) -> None:
        self._xy = (float(odom.position.x), float(odom.position.y))
        self._inner.update_odom(odom)

    def start_path(self, path: NavPath, current_odom: PoseStamped) -> bool:
        self._cap.for_path(path)
        self._xy = (float(current_odom.position.x), float(current_odom.position.y))
        return self._inner.start_path(path, current_odom)

    def compute(self, state):
        out = self._inner.compute(state)
        if out is None or out.mode != ControlMode.VELOCITY or out.velocities is None:
            return out
        vx, vy, wz = ([*out.velocities, 0.0, 0.0, 0.0])[:3]
        cx, cy, cz = self._cap.cap(self._xy[0], self._xy[1], vx, vy, wz)
        return JointCommandOutput(
            joint_names=out.joint_names,
            velocities=[cx, cy, cz],
            mode=ControlMode.VELOCITY,
        )


def _go2_sim_base() -> HardwareComponent:
    return HardwareComponent(
        hardware_id="base",
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints("base"),
        adapter_type="go2_sim_twist_base",
    )


def _odom_to_pose(odom: list[float]) -> PoseStamped:
    return PoseStamped(
        position=Vector3(odom[0], odom[1], 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, odom[2])),
    )


def _vels_to_twist(v: list[float]) -> Twist:
    return Twist(linear=Vector3(v[0], v[1], 0.0), angular=Vector3(0.0, 0.0, v[2]))


def _run_baseline_sim(
    path: NavPath,
    speed: float,
    k_angular: float,
    ff_config: FeedforwardGainConfig,
    profile_config: VelocityProfileConfig,
    timeout_s: float,
) -> ExecutedTrajectory:
    """Production baseline P-controller in sim against the FOPDT
    ``Go2SimTwistBaseAdapter``, with the derived FF + curvature profile."""
    coord = ControlCoordinator(
        tick_rate=GO2_TICK_RATE_HZ,
        hardware=[_go2_sim_base()],
        tasks=[
            TaskConfig(name="vel_base", type="velocity", joint_names=_base_joints, priority=10),
        ],
    )

    def _make() -> _PathFollowerLike:
        base = BaselinePathFollowerTask(
            name="baseline_follower",
            config=BaselinePathFollowerTaskConfig(
                speed=speed, k_angular=k_angular, ff_config=ff_config
            ),
            global_config=global_config,
        )
        return _VelocityProfileProxyTask(base, PathSpeedCap(profile_config))

    coord.start()
    try:
        adapter: Go2SimTwistBaseAdapter = coord._hardware["base"].adapter
        start = path.poses[0]
        adapter.set_initial_pose(start.position.x, start.position.y, start.orientation.euler[2])
        adapter.connect()

        task = _make()
        coord.add_task(task)
        task.start_path(path, _odom_to_pose(adapter.read_odometry()))

        ticks: list[TrajectoryTick] = []
        period = 1.0 / GO2_TICK_RATE_HZ
        t0 = time.perf_counter()
        next_sample = t0
        arrived = False
        while True:
            now = time.perf_counter()
            t_rel = now - t0
            if t_rel > timeout_s:
                break
            pose = _odom_to_pose(adapter.read_odometry())
            task.update_odom(pose)
            ticks.append(
                TrajectoryTick(
                    t=t_rel,
                    pose=pose,
                    cmd_twist=_vels_to_twist(adapter._cmd),
                    actual_twist=_vels_to_twist(adapter.read_velocities()),
                )
            )
            s = task.get_state()
            if s in _ARRIVED_STATES:
                arrived = True
                break
            if s in _FAILED_STATES:
                break
            next_sample += period
            sleep_for = next_sample - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
        return ExecutedTrajectory(ticks=ticks, arrived=arrived)
    finally:
        coord.stop()


# --- benchmark ----------------------------------------------------------


def _path_set() -> dict:
    """Real-space-constrained fixed path set (locked — do not widen)."""
    return {
        "straight_line": straight_line(),
        "single_corner": single_corner(leg_length=2.0, angle_deg=90.0),
        "square": square(side=2.0),
        "circle": circle(radius=1.0),
    }


REPORTS_DIR = Path(__file__).parent / "reports"


def _run_ladder(
    cfg: Go2TuningConfig, speeds: list[float], timeout_s: float
) -> list[OperatingPoint]:
    ff = cfg.feedforward.to_runtime()
    k_angular = float(cfg.recommended_controller.params.get("k_angular", 0.5))
    points: list[OperatingPoint] = []
    for name, path in _path_set().items():
        for speed in speeds:
            profile = cfg.velocity_profile.to_runtime(max_linear_speed=speed)
            traj = _run_baseline_sim(path, speed, k_angular, ff, profile, timeout_s)
            s = score_run(path, traj)
            points.append(
                OperatingPoint(
                    path=name,
                    speed=speed,
                    cte_max=s.cte_max,
                    cte_rms=s.cte_rms,
                    arrived=s.arrived,
                )
            )
            print(
                f"  {name:14} v={speed:.2f}  cte_max={s.cte_max * 100:6.1f}cm  "
                f"cte_rms={s.cte_rms * 100:6.1f}cm  arrived={s.arrived}"
            )
    return points


def _plot(points: list[OperatingPoint], out: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for name in sorted({p.path for p in points}):
        xs = [p.speed for p in points if p.path == name]
        ys = [p.cte_max * 100 for p in points if p.path == name]
        ax.plot(xs, ys, marker="o", label=name)
    ax.set_xlabel("commanded speed (m/s)")
    ax.set_ylabel("cte_max (cm)")
    ax.set_title("Go2 baseline tracking: cross-track error vs speed")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description="Go2 operating-point benchmark")
    ap.add_argument("--config", required=True, help="config artifact from go2_characterization")
    ap.add_argument("--speeds", default="0.3,0.5,0.7,0.9,1.0")
    ap.add_argument("--tolerances", default="5,10,15", help="cm")
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    config_path = Path(args.config).expanduser()
    cfg = Go2TuningConfig.from_json(config_path)  # asserts schema_version
    speeds = [float(s) for s in args.speeds.split(",")]
    tolerances = [float(t) for t in args.tolerances.split(",")]

    print(
        f"Speed ladder {speeds} over {len(_path_set())} paths "
        f"(baseline k_angular={cfg.recommended_controller.params.get('k_angular')}):"
    )
    points = _run_ladder(cfg, speeds, args.timeout)
    inversion = invert_tolerance(points, tolerances)
    cfg.operating_point_map = OperatingPointMap(
        speeds=speeds, points=points, tolerance_inversion=inversion
    )

    cfg.to_json(config_path)  # augment input artifact in place (section 5)
    sha = cfg.provenance.git_sha
    bench_path = REPORTS_DIR / f"go2_benchmark_{sha}.json"
    bench_path.parent.mkdir(parents=True, exist_ok=True)
    bench_path.write_text(json.dumps(asdict(cfg.operating_point_map), indent=2))
    plot_path = REPORTS_DIR / f"go2_benchmark_cte_vs_speed_{sha}.png"
    _plot(points, plot_path)

    print(f"\nAugmented artifact: {config_path.resolve()}")
    print(f"Benchmark json    : {bench_path.resolve()}")
    print(f"Plot              : {plot_path.resolve()}")
    print("\nOperating-point recommendation:")
    for row in inversion:
        if row.max_speed is None:
            print(
                f"  tolerance {row.tol_cm:g} cm: NO tested speed keeps every "
                f"path within tolerance — relax the tolerance or slow the fleet."
            )
        else:
            print(
                f"  For tolerance {row.tol_cm:g} cm, run at speed "
                f"{row.max_speed:.2f} m/s with this profile "
                f"(binding path: {row.binding_path})."
            )


if __name__ == "__main__":
    main()
