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

"""TarsClient: the public high-level API.

client = TarsClient()
client.connect()          # starts the simulator (real time, background thread)
client.stand()
client.move(0.2, 0.0)     # (vx m/s, wz rad/s); resend faster than cmd_timeout
state = client.get_state()
client.sit(); client.disconnect()
"""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING

from tars_sdk.assist import Assist, AssistParams
from tars_sdk.estimator import LegOdometry
from tars_sdk.gait import GaitParams, PairGait
from tars_sdk.kinematics import SlabGeometry, yaw_pitch
from tars_sdk.model.params import Params
from tars_sdk.roll import RollGait, RollParams
from tars_sdk.sim import SimBackend

if TYPE_CHECKING:
    from tars_sdk.mirror import MirrorPublisher
from tars_sdk.types import CameraFrame, JointTargets, Measurement, Odometry, TarsState


class TarsError(RuntimeError):
    pass


class TarsClient:
    def __init__(
        self,
        realtime: bool = True,
        cmd_timeout: float | None = 0.5,
        gait_params: GaitParams | None = None,
        max_wz: float = 0.3,
        assist: AssistParams | None = AssistParams(),
        scale: float | None = None,
        scene: str | Path | None = None,
        spawn: tuple[float, float, float] = (0.0, 0.0, 0.0),
        viewer: bool = False,
    ) -> None:
        """
        Args:
            realtime: run the simulator in a background thread at wall-clock speed. If False,
                nothing moves until you call `step()` (deterministic, as fast as possible).
            cmd_timeout: seconds after the last `move()` before the command decays to zero.
                None disables the watchdog.
            gait_params: gait tuning for the 1.52 m reference robot; Froude-scaled to the
                actual size (Params.scale) like every other tunable.
            max_wz: yaw-rate command limit (rad/s).
            assist: sim-only virtual wrench on the hub that keeps TARS upright and supplies
                the forward push / yaw the stock design can't produce (see tars_sdk.assist).
                None = pure physics.
            scale: robot size relative to the 1.52 m film TARS (default Params.scale = 0.5).
            scene: MJCF of an environment to drop TARS into (e.g. dimos' scene_office1.xml);
                None = empty checker floor.
            spawn: (x, y, yaw) in the scene; the odom frame has its origin here.
            viewer: open a MuJoCo viewer window (separate process, works from any thread).
        """
        self.realtime = realtime
        self.cmd_timeout = cmd_timeout
        ref = Params() if scale is None else Params(scale=scale)
        s = ref.scale
        self.scale = s
        self.max_wz = max_wz * s**-0.5
        self._scene = Path(scene) if scene is not None else None
        self._spawn = spawn
        self._viewer = viewer
        self._mirror: MirrorPublisher | None = None
        self._params = ref.resolved()
        self._geom = SlabGeometry.from_params(self._params)
        self._gait_params = (gait_params or GaitParams()).scaled(s)
        self._assist = Assist(assist.scaled(s)) if assist is not None else None
        self._backend: SimBackend | None = None
        self._gait: PairGait | None = None
        self._roll: RollGait | None = None
        self._roll_params = RollParams().scaled(s)
        self._want_roll = False
        self._odom: LegOdometry | None = None
        self._cmd = (0.0, 0.0)
        self._cmd_time = 0.0
        self._last_meas: Measurement | None = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------ lifecycle
    def connect(self) -> None:
        if self._backend is not None:
            return
        p = self._params
        self._gait = PairGait(self._geom, self._gait_params, p.total_mass, p.lower_mass)
        self._roll = RollGait(self._geom, self._roll_params, p.total_mass)
        self._odom = LegOdometry(
            self._geom,
            contact_threshold=0.05 * p.total_mass * 9.81,
            vel_filter_hz=5.0 * self.scale**-0.5,
        )
        self._backend = SimBackend(
            self._params, self._scene, self._spawn, control_hz=self._params.control_hz
        )
        self._backend.set_controller(self._control)
        self._backend.advance(self._backend.control_dt)  # prime state
        if self._viewer:
            from tars_sdk.mirror import MirrorPublisher

            self._mirror = MirrorPublisher(self._backend, self.scale)
        if self.realtime:
            self._backend.start()

    def disconnect(self) -> None:
        if self._backend is not None:
            self._backend.stop()
        if self._mirror is not None:
            self._mirror.close()
            self._mirror = None
        self._backend = None

    @property
    def connected(self) -> bool:
        return self._backend is not None

    @property
    def sim(self) -> SimBackend:
        """The MuJoCo backend (model, data, lock) for viewers and debugging."""
        return self._require()

    def step(self, seconds: float) -> None:
        """Advance the simulation (only when realtime=False)."""
        if self.realtime:
            raise TarsError("step() is only available with realtime=False")
        self._require().advance(seconds)

    # ------------------------------------------------------------ high-level motion
    def stand(self) -> None:
        self._require_gait().request_stand()

    def sit(self) -> None:
        self._require_gait().request_sit()

    def set_mode(self, mode: str) -> None:
        """Locomotion mode: "walk" (pair gait) or "roll" (slabs become wheel spokes).

        Switching to roll waits until the robot is standing still ("ready"); switching back
        brakes, folds the spokes down, and stands up again.
        """
        if mode not in ("walk", "roll"):
            raise ValueError(f"mode must be 'walk' or 'roll', got {mode!r}")
        self._want_roll = mode == "roll"
        if self._want_roll:
            self._require_gait().request_stand()

    @property
    def locomotion(self) -> str:
        """Active locomotion mode: "walk" or "roll" (roll includes its transitions)."""
        return "roll" if self._roll is not None and self._roll.active else "walk"

    def move(self, vx: float, wz: float = 0.0) -> None:
        """Body-frame velocity command. vx in m/s (forward +), wz in rad/s (left +)."""
        max_vx = max(self._gait_params.max_vx, self._roll_params.max_speed)
        with self._lock:
            self._cmd = (max(-max_vx, min(max_vx, vx)), max(-self.max_wz, min(self.max_wz, wz)))
            self._cmd_time = time.monotonic() if self.realtime else self._sim_time()

    def stop(self) -> None:
        self.move(0.0, 0.0)

    def wait_for_mode(self, mode: str, timeout: float = 10.0) -> bool:
        """Block until the gait reaches `mode` (e.g. "ready", "sit")."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._phase() == mode:
                return True
            if self.realtime:
                time.sleep(0.02)
            else:
                self.step(0.02)
        return False

    @property
    def limits(self) -> tuple[float, float]:
        """(max |vx| m/s, max |wz| rad/s) for the requested locomotion mode."""
        vmax = self._roll_params.max_speed if self._want_roll else self._gait_params.max_vx
        return vmax, self.max_wz

    # ------------------------------------------------------------ state
    def get_state(self) -> TarsState:
        backend, gait, odom = self._require(), self._require_gait(), self._odom
        assert odom is not None
        with backend.lock:
            meas = self._last_meas or backend.measure()
            gt = backend.ground_truth()
            return TarsState(
                time=meas.time,
                mode=self._phase(),
                command=gait.cmd,
                odom=Odometry(**vars(odom.odom)),
                odom_gt=gt,
                measurement=meas,
            )

    def get_odometry(self, ground_truth: bool = False) -> Odometry:
        s = self.get_state()
        return s.odom_gt if ground_truth and s.odom_gt is not None else s.odom

    def get_camera(self, width: int = 640, height: int = 480, depth: bool = True) -> CameraFrame:
        """RGB (+ depth in meters) from the front camera on slab 2."""
        return self._require().render_camera(width, height, depth)

    # ------------------------------------------------------------ internals
    def _require(self) -> SimBackend:
        if self._backend is None:
            raise TarsError("not connected; call connect() first")
        return self._backend

    def _require_gait(self) -> PairGait:
        if self._gait is None:
            raise TarsError("not connected; call connect() first")
        return self._gait

    def _phase(self) -> str:
        if self._roll is not None and self._roll.active:
            return self._roll.phase
        return self._require_gait().phase

    def _sim_time(self) -> float:
        return float(self._backend.data.time) if self._backend is not None else 0.0

    def _control(self, dt: float, meas: Measurement) -> JointTargets:
        gait, odom = self._gait, self._odom
        assert gait is not None and odom is not None
        with self._lock:
            vx, wz = self._cmd
            if self.cmd_timeout is not None:
                now = time.monotonic() if self.realtime else meas.time
                if now - self._cmd_time > self.cmd_timeout:
                    vx, wz = 0.0, 0.0
        roll, backend = self._roll, self._backend
        assert roll is not None and backend is not None
        yaw, pitch = yaw_pitch(meas.imu_quat)
        v, w = backend.hub_velocity_world()
        v_fwd = float(v[0] * math.cos(yaw) + v[1] * math.sin(yaw))

        if self._want_roll and not roll.active and gait.phase == "ready":
            roll.start(meas)
        if roll.active:
            if not self._want_roll:
                roll.request_exit()
            vmax = self._roll_params.max_speed
            roll.set_command(max(-vmax, min(vmax, vx)), wz)
            targets = roll.update(dt, meas, pitch, v_fwd)
            if not roll.active:  # folded back up: stand again with the walking gait
                gait.reset_seated()
                gait.request_stand()
            planned, lag, active, rolling = roll.speed_command, 0.0, True, True
        else:
            vmax = self._gait_params.max_vx
            gait.set_command(max(-vmax, min(vmax, vx)), wz)
            targets = gait.update(dt, meas, yaw, pitch)
            planned, lag = gait.planned_hub_speed, gait.hub_lag
            active, rolling = gait.phase in ("ready", "swing", "shift", "twist"), False
        odom.update(meas, dt)
        if self._assist is not None:
            roll_force = 0.0
            if roll.phase in ("roll", "brake", "gather"):
                roll_force = self._assist.roll_push(
                    roll.foot_offset,
                    backend.ground_truth().z,
                    roll.weight,
                    v_fwd,
                    planned,
                    hold=roll.phase != "roll",
                )
            wrench = self._assist.wrench(
                dt, meas.imu_quat, v, w, wz, planned, lag, active, rolling, roll_force
            )
            backend.set_hub_wrench(wrench)
        self._last_meas = meas
        return targets
