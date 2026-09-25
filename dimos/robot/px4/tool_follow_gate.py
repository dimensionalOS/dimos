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

"""Follow gate: ``px4-sitl-follow`` from detections to a flown FOLLOW. PASS or FAIL with numbers.

    python dimos/robot/px4/tool_follow_gate.py --offline  # no PX4: the gate is the vehicle
    python dimos/robot/px4/tool_follow_gate.py            # against `make px4_sitl gz_x500`
    python dimos/robot/px4/tool_follow_gate.py --fly      # + YAW_TRACK and FOLLOW flown

Asserts: Detection2DModule reports the synthetic square as track 1 at 5 Hz or better; after
``select_track(1)`` the target is valid at the range the fake gimbal's pitch implies, the
solved azimuth is within 2 deg of heading plus gimbal yaw, and ``target_los`` carries the
gimbal yaw; the gimbal module sent aim requests and the fake A8 stayed on the target. With
``--fly``: YAW_TRACK yaws the vehicle toward the gimbal at the guidance law's rate while it
holds position; FOLLOW, with the gimbal centred, flies along the target bearing under the
speed cap; a cleared selection holds, then falls back to HOVER; land. The synthetic target
is fixed in the image, so the range never closes: the leg proves direction, the cap and the
loss ladder, not convergence.

Run it alone, or on a private zenoh bus with ``ZENOH_SCOUT_ADDR=224.0.0.231:7461``: a second
dimos process on the same bus (another gate, ``dimos run``) shares its topics and RPC names.
"""

from __future__ import annotations

import argparse
import math
import os
import threading
import time
from typing import Any

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.gimbal.siyi.gimbal import SiyiA8Gimbal
from dimos.hardware.sensors.camera.rtsp.camera import RtspCamera
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.module2D import Detection2DModule
from dimos.perception.geolocation.estimators import EstimatorConfig
from dimos.robot.px4.blueprints_follow import px4_sitl_follow
from dimos.robot.px4.connection import Px4DroneConnection
from dimos.robot.px4.follow import FollowConfig, YawTrackConfig, yaw_track_rate
from dimos.robot.px4.perception_bridge import PerceptionBridge
from dimos.robot.px4.sitl import FakeA8
from dimos.robot.px4.tool_sitl_gate import fake_gcs, place, wait_state
from dimos.utils.transform_utils import normalize_angle
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

_MIN_DETECTION_HZ = 5.0
_FIRST_DETECTION_WITHIN = 10
_AZIMUTH_TOL_DEG = 2.0
_GIMBAL_TOL_DEG = 3.0
_RANGE_TOL_M = 1.0
# The blueprint's fixed 10 m AGL, less the height up a person the estimator aims at.
_AIM_DROP_M = 10.0 - EstimatorConfig().aim_height_m["person"]
# The blueprint is composed at import, viewer included whatever ``g.viewer`` says later; the
# rerun bridge would open a viewer window on every run.
_VIEWER_MODULES = {RerunBridgeModule, RerunWebSocketServer}
# --offline: the modules that run without PX4, and the vehicle the gate stands in for.
_OFFLINE_MODULES = {RtspCamera, SiyiA8Gimbal, FakeA8, Detection2DModule, PerceptionBridge}
_OFFLINE_HEADING_DEG = 10.0
_OFFLINE_ALT_M = 10.0
# --fly
_TAKEOFF_ALT_M = 3.0
_LEG_S = 4.0
_HOLD_DRIFT_M = 1.0
_FOLLOW_MIN_MOVE_M = 3.0
_FOLLOW_BEARING_TOL_DEG = 10.0


def _wrap180(deg: float) -> float:
    return math.degrees(normalize_angle(math.radians(deg)))


class _Taps:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.arrays = 0
        self.first_detection = -1
        self.ids: set[str] = set()
        self.los_yaw_flu = math.nan
        self.heading_deg = math.nan

    def on_detections(self, msg: Detection2DArray) -> None:
        with self.lock:
            self.arrays += 1
            if msg.detections_length > 0 and self.first_detection < 0:
                self.first_detection = self.arrays
            self.ids.update(d.id for d in msg.detections[: msg.detections_length])

    def on_los(self, msg: PoseStamped) -> None:
        with self.lock:
            self.los_yaw_flu = msg.yaw

    def on_odom(self, msg: Odometry) -> None:
        with self.lock:
            self.heading_deg = -math.degrees(msg.orientation.to_euler().z)  # FLU yaw -> NED

    def heading(self) -> float:
        with self.lock:
            return self.heading_deg


def _stand_in_vehicle(coordinator: ModuleCoordinator, taps: _Taps, stop: threading.Event) -> None:
    """--offline: the odometry of a vehicle hovering on a fixed heading, in place of PX4."""
    odometry = coordinator.transports[("odometry", Odometry)]
    q = Quaternion.from_euler(Vector3(0.0, 0.0, -math.radians(_OFFLINE_HEADING_DEG)))
    while not stop.wait(1 / 30):
        msg = Odometry(
            ts=time.time(),
            frame_id="odom",
            child_frame_id="base_link",
            pose=Pose(Vector3(0.0, 0.0, _OFFLINE_ALT_M), q),
            twist=Twist(),
        )
        odometry.publish(msg)
        taps.on_odom(msg)


def _check_line_of_sight(bridge: Any, a8: Any, taps: _Taps) -> bool:
    """The selected target against where the fake A8 points. Prints what it saw."""
    status, fake, heading = bridge.status(), a8.attitude(), taps.heading()
    with taps.lock:
        los_yaw_flu = taps.los_yaw_flu
    los, target = status["los"], status["target"]
    print("los:", los)
    print("target:", target)
    if not (los and los["valid"] and target and target["valid"]) or math.isnan(los_yaw_flu):
        print("no valid line of sight, target or target_los")
        return False
    az_err = abs(_wrap180(los["azimuth_deg"] - (heading + fake["yaw"])))
    yaw_err = abs(_wrap180(-math.degrees(los_yaw_flu) - fake["yaw"]))
    expected_range = _AIM_DROP_M / math.tan(math.radians(-fake["pitch"]))
    range_err = abs(target["range_m"] - expected_range)
    print(
        f"azimuth {los['azimuth_deg']:.2f} vs heading {heading:.2f} + gimbal {fake['yaw']:.2f}: "
        f"err {az_err:.2f} deg; target_los yaw err {yaw_err:.2f} deg; "
        f"range {target['range_m']:.2f} m vs {expected_range:.2f} m"
    )
    return az_err <= _AZIMUTH_TOL_DEG and yaw_err <= _AZIMUTH_TOL_DEG and range_err <= _RANGE_TOL_M


def _centre_gimbal(bridge: Any, gimbal: Any, a8: Any) -> bool:
    """Point the fake A8 straight ahead, 20 deg down, then select the target again.

    The target is fixed in the image, so its bearing is heading plus gimbal yaw; with the
    gimbal off-centre FOLLOW would chase a bearing that turns with the vehicle. The
    selection is dropped first because the aim path otherwise re-requests the present yaw,
    and for long enough that the target filter starts afresh instead of seeing a 30 deg jump.
    """
    bridge.clear_selection()
    time.sleep(0.5)
    sent = gimbal.aim(-20.0, 0.0)
    time.sleep(EstimatorConfig().drop_after_s + 1.0)
    bridge.select_track(1)
    time.sleep(2.0)
    fake = a8.attitude()
    print(f"gimbal centred: aim sent {sent}, fake A8 {fake}")
    return bool(sent) and abs(fake["yaw"]) < _GIMBAL_TOL_DEG


def _yaw_track_leg(drone: Any, a8: Any, taps: _Taps) -> bool:
    """The gimbal sits 30 deg right of the nose, so the vehicle yaws clockwise in place."""
    rate = yaw_track_rate(a8.attitude()["yaw"], True, YawTrackConfig())
    start, heading = place(drone), taps.heading()
    print("YAW_TRACK:", drone.set_guidance_mode("YAW_TRACK"))
    time.sleep(_LEG_S)
    status = drone.status()
    turned = _wrap180(taps.heading() - heading)
    drift = math.hypot(*(a - b for a, b in zip(place(drone)[:2], start[:2], strict=True)))
    print(
        f"state={status['state']} target={status['target']} turned {turned:.1f} deg clockwise "
        f"in {_LEG_S:.0f}s (law {rate:.1f} deg/s), drift {drift:.2f} m"
    )
    ok: bool = status["state"] == "YAW_TRACK" and rate > 0.0
    ok &= 0.5 * rate * _LEG_S <= turned <= 1.5 * rate * _LEG_S
    ok &= drift <= _HOLD_DRIFT_M
    print("HOVER:", drone.set_guidance_mode("HOVER"))
    return ok and wait_state(drone, {"HOVER"}, timeout_s=5.0) == "HOVER"


def _follow_leg(drone: Any, bridge: Any, gimbal: Any, a8: Any) -> bool:
    """Gimbal centred, so the target bearing is the heading and stays put; then the loss."""
    ok = _centre_gimbal(bridge, gimbal, a8)
    target = bridge.status()["target"]
    if not (target and target["valid"]):
        print("no valid target to follow:", target)
        return False
    bearing = target["bearing_deg"]
    start = place(drone)
    print("FOLLOW:", drone.set_guidance_mode("FOLLOW"))
    time.sleep(_LEG_S)
    status, here = drone.status(), place(drone)
    sp = status["setpoint"] or {}
    north, east = here[0] - start[0], here[1] - start[1]
    moved = math.hypot(north, east)
    off = abs(_wrap180(math.degrees(math.atan2(east, north)) - bearing))
    speed = math.hypot(sp.get("vn", 0.0), sp.get("ve", 0.0))
    print(
        f"state={status['state']} reason={status['reason']!r} moved {moved:.2f} m, "
        f"{off:.1f} deg off the target bearing {bearing:.1f}; commanded {speed:.2f} m/s"
    )
    ok &= status["state"] == "FOLLOW" and status["target_fresh"] and sp.get("kind") == "vel"
    ok &= moved >= _FOLLOW_MIN_MOVE_M and off <= _FOLLOW_BEARING_TOL_DEG
    ok &= speed <= FollowConfig().v_max_mps + 1e-6

    # The selection goes: the filter coasts, the supervisor holds, then gives up to HOVER.
    print("clear_selection:", bridge.clear_selection())
    time.sleep(EstimatorConfig().max_meas_age_s + 1.0)
    status = drone.status()
    print(f"state={status['state']} reason={status['reason']!r}")
    ok &= status["state"] == "FOLLOW" and "holding" in status["reason"]
    state = wait_state(drone, {"HOVER", "IDLE", "ABORT"}, FollowConfig().loss_hover_s + 2.0)
    status = drone.status()
    print(f"state={state} reason={status['reason']!r} guidance_mode={status['guidance_mode']}")
    return ok and state == "HOVER" and "lost" in status["reason"]


def _fly(drone: Any, bridge: Any, gimbal: Any, a8: Any, taps: _Taps) -> bool:
    """Takeoff, YAW_TRACK, FOLLOW and its loss ladder, land. Returns whether all of it held."""
    print("sitl_enable:", drone.sitl_enable(True))
    print(f"takeoff({_TAKEOFF_ALT_M}):", drone.takeoff(_TAKEOFF_ALT_M))
    state = wait_state(drone, {"HOVER", "IDLE", "ABORT"}, timeout_s=60.0)
    time.sleep(4.0)
    print(f"after takeoff: state={state} place={place(drone)}")
    ok: bool = state == "HOVER"
    try:
        ok = ok and _yaw_track_leg(drone, a8, taps) and _follow_leg(drone, bridge, gimbal, a8)
    finally:
        print("land:", drone.land())
        state = wait_state(drone, {"IDLE", "ABORT"}, timeout_s=90.0)
        print(f"after land: state={state} reason={drone.status()['reason']!r}")
    return ok and state == "IDLE"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--seconds", type=float, default=6.0, help="detection listening window")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--offline", action="store_true", help="no PX4: the perception chain only")
    mode.add_argument("--fly", action="store_true", help="fly YAW_TRACK and FOLLOW as well")
    args = ap.parse_args()

    modules = [a.module for a in px4_sitl_follow.blueprints]
    if args.offline:
        unused = [m for m in modules if m not in _OFFLINE_MODULES]
    else:
        unused = [m for m in modules if m in _VIEWER_MODULES]
    blueprint = px4_sitl_follow.disabled_modules(*unused)
    stop = threading.Event()
    taps = _Taps()
    # Nothing of the shell's dimos settings but the private bus: build() resets the global
    # config to this parse, so a scout address left out here is lost to every worker.
    environ = {k: v for k, v in os.environ.items() if k == "ZENOH_SCOUT_ADDR"}
    parsed = BlueprintConfigParser(blueprint).parse(
        environ=environ, overrides={"g": {"viewer": "none"}}
    )
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    # PX4 refuses to arm without a ground station; offline the gate is the vehicle instead.
    helper = (
        threading.Thread(target=_stand_in_vehicle, args=(coordinator, taps, stop), daemon=True)
        if args.offline
        else threading.Thread(target=fake_gcs, args=(stop,), daemon=True)
    )
    ok = True
    try:
        transports = coordinator.transports
        unsubs = [
            transports[("detections", Detection2DArray)].subscribe(taps.on_detections),
            transports[("target_los", PoseStamped)].subscribe(taps.on_los),
        ]
        if not args.offline:
            unsubs.append(transports[("odometry", Odometry)].subscribe(taps.on_odom))
        helper.start()
        bridge = coordinator.get_instance(PerceptionBridge)
        gimbal = coordinator.get_instance(SiyiA8Gimbal)
        a8 = coordinator.get_instance(FakeA8)

        # 1. Detections.
        time.sleep(2.0)  # the camera writes its clip and the workers settle
        with taps.lock:
            before = taps.arrays
        time.sleep(args.seconds)
        with taps.lock:
            hz = (taps.arrays - before) / args.seconds
            first, ids = taps.first_detection, set(taps.ids)
        print(f"detections: {hz:.1f} arrays/s, first box in array {first}, track ids {sorted(ids)}")
        ok &= hz >= _MIN_DETECTION_HZ and 0 < first <= _FIRST_DETECTION_WITHIN and ids == {"1"}

        # 2. Selection, line of sight, target.
        print("select:", bridge.select_track(1))
        time.sleep(2.0)
        ok &= _check_line_of_sight(bridge, a8, taps)

        # 3. Gimbal aim.
        state, fake = gimbal.state(), a8.attitude()
        print(f"aim requests: {state['aim_sent']}  fake A8: {fake}")
        ok &= state["aim_sent"] > 0
        ok &= (
            abs(fake["pitch"] + 20.0) < _GIMBAL_TOL_DEG
            and abs(fake["yaw"] - 30.0) < _GIMBAL_TOL_DEG
        )

        # 4. The flight.
        if args.fly:
            ok &= _fly(coordinator.get_instance(Px4DroneConnection), bridge, gimbal, a8, taps)
        elif args.offline:
            ok &= _centre_gimbal(bridge, gimbal, a8) and _check_line_of_sight(bridge, a8, taps)
        for u in unsubs:
            u()
    finally:
        if args.offline:  # the stand-in publishes on the coordinator's transports: it goes first
            stop.set()
            if helper.is_alive():
                helper.join(timeout=2.0)
        coordinator.stop()
        stop.set()
        if helper.is_alive():
            helper.join(timeout=2.0)
    print("GATE", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
