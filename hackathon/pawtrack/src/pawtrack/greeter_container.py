"""DimOS glue for the greeter loop: wander -> identify -> select -> navigate
-> greet.

This container orchestrates the loop but keeps every step a separable seam, so a
more capable module can later replace any one of them:

- wander -> DimOS ``PatrollingModule`` via the injected ``PatrollingModuleSpec``
  (``start_patrol`` / ``stop_patrol`` / ``is_patrolling``).
- identify -> :func:`pawtrack.identify.detect_all` (multi-box VLM).
- select -> :func:`pawtrack.visited_registry.select_target` over a
  :class:`pawtrack.visited_registry.VisitedRegistry` (random, not greeted in the
  last ``revisit_forget_s`` seconds).
- navigate (approach) -> :func:`pawtrack.approach_geometry.approach_velocity`
  over the geometric ground distance from the raycast (the seam a real planner
  would replace).
- phase decisions -> the pure :class:`pawtrack.greeter_state.GreeterMachine`.
- greet -> halt, then wave once with ``Hello`` (1016) whichever way the person
  faces; afterward recover with ``RecoveryStand`` (1006) + ``BalanceStand``
  (1002) so it can walk again -- via the injected GO2 connection (a no-op
  off-robot). There is no orbit-to-front step: any seated person reached at a
  safe standoff gets a wave.

The per-tick logic lives in :meth:`_tick`, callable synchronously (the loop
thread just calls it on a clock), so the orchestration is unit-testable with
fakes for the models, the patrol spec, and the connection -- no GPU or robot.
"""

from __future__ import annotations

import json
import math
import random
import threading
import time
from typing import Any

import cv2
import numpy as np
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.models.segmentation.edge_tam import EdgeTAMProcessor
from dimos.models.vl.base import VlModel
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.patrolling.patrolling_module_spec import (
    PatrollingModuleSpec,
)
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.robot.unitree.dimsim_connection import DimSimConnection
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.connection_spec import GO2ConnectionSpec
from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.utils.logging_config import setup_logger
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

from pawtrack.approach_geometry import (
    approach_velocity,
    centering_yaw,
    drive_to_position,
)
from pawtrack.greeter_state import (
    GreeterMachine,
    GreeterObservation,
    GreeterParams,
    GreeterSnapshot,
)
from pawtrack.ground_raycast import CameraIntrinsics, pixel_to_ground_point
from pawtrack.identify import detect_all, detect_facing
from pawtrack.occupancy import is_near_obstacle
from pawtrack.qwen_china import QwenChinaVlModel
from pawtrack.track_state import clamp_bbox, ground_contact_pixel, valid_bbox
from pawtrack.visited_registry import Candidate, VisitedRegistry, select_target

logger = setup_logger()

_HELLO_API_ID = SPORT_CMD["Hello"]
_BALANCE_STAND_API_ID = SPORT_CMD["BalanceStand"]
_RECOVERY_STAND_API_ID = SPORT_CMD["RecoveryStand"]
_FINGER_HEART_API_ID = SPORT_CMD["FingerHeart"]
# api_id -> name, for readable sport-command tracing in the logs.
_SPORT_NAMES = {api_id: name for name, api_id in SPORT_CMD.items()}


class Config(ModuleConfig):
    """Configuration for the greeter loop."""

    description: str = "a person sitting on a chair"
    loop_hz: float = 12.0
    scan_interval_s: float = 1.5  # how often to run the VLM while wandering
    # When wander can see only already-greeted people, rotate in place to look
    # for someone new: the patrol is usually goal-less and blocked by the person
    # right ahead, so the dog would otherwise sit frozen facing a handled
    # subject. Bounded so it hands back to the patrol instead of spinning forever
    # (e.g. once everyone in view has been greeted).
    wander_rescan_yaw_radps: float = 0.5
    wander_rescan_max_s: float = 8.0
    # Free-explore fallback. The coverage patrol goes silent once it has no goal
    # (e.g. coverage exhausted), and is_patrolling() cannot tell us that, so the
    # dog would sit frozen. Detect the wedge from odom -- no position change
    # beyond stall_move_threshold_m for stall_timeout_s -- then take over and
    # roam into new space (brief in-place reorient, then forward) until the dog
    # has escaped roam_escape_distance_m from the stuck spot (or roam_max_s
    # elapses), then hand back so the patrol can replan from the new location.
    # Needs odom; with none it falls back to the rotate-only rescan and never
    # drives blind. roam_forward_mps stays above the sim RL translation deadband
    # (~0.2-0.3 m/s).
    enable_roam_when_wedged: bool = True
    stall_move_threshold_m: float = 0.3
    stall_timeout_s: float = 4.0
    roam_forward_mps: float = 0.35
    roam_yaw_radps: float = 0.6
    roam_turn_s: float = 1.5
    roam_escape_distance_m: float = 1.0
    roam_max_s: float = 6.0
    # Stop the loop (failing safe) after this many consecutive tick exceptions.
    max_consecutive_errors: int = 5
    # Phase thresholds (fed to the pure GreeterMachine).
    # 2.0 m standoff: the Go2's onboard obstacle avoidance halts the dog ~2 m
    # from a person, so targeting 1.5 m left it stuck just outside greeting range
    # (distance plateaued ~2.0-2.5 m and timed out). Hold + greet from ~2 m, which
    # it can actually reach.
    standoff_m: float = 2.0
    standoff_tolerance_m: float = 0.3
    # Hard floor on the greeting distance -- the dog's footprint + Hello reach.
    # Relaxed to 0.6 m: seated people (especially at desks) tend to end up close,
    # and the window would otherwise reject them. Greet window is
    # [min_safe_distance_m, standoff_m + standoff_tolerance_m] = [0.6, 2.3] m.
    min_safe_distance_m: float = 0.6
    engage_timeout_s: float = 10.0  # bail to patrol if an engagement stalls
    # On track loss, dead-reckon to the subject's last-known floor position
    # (odom) rather than abandoning. Only drive forward when the heading error to
    # it is under this; on arrival, a candidate within redetect_radius_m of the
    # last-known spot counts as re-acquiring the same subject (else: it moved /
    # was a bad fix -> give up).
    deadreckon_heading_tol_rad: float = 0.5
    redetect_radius_m: float = 1.0
    greet_duration_s: float = 4.0
    cooldown_duration_s: float = 6.0
    # After the wave, if the subject faces the camera, reward them with a
    # FingerHeart; this is how long to let that gesture play before recovering.
    heart_duration_s: float = 3.0
    # Settle time between RecoveryStand and BalanceStand in a one-shot wave.
    recover_settle_s: float = 1.5
    # A sport command (Hello / BalanceStand / RecoveryStand) goes out over the
    # WebRTC data channel, which raises "Data channel is not open" whenever the
    # channel briefly drops mid-session. Retry a bounded number of times to
    # bridge a transient blip; a persistent failure is logged and skipped rather
    # than crashing the control loop.
    sport_retry_attempts: int = 3
    sport_retry_delay_s: float = 0.3
    # Treat a candidate within this radius of a greeted position as the same
    # subject (chair footprint + tracker noise), so it is not greeted twice.
    revisit_radius_m: float = 1.0
    # How long a greeted subject is skipped before it may be greeted again. The
    # demo stays lively instead of going quiet once everyone has been greeted
    # once; set to None to greet each subject only once per run.
    revisit_forget_s: float | None = 60.0
    # Approach distance control: hold the standoff from the geometric ground
    # distance, yaw to recenter. The deadband holds the dog steady at the standoff
    # without hunting in/out; the bounded reverse backs it off if it gets too
    # close.
    approach_forward_gain: float = 0.8
    approach_turn_gain: float = 1.0
    approach_max_forward_mps: float = 0.5
    approach_max_reverse_mps: float = 0.3
    standoff_deadband_m: float = 0.2
    approach_max_yaw_radps: float = 0.8
    # Ground raycast (subject floor position + distance).
    camera_info: CameraInfo | None = None  # auto-resolved if None
    world_frame_id: str = "world"
    camera_optical_frame_id: str = "camera_optical"
    # How often (s) to log a lidar-path diagnostic line, so a fallback run shows
    # which boundary failed (no cloud / empty / no TF / no points in the box).
    lidar_log_interval_s: float = 3.0
    world_floor_z_m: float = 0.0
    tf_tolerance_s: float = 1.0
    # Reflection filter: drop a candidate whose floor position lands on (or
    # within this clearance of) a mapped wall in the navigation costmap. A person
    # reflected in a glass wall is detected by the VLM and ranged by the lidar at
    # the glass surface, so without this the dog walks over and waves at the
    # reflection; a real subject stands in free space. Fails open -- no costmap
    # means no filtering -- so a fresh, unmapped run greets exactly as before.
    reject_on_mapped_obstacle: bool = True
    obstacle_clearance_m: float = 0.3
    obstacle_cost_threshold: int = 50  # 0 free .. 100 lethal; raise to be lenient
    # Distance sanity bound: drop a candidate whose floor range from the robot
    # exceeds this. A box near the image horizon raycasts to a near-infinite floor
    # point, so without this the dog commits to phantom targets metres (even tens
    # of metres) away and walks at empty space. Set generously (30 m) so the dog
    # can still notice and head toward distant real people while the bound only
    # culls the clearly-impossible horizon artifacts.
    max_subject_distance_m: float = 30.0


class GreeterSkillContainer(Module):
    """Autonomous greeter: wander, find a seated person, face them, say hi."""

    config: Config

    color_image: In[Image]
    lidar: In[PointCloud2]  # world-frame lidar cloud for subject ranging
    odom: In[PoseStamped]  # robot world pose, for dead reckoning to a lost subject
    global_costmap: In[OccupancyGrid]  # nav costmap; rejects on-wall candidates
    cmd_vel: Out[Twist]
    debug_image: Out[Image]  # annotated frame: bbox + phase/distance/front
    greeter_phase: Out[str]  # rich JSON trace: phase + why (distinct from the
    # ``greeter_status`` skill below -- a port and a skill must not share a name)
    subject_world_pose: Out[PoseStamped]  # where it thinks the subject is (3D)
    # Injected by the blueprint: the patrol provides the wander; the GO2
    # connection issues the greeting (None off-robot -> greeting is a no-op).
    _patrolling_module_spec: PatrollingModuleSpec
    _connection: GO2ConnectionSpec | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._should_stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest_image: Image | None = None
        self._latest_pointcloud: PointCloud2 | None = None
        self._latest_odom: PoseStamped | None = None
        self._latest_costmap: OccupancyGrid | None = None
        self._last_lidar_log_s = 0.0
        # Horizontal bbox error of the last successful track, for re-centering
        # the search yaw when the tracker briefly drops the subject.
        self._last_error_x = 0.0
        self._machine = GreeterMachine(GreeterParams(
            standoff_m=self.config.standoff_m,
            standoff_tolerance_m=self.config.standoff_tolerance_m,
            min_safe_distance_m=self.config.min_safe_distance_m,
            engage_timeout_s=self.config.engage_timeout_s,
            greet_duration_s=self.config.greet_duration_s,
            cooldown_duration_s=self.config.cooldown_duration_s,
        ))
        self._registry = VisitedRegistry(
            self.config.revisit_radius_m, self.config.revisit_forget_s)
        self._rng = random.Random()
        self._vl_model: VlModel | None = None
        self._tracker: EdgeTAMProcessor | None = None
        # What to find / approach / wave at. Defaults to the configured subject;
        # start_greeting(target=...) overrides it per run (e.g. "a chair" in sim).
        self._description = self.config.description
        self._chosen: Candidate | None = None
        self._last_position: tuple[float, float] | None = None
        self._last_scan_s = 0.0
        # How many detections the last scan dropped (too far / on a wall), so
        # wander can rotate away from a wall/reflection instead of freezing.
        self._last_scan_dropped = 0
        # Rescan state: rotating in place to look past already-greeted people.
        self._rescanning = False
        self._rescan_started_s = 0.0
        # Free-explore (roam-when-wedged) state, driven by odom stall detection.
        self._roaming = False
        self._roam_anchor_xy: tuple[float, float] | None = None
        self._roam_progress_s = 0.0
        self._roam_start_xy: tuple[float, float] = (0.0, 0.0)
        self._roam_started_s = 0.0
        self._roam_turn_dir = 1.0
        self._last_snapshot: GreeterSnapshot | None = None
        self._last_trace: str | None = None  # last rich greeter_phase JSON
        camera_info = self._resolve_camera_info()
        self._camera_info = camera_info  # for the lidar pointcloud projection
        self._intrinsics = (
            CameraIntrinsics.from_k(list(camera_info.K))
            if camera_info is not None else None
        )

    def _resolve_camera_info(self) -> CameraInfo | None:
        """Pick camera intrinsics: explicit config, else sim/Go2 statics."""
        camera_info = self.config.camera_info
        if camera_info is not None:
            return camera_info
        simulation = self.config.g.simulation
        if simulation == "mujoco":
            return MujocoConnection.camera_info_static
        if simulation == "dimsim":
            return DimSimConnection.camera_info_static
        return GO2Connection.camera_info_static

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.color_image.subscribe(self._on_color_image))
        )
        # The lidar input is optional: when no producer is wired (e.g. a sim
        # without lidar), this simply never fires and _locate falls back to the
        # monocular ground raycast.
        self.register_disposable(
            Disposable(self.lidar.subscribe(self._on_pointcloud))
        )
        # Odom drives the dead-reckon-to-last-known fallback on track loss; if no
        # producer is wired it never fires and the loss falls back to an in-place
        # re-find search.
        self.register_disposable(
            Disposable(self.odom.subscribe(self._on_odom))
        )
        # The costmap feeds the reflection filter (drop candidates on a mapped
        # wall). Optional: if no mapper is wired the filter simply never fires.
        self.register_disposable(
            Disposable(self.global_costmap.subscribe(self._on_costmap))
        )
        # Trace the connection up front: if it is None, every sport gesture
        # (Hello / FingerHeart / Recovery / Balance) silently no-ops -- the first
        # thing to check when "it greets but does not wave".
        conn = self._connection
        logger.info(
            "greeter: GO2 connection = %s",
            type(conn).__name__ if conn is not None
            else "NONE -- sport gestures will no-op")

    @rpc
    def stop(self) -> None:
        self._stop_loop()
        # DimOS may tear the module down without going through stop_greeting, so
        # halt here too -- otherwise the last cmd_vel or an active patrol persists.
        self._halt()
        with self._lock:
            tracker = self._tracker
            self._tracker = None
            vl_model = self._vl_model
            self._vl_model = None
        if tracker is not None:
            tracker.stop()
        if vl_model is not None:
            vl_model.stop()
        super().stop()

    # -- Skills --

    @skill
    def start_greeting(self, target: str | None = None) -> str:
        """Start the autonomous greeter loop: wander, find the target, walk over,
        and wave hello (whichever way it faces). Call stop_greeting to halt.

        Args:
            target: What to look for, approach, and wave at, as a short visual
                description (e.g. "a person sitting on a chair", "a person", "a
                chair"). Defaults to the configured subject when omitted. Handy
                in simulation, where there may be no seated person -- point it at
                "a chair" or "a person" to exercise the loop.
        """
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return "Greeter is already running."
            # Per-run target: an explicit one overrides the configured default
            # (cleared back to the default when omitted). Set before the thread
            # starts so the loop's scan sees it.
            self._description = target or self.config.description
            # Start every run from a clean wander state, so a stop during
            # approach/greet/cooldown cannot resume mid-engagement. The
            # visited registry is intentionally kept across restarts.
            self._reset_engagement()
            self._should_stop.clear()
            self._thread = threading.Thread(
                target=self._run_loop, name="greeter", daemon=True
            )
            self._thread.start()
        return f"Greeter started. Looking for {self._description!r} to greet."

    @skill
    def stop_greeting(self) -> str:
        """Stop the greeter loop and halt the robot."""
        self._stop_loop()
        self._halt()  # zero velocity + stop the patrol (it owns motion in wander)
        self._publish_status(GreeterSnapshot(
            state="wander", message="Greeter stopped; idle."))
        return "Greeter stopped."

    @skill
    def greeter_status(self) -> str:
        """Report the greeter's current phase and why, as JSON.

        Includes the phase, the per-tick perception driving it (subject visible,
        distance), whether the patrol is running, where it thinks the subject is,
        and how many have been greeted -- enough to tell why it is or is not
        progressing.
        """
        if self._last_trace is None:
            return GreeterSnapshot(
                state="wander", message="Idle; greeter not started."
            ).to_json()
        return self._last_trace

    @skill
    def wave_hello(self) -> str:
        """Make the dog wave hello right now: stop, stand still, play the Hello
        gesture, then recover (RecoveryStand + BalanceStand) so it can walk again.

        If the autonomous greeter loop is running, this stops it first and waits
        for the tick thread to exit, so nothing restarts the patrol or publishes
        cmd_vel during the gesture -- call start_greeting to resume greeting
        afterward. The same greeting the loop performs; handy to greet on command
        or to verify the wave end-to-end. A no-op off-robot (no GO2 connection).
        """
        # Stop the loop and JOIN the tick thread before moving on, so no tick can
        # restart the patrol or drive cmd_vel while the gesture plays. Then stop
        # the patrol and zero velocity.
        self._stop_loop()
        self._halt()
        delivered = self._send_sport(_HELLO_API_ID)
        if delivered:
            time.sleep(self.config.greet_duration_s)  # let the gesture play out
        # Restore a clean stand + balance so it can walk again afterward.
        self._recover_locomotion()
        if not delivered:
            return (
                "Tried to wave, but the Hello gesture did not go out (no GO2 "
                "connection, or the data channel is not open)."
            )
        return "Waved hello."

    # -- Loop --

    def _run_loop(self) -> None:
        period = 1.0 / self.config.loop_hz
        next_time = time.monotonic()
        errors = 0
        while not self._should_stop.is_set():
            next_time += period
            try:
                self._tick(time.monotonic())
                errors = 0
            except Exception:  # noqa: BLE001 - thread-boundary guard
                # Fail safe on every tick error (halt, do not keep the last
                # motion command running), and give up after a persistent run of
                # them rather than driving on a broken detector/TF/model/stream.
                errors += 1
                logger.exception(
                    "greeter tick crashed (%d in a row)", errors)
                self._halt()
                if errors >= self.config.max_consecutive_errors:
                    self._set_error_status(
                        "Greeter stopped after repeated errors; see logs.")
                    break
            now = time.monotonic()
            sleep_for = next_time - now
            if sleep_for > 0:
                self._should_stop.wait(sleep_for)
            else:
                next_time = now

    def _tick(self, now: float) -> GreeterObservation:
        """One synchronous loop step: observe + drive, advance the machine.

        Returns the observation it fed the machine (handy for tests).
        """
        with self._lock:
            image = self._latest_image
        state = self._machine.state
        if state == "wander":
            acquired = self._wander_tick(image, now)
            obs = GreeterObservation(
                target_acquired=acquired, subject_visible=acquired)
        elif state == "approach":
            obs = self._engage_tick(image)
        else:  # greet / cooldown -- the sport gesture plays here
            # Do NOT stream cmd_vel during the gesture. The dog was already
            # halted on the greet edge (_on_enter -> _halt), and on the Go2,
            # streaming cmd_vel keeps it in joystick/balance mode, which
            # suppresses sport animations (Hello / FingerHeart) and the
            # RecoveryStand -- a likely reason a *sent* gesture does not play.
            obs = GreeterObservation()

        snapshot = self._machine.step(obs, now)
        if snapshot.entered:
            self._on_enter(snapshot.state, snapshot.reason, now)
        self._publish_status(snapshot, obs)
        return obs

    # -- Step: wander (DimOS patrol) + identify + select --

    def _wander_tick(self, image: Image | None, now: float) -> bool:
        """Roam (via the patrol), scan for a target, and break free if wedged.

        The patrol normally owns wander motion. But the patrol often has no goal
        when the dog faces something it cannot act on -- everyone in view already
        greeted, or every detection rejected by the filters (too far, or on a
        mapped wall, e.g. a glass reflection) -- and the thing ahead is an
        obstacle, so the dog freezes staring at it. In that case the greeter
        rotates in place to rescan a new direction, then (bounded) hands back to
        the patrol from the new heading.
        """
        chosen: Candidate | None = None
        candidates: list[Candidate] = []
        if (image is not None
                and now - self._last_scan_s >= self.config.scan_interval_s):
            self._last_scan_s = now
            candidates = self._scan(image)
            chosen = select_target(candidates, self._registry, self._rng, now)
            # Show each scan in the debug video: what was detected and the pick.
            scan_bbox = chosen.bbox if chosen else (
                candidates[0].bbox if candidates else None)
            self._publish_debug(
                image, scan_bbox, f"wander: {len(candidates)} cand")
            # Lock the tracker first; only stop the patrol once that succeeds, so
            # a failed init does not leave the robot idle with the patrol paused.
            if chosen is not None and self._init_tracker(chosen.bbox, image):
                self._patrolling_module_spec.stop_patrol()
                self._rescanning = False
                self._chosen = chosen
                self._last_position = chosen.position
                logger.info(
                    "greeter: engaging subject at (%.2f, %.2f) of %d "
                    "candidate(s)",
                    chosen.position[0], chosen.position[1], len(candidates))
                return True
            if chosen is not None:
                logger.warning(
                    "greeter: tracker init failed on the chosen subject")
            # Rotate to a new heading when the view holds something the dog
            # cannot act on -- people already greeted (kept candidates) OR
            # detections all rejected as too-far/on-a-wall (a wall/reflection the
            # filter dropped). Either way the patrol is usually goal-less here, so
            # rotating finds a fresh direction instead of freezing. Stop once the
            # view is genuinely empty so the patrol can explore from there.
            saw_something = bool(candidates) or self._last_scan_dropped > 0
            if saw_something and chosen is None and not self._rescanning:
                self._rescanning = True
                self._rescan_started_s = now
            elif not saw_something:
                self._rescanning = False
        # Don't spin forever (e.g. everyone in view is greeted): after a bounded
        # rotation, hand back to the patrol to retry from the new heading.
        if self._rescanning and (
                now - self._rescan_started_s >= self.config.wander_rescan_max_s):
            self._rescanning = False
        if self._rescanning:
            if self._patrolling_module_spec.is_patrolling():
                self._patrolling_module_spec.stop_patrol()
            self.cmd_vel.publish(self._rescan_twist())
            self._reset_roam(now)  # rotating is activity; don't also roam now
            return False
        # Free-explore: when odom shows the dog is wedged (patrol goal-less and
        # not moving), roam into new space, then hand back to the patrol.
        roam = self._roam_tick(now)
        if roam is not None:
            if self._patrolling_module_spec.is_patrolling():
                self._patrolling_module_spec.stop_patrol()
            self.cmd_vel.publish(roam)
        elif not self._patrolling_module_spec.is_patrolling():
            self._patrolling_module_spec.start_patrol()
        return False

    def _rescan_twist(self) -> Twist:
        """In-place yaw to scan a new direction when wedged on visited people."""
        return Twist(
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, self.config.wander_rescan_yaw_radps))

    def _reset_roam(self, now: float) -> None:
        """Clear roam state (rescan takes over, the dog escaped, or a new run)."""
        self._roaming = False
        self._roam_anchor_xy = None
        self._roam_progress_s = now

    def _roam_tick(self, now: float) -> Twist | None:
        """Roam into new space when the dog is wedged; None when it is not.

        The coverage patrol cannot tell us it is goal-less, so the wedge is
        detected from odom: no position change beyond ``stall_move_threshold_m``
        for ``stall_timeout_s``. On a wedge the dog takes over -- a brief in-place
        reorient (``roam_turn_s``) so it does not drive straight back into
        whatever blocked it, then forward at ``roam_forward_mps`` -- until it has
        escaped ``roam_escape_distance_m`` from the stuck spot (or ``roam_max_s``
        elapses), then it stops roaming so the caller hands back to the patrol.
        Returns None (no roam) when disabled, when odom is unavailable (never
        drive blind), or while the dog is already making progress.
        """
        if not self.config.enable_roam_when_wedged:
            return None
        with self._lock:
            odom = self._latest_odom
        if odom is None:
            return None
        xy = (odom.position.x, odom.position.y)
        if self._roam_anchor_xy is None:
            self._roam_anchor_xy = xy
            self._roam_progress_s = now
        if not self._roaming:
            moved = math.hypot(
                xy[0] - self._roam_anchor_xy[0], xy[1] - self._roam_anchor_xy[1])
            if moved > self.config.stall_move_threshold_m:
                self._roam_anchor_xy = xy  # progress -> reset the stall timer
                self._roam_progress_s = now
                return None
            if now - self._roam_progress_s < self.config.stall_timeout_s:
                return None
            # Wedged: start roaming from here, in a random direction.
            self._roaming = True
            self._roam_start_xy = xy
            self._roam_started_s = now
            self._roam_turn_dir = self._rng.choice((-1.0, 1.0))
        escaped = math.hypot(
            xy[0] - self._roam_start_xy[0], xy[1] - self._roam_start_xy[1])
        if (escaped > self.config.roam_escape_distance_m
                or now - self._roam_started_s > self.config.roam_max_s):
            self._reset_roam(now)
            return None
        # Reorient in place first, then drive forward (a slight same-way curve so
        # repeated wedges sweep the area rather than retrace one line).
        if now - self._roam_started_s < self.config.roam_turn_s:
            return Twist(
                linear=Vector3(0.0, 0.0, 0.0),
                angular=Vector3(
                    0.0, 0.0, self._roam_turn_dir * self.config.roam_yaw_radps))
        return Twist(
            linear=Vector3(self.config.roam_forward_mps, 0.0, 0.0),
            angular=Vector3(
                0.0, 0.0,
                self._roam_turn_dir * self.config.roam_yaw_radps * 0.3))

    def _scan(self, image: Image) -> list[Candidate]:
        """Detect every described subject and locate each on the floor.

        Candidates whose floor position lands on a mapped wall are dropped: a
        person reflected in a glass wall is detected by the VLM and ranged by
        the lidar at the glass surface, so without this filter the dog walks
        over and waves at the reflection. A real subject stands in free space.
        """
        candidates: list[Candidate] = []
        n_too_far = 0
        n_on_wall = 0
        for raw in detect_all(self._get_vl_model(), image, self._description):
            # Reject out-of-frame / degenerate boxes here, before one can be
            # selected and stop the patrol only to fail tracker init later.
            if not valid_bbox(list(raw), image.width, image.height):
                continue
            # Clamp to the frame BEFORE raycasting/storing: a box that spills off
            # the edge would otherwise raycast an off-image bottom-center to a
            # wrong floor position (and visited key). The tracker init clamps too,
            # so the candidate position and the tracked box now agree.
            box = clamp_bbox(raw, image.width, image.height)
            located = self._locate(box, image)
            if located is None:
                continue
            position, distance = located
            # Distance sanity bound: a box near the horizon raycasts to a
            # near-infinite floor point, so drop implausibly far "subjects"
            # before engaging a phantom and walking at empty space.
            if distance > self.config.max_subject_distance_m:
                n_too_far += 1
                continue
            if self._on_mapped_obstacle(position):
                n_on_wall += 1
                continue
            candidates.append(Candidate(position=position, bbox=box))
        # Record drops so wander can tell "empty room" (do nothing, let the
        # patrol roam) from "saw things but rejected them all" (wedged facing a
        # wall/reflection -> rotate to look elsewhere, since the candidate list
        # being emptied would otherwise suppress the rescan).
        self._last_scan_dropped = n_too_far + n_on_wall
        if n_too_far or n_on_wall:
            logger.info(
                "greeter scan: dropped %d beyond %.0fm and %d on a mapped wall "
                "(ranging artifacts / glass reflections)",
                n_too_far, self.config.max_subject_distance_m, n_on_wall)
        return candidates

    def _on_mapped_obstacle(self, position: tuple[float, float]) -> bool:
        """Whether a candidate floor position lands on/near a mapped wall.

        Reads the latest global costmap (``world`` frame, same as the candidate)
        and tests it with :func:`pawtrack.occupancy.is_near_obstacle`. Returns
        False when the filter is disabled or no costmap has arrived, so a missing
        map never suppresses greeting.
        """
        if not self.config.reject_on_mapped_obstacle:
            return False
        with self._lock:
            costmap = self._latest_costmap
        if costmap is None:
            return False
        return is_near_obstacle(
            costmap.grid,
            costmap.resolution,
            (costmap.origin.position.x, costmap.origin.position.y),
            position,
            clearance_m=self.config.obstacle_clearance_m,
            cost_threshold=self.config.obstacle_cost_threshold,
        )

    # -- Step: navigate (approach) --

    def _engage_tick(self, image: Image | None) -> GreeterObservation:
        """Track the chosen subject and drive toward the standoff to wave.

        Greeting is unconditional once visible at a safe standoff (the machine
        gates on distance, not facing) -- no orbit-to-front step. While the
        tracker holds the subject this visually servos toward it (and keeps the
        last-known position refreshed, so a *moving* subject is chased). On a
        track loss it does not bail; it dead-reckons to the last-known position
        and re-detects there (:meth:`_deadreckon_tick`).
        """
        if image is None:
            self.cmd_vel.publish(Twist.zero())
            return GreeterObservation(subject_visible=False)
        detection = self._track(image)
        if detection is None:
            return self._deadreckon_tick(image)
        x1, _, x2, _ = detection.bbox
        self._last_error_x = (
            ((x1 + x2) / 2.0 - image.width / 2.0) / (image.width / 2.0))

        located = self._locate(detection.bbox, image)
        distance = None if located is None else located[1]
        if located is not None:
            self._last_position = located[0]  # refresh dest from the live position
            self._publish_world_pose(image, located[0])  # 3D marker in Rerun

        self.cmd_vel.publish(
            self._approach_twist(detection.bbox, image.width, distance))
        dist_label = "?" if distance is None else f"{distance:.1f}m"
        self._publish_debug(image, detection.bbox, f"approach  d={dist_label}")
        return GreeterObservation(subject_visible=True, distance_m=distance)

    def _deadreckon_tick(self, image: Image) -> GreeterObservation:
        """Tracker lost the subject: drive to its last-known spot, re-detect there.

        Uses odom (not the camera) to navigate to the last-known floor position,
        so a brief loss does not abandon the engagement. On arrival it re-runs
        the detector: a match near the last-known spot re-acquires the subject
        (it may have shifted); none means it moved off or was a bad fix, and the
        machine gives up (``reached_destination`` with the subject still unseen).
        Falls back to an in-place re-find yaw when no odom or last position is
        available.
        """
        with self._lock:
            odom = self._latest_odom
        if odom is None or self._last_position is None:
            self.cmd_vel.publish(self._search_twist())
            self._publish_debug(image, None, f"searching: {self._description}")
            return GreeterObservation(subject_visible=False)
        robot_xy = (odom.position.x, odom.position.y)
        range_m = math.hypot(
            self._last_position[0] - robot_xy[0],
            self._last_position[1] - robot_xy[1])
        limit = self.config.standoff_m + self.config.standoff_tolerance_m
        if range_m > limit:
            vx, wz = drive_to_position(
                robot_xy, odom.yaw, self._last_position, self.config.standoff_m,
                forward_gain=self.config.approach_forward_gain,
                turn_gain=self.config.approach_turn_gain,
                max_forward_mps=self.config.approach_max_forward_mps,
                max_yaw_radps=self.config.approach_max_yaw_radps,
                heading_tol_rad=self.config.deadreckon_heading_tol_rad)
            self.cmd_vel.publish(
                Twist(linear=Vector3(vx, 0.0, 0.0),
                      angular=Vector3(0.0, 0.0, wz)))
            self._publish_debug(
                image, None, f"to last seen  d={range_m:.1f}m")
            return GreeterObservation(
                subject_visible=False, distance_m=range_m,
                reached_destination=False)
        # Arrived at the last-known spot: stop, and try to re-detect there.
        self.cmd_vel.publish(Twist.zero())
        now = time.monotonic()
        if now - self._last_scan_s < self.config.scan_interval_s:
            # The re-scan (a VLM query) is rate-limited; hold and wait for it
            # rather than declaring the subject gone before we have looked.
            self._publish_debug(image, None, "arrived; re-checking")
            return GreeterObservation(
                subject_visible=False, distance_m=range_m,
                reached_destination=False)
        self._last_scan_s = now
        position = self._reacquire_near_last(image)
        if position is not None:
            self._last_position = position
            self._publish_world_pose(image, position)
            self._publish_debug(image, None, "re-acquired at last seen")
            return GreeterObservation(subject_visible=True, distance_m=range_m)
        # Re-scanned and nothing near the last-known spot: it moved off or was a
        # bad fix -- give up (the machine skips it on reached_destination).
        self._publish_debug(image, None, "arrived, subject gone")
        return GreeterObservation(
            subject_visible=False, distance_m=range_m, reached_destination=True)

    def _reacquire_near_last(self, image: Image):
        """Re-detect a subject near the last-known position; re-init the tracker.

        Runs the multi-box VLM detect and matches the candidate nearest the
        last-known floor position. Returns that position (within
        ``redetect_radius_m``) after re-seeding the tracker on its box, or None
        if nothing matches close enough.
        """
        candidates = self._scan(image)
        if not candidates or self._last_position is None:
            return None
        nearest = min(
            candidates,
            key=lambda c: math.dist(c.position, self._last_position))
        if math.dist(nearest.position, self._last_position) > (
                self.config.redetect_radius_m):
            return None
        if not self._init_tracker(nearest.bbox, image):
            return None
        return nearest.position

    def _search_twist(self) -> Twist:
        """Yaw toward the subject's last-seen side to bring it back into frame.

        On a brief track loss the subject has usually just slid out of the
        camera's view as the dog turned to engage; rotating the way that would
        re-centre its last-seen position recovers it, rather than freezing
        (which strands an off-angle subject out of frame until the lost timeout
        gives up). Pure yaw, no forward motion -- it does not drive blind.
        """
        wz = centering_yaw(
            self._last_error_x, self.config.approach_turn_gain,
            self.config.approach_max_yaw_radps)
        return Twist(
            linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, wz))

    def _approach_twist(
        self, bbox, image_width: int, distance_m: float | None
    ) -> Twist:
        """Drive toward the subject to the standoff distance (the planner seam).

        Uses the geometric ground distance from the raycast plus bbox centering,
        not a monocular width model: our target is a person *on a chair*, whose
        tracked box is wider than the person-shoulder width a width-based
        estimator assumes, which would stop the approach short. A real planner
        would replace this method.
        """
        x1, _, x2, _ = bbox
        error_x = ((x1 + x2) / 2.0 - image_width / 2.0) / (image_width / 2.0)
        if distance_m is None:
            # No ground fix: face the subject but do not drive blind.
            wz = centering_yaw(
                error_x, self.config.approach_turn_gain,
                self.config.approach_max_yaw_radps)
            return Twist(
                linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, wz))
        vx, wz = approach_velocity(
            distance_m, self.config.standoff_m, error_x,
            deadband_m=self.config.standoff_deadband_m,
            forward_gain=self.config.approach_forward_gain,
            turn_gain=self.config.approach_turn_gain,
            max_forward_mps=self.config.approach_max_forward_mps,
            max_reverse_mps=self.config.approach_max_reverse_mps,
            max_yaw_radps=self.config.approach_max_yaw_radps,
        )
        return Twist(
            linear=Vector3(vx, 0.0, 0.0), angular=Vector3(0.0, 0.0, wz))

    # -- Step: greet (on phase edges) --

    def _on_enter(
        self, state: str, reason: str | None = None, now: float = 0.0
    ) -> None:
        # One INFO line per phase change narrates the whole run in the logs.
        logger.info(
            "greeter -> %s%s", state, f" ({reason})" if reason else "")
        if state == "greet":
            # Halt first: stop the patrol and override this tick's approach
            # command, so the wave does not start with the dog still moving.
            self._halt()
            self._send_sport(_HELLO_API_ID)
        elif state == "cooldown":
            # Record the greet and clear the target FIRST. These are pure and
            # cannot fail, so a hiccup in the fallible steps below (a VLM timeout
            # or a sport-command error) can never leave the subject un-marked --
            # which would otherwise let the loop re-select and re-greet it,
            # because these entered-edge effects do not run again.
            if self._last_position is not None:
                self._registry.mark_visited(self._last_position, now)
            self._chosen = None
            # Optional flair: if the subject faces the camera, reward them with a
            # FingerHeart; if not (or unsure), just move on. Best-effort -- a VLM
            # or sport failure here must not skip the mandatory recovery below.
            # The dog is stopped, so the brief blocking gesture is fine (same
            # pattern as the one-shot wave_hello).
            try:
                facing = self._detect_facing()
                # Logged both ways so the decision is observable in sim, where
                # the FingerHeart animation itself is a no-op.
                logger.info(
                    "greeter: %r faces the camera=%s%s",
                    self._description, facing,
                    " -> FingerHeart" if facing else " -> no heart, moving on")
                if facing:
                    if self._send_sport(_FINGER_HEART_API_ID):
                        time.sleep(self.config.heart_duration_s)
            except Exception:  # noqa: BLE001 - flair must never block recovery
                logger.exception(
                    "greeter: facing/heart step failed; skipping it")
            # Mandatory: the gesture(s) (Hello, and FingerHeart which rears up on
            # its hind legs) leave the robot out of BalanceStand. Issue
            # RecoveryStand now; BalanceStand is deferred to the wander edge so
            # RecoveryStand gets the whole cooldown to physically stand the dog up
            # before BalanceStand -- sending BalanceStand right after (a 1.5 s
            # settle) interrupted the stand-up on the real robot.
            self._send_sport(_RECOVERY_STAND_API_ID)
        elif state == "wander":
            # Record/clear state FIRST (pure, cannot fail): skip a subject we gave
            # up on -- a stalled approach (reason "stuck") or one that was gone
            # when we reached its last-known spot (reason "lost") -- so we do not
            # re-pick and re-fail on it. Doing this before the commands below means
            # a sport hiccup cannot leave it un-skipped. The revisit-forget window
            # still makes it eligible again later (e.g. a person who stepped away
            # returns).
            if reason in ("stuck", "lost") and self._last_position is not None:
                self._registry.mark_visited(self._last_position, now)
            self._chosen = None
            self._last_position = None
            # Halt any leftover engage command, then re-enable BalanceStand so the
            # resuming patrol's cmd_vel is accepted -- and, after a greet, this is
            # the BalanceStand that completes the gesture recovery (RecoveryStand
            # ran on the cooldown edge and has had the cooldown to settle). The
            # patrol is (re)started by the next _wander_tick.
            self.cmd_vel.publish(Twist.zero())
            self._send_sport(_BALANCE_STAND_API_ID)

    def _detect_facing(self) -> bool:
        """Whether the just-greeted subject faces the camera (one-shot VLM).

        Asked once after the wave, while the dog is stopped, so its latency does
        not matter and it costs no extra GPU (the VL model is remote). An
        ambiguous or unparseable answer defaults to False, so an uncertain read
        does not trigger the rear-up FingerHeart.
        """
        with self._lock:
            image = self._latest_image
        if image is None:
            return False
        return detect_facing(
            self._get_vl_model(), image, self._description) is True

    def _recover_locomotion(self) -> None:
        """Restore a clean standing + balance state after a sport gesture.

        A gesture (Hello or the FingerHeart) leaves the robot out of
        BalanceStand, so cmd_vel is ignored. Bring it back up with RecoveryStand,
        let it settle, then enter BalanceStand so it can walk and do other things
        again. Synchronous (blocks for ``recover_settle_s``); used both on the
        loop's cooldown edge and by the one-shot wave. A no-op off-robot.
        """
        self._send_sport(_RECOVERY_STAND_API_ID)
        time.sleep(self.config.recover_settle_s)
        self._send_sport(_BALANCE_STAND_API_ID)

    def _send_sport(self, api_id: int) -> bool:
        """Issue a sport command, tolerating a transient closed data channel.

        The WebRTC publish raises ``Data channel is not open`` whenever the
        channel is not in the open state -- it can briefly drop mid-session. A
        failed gesture must never crash the control loop, so this retries a few
        times to bridge a transient blip, then logs and gives up. Any other
        failure is re-raised for the loop's fail-safe to handle.

        Args:
            api_id: The SPORT_MOD api id to publish (e.g. Hello, BalanceStand).

        Returns:
            True if the command was delivered, False if the channel stayed
            closed across every retry.
        """
        name = _SPORT_NAMES.get(api_id, "?")
        connection = self._connection
        if connection is None:
            logger.warning(
                "greeter: NO GO2 connection -- sport %s (%d) is a no-op "
                "(this is why a gesture would not play)", name, api_id)
            return False
        last_error: Exception | None = None
        for attempt in range(self.config.sport_retry_attempts):
            try:
                connection.publish_request(
                    RTC_TOPIC["SPORT_MOD"], {"api_id": api_id})
                # Log on success too -- otherwise the gesture/recovery sequence
                # (Hello / RecoveryStand / BalanceStand / FingerHeart) is
                # invisible in the run log, which made the "no wave / no stand
                # up" reports hard to diagnose.
                logger.info(
                    "greeter: sent sport %s (%d)%s", name, api_id,
                    f" [retry {attempt}]" if attempt else "")
                return True
            except Exception as error:  # noqa: BLE001 - lib raises bare Exception
                if "Data channel is not open" not in str(error):
                    raise  # unexpected -> let the loop's fail-safe handle it
                last_error = error
                if attempt + 1 < self.config.sport_retry_attempts:
                    time.sleep(self.config.sport_retry_delay_s)
        logger.warning(
            "greeter: sport %s (%d) NOT delivered after %d attempts (data "
            "channel not open): %s",
            name, api_id, self.config.sport_retry_attempts, last_error)
        return False

    # -- Tracker / models / geometry --

    def _init_tracker(self, bbox, image: Image) -> bool:
        if not valid_bbox(list(bbox), image.width, image.height):
            logger.warning("greeter: VLM bbox %s invalid for the frame", bbox)
            return False
        box = clamp_bbox(bbox, image.width, image.height)
        tracker = self._get_tracker()
        detections = tracker.init_track(
            image=image, box=np.array(box, dtype=np.float32), obj_id=1)
        return len(detections) > 0

    def _track(self, image: Image):
        tracker = self._get_tracker()
        detections = tracker.process_image(image)
        people = list(getattr(detections, "detections", []))
        if not people:
            return None
        best = max(people, key=lambda d: d.bbox_2d_volume())
        return best

    def _locate(self, bbox, image: Image):
        """Subject world position + distance for a bbox.

        Prefers the lidar pointcloud (real returns off the subject's body --
        robust to the dog's low viewpoint), and falls back to the monocular
        ground raycast when no lidar is wired or no points land in the box.

        Returns ``((x, y), distance_m)`` in the world frame, or None if neither
        method can produce a fix.
        """
        located = self._locate_lidar(bbox, image)
        if located is not None:
            return located
        return self._locate_raycast(bbox, image)

    def _locate_lidar(self, bbox, image: Image):
        """Range the subject from the lidar returns inside its bbox.

        Projects the lidar cloud into the camera, keeps the points that fall in
        the tracked box, and takes their centroid via DimOS'
        :meth:`Detection3DPC.from_2d`. Unlike the ground raycast it does not
        assume the subject's feet rest on a flat floor -- the failure mode that
        put a seated person metres from their true range. Returns
        ``((x, y), distance_m)`` in the world frame, or None if unavailable.

        Logs (rate-limited) *why* a fix did or did not happen, so a run that
        falls back to the raycast says which boundary failed: no cloud arriving,
        an empty cloud, a missing TF, or no lidar returns inside the box.
        """
        with self._lock:
            pointcloud = self._latest_pointcloud
        if self._camera_info is None:
            return None
        if pointcloud is None:
            self._log_lidar("no cloud received on /lidar yet")
            return None
        point_count = pointcloud.as_numpy()[0].shape[0]
        if point_count == 0:
            self._log_lidar("/lidar cloud is empty")
            return None
        # Use the cloud's own frame (DimOS' detector does the same): the tf must
        # map those points into the camera, whatever frame the driver tags them.
        world_to_optical = self.tf.get(
            self.config.camera_optical_frame_id,
            pointcloud.frame_id,
            time_point=image.ts,
            time_tolerance=self.config.tf_tolerance_s,
        )
        if world_to_optical is None:
            self._log_lidar(
                "no TF %s<-%s at the image timestamp"
                % (self.config.camera_optical_frame_id, pointcloud.frame_id))
            return None
        detection = Detection2DBBox(
            bbox=tuple(bbox), track_id=-1, class_id=0, confidence=1.0,
            name=self.config.description, ts=image.ts, image=image,
        )
        # No filters: the Go2 lidar is sparse, so the default outlier/raycast
        # filters can drop every return inside a person's box. The bbox alone
        # isolates the subject well enough for a range estimate.
        detection_3d = Detection3DPC.from_2d(
            detection,
            world_pointcloud=pointcloud,
            camera_info=self._camera_info,
            world_to_optical_transform=world_to_optical,
            filters=[],
        )
        if detection_3d is None:
            self._log_lidar(
                "from_2d found no lidar returns in the bbox (cloud=%d pts)"
                % point_count)
            return None
        # Camera origin in the cloud's own frame, so it subtracts cleanly from
        # the (same-frame) detection centre.
        cam = self.tf.get(
            pointcloud.frame_id,
            self.config.camera_optical_frame_id,
            time_point=image.ts,
            time_tolerance=self.config.tf_tolerance_s,
        )
        if cam is None:
            return None
        origin = cam.to_matrix()[:3, 3]
        center = detection_3d.center
        distance = math.hypot(center.x - origin[0], center.y - origin[1])
        self._log_lidar(
            "fix from %d-pt cloud -> (%.2f, %.2f) d=%.2fm"
            % (point_count, center.x, center.y, distance))
        return ((center.x, center.y), distance)

    def _log_lidar(self, message: str) -> None:
        """Rate-limited lidar-path diagnostics (the why behind a fix/fallback)."""
        now = time.monotonic()
        if now - self._last_lidar_log_s >= self.config.lidar_log_interval_s:
            self._last_lidar_log_s = now
            logger.info("greeter lidar: %s", message)

    def _locate_raycast(self, bbox, image: Image):
        """Floor position + distance of a bbox via the monocular ground raycast.

        Returns ``((x, y), distance_m)`` in the world frame, or None if
        intrinsics or the TF are unavailable or the ray misses the floor.
        """
        if self._intrinsics is None:
            return None
        transform = self.tf.get(
            self.config.world_frame_id,
            self.config.camera_optical_frame_id,
            time_point=image.ts,
            time_tolerance=self.config.tf_tolerance_s,
        )
        if transform is None:
            return None
        matrix = transform.to_matrix()
        point = pixel_to_ground_point(
            ground_contact_pixel(bbox),
            self._intrinsics,
            matrix,
            ground_z=self.config.world_floor_z_m,
        )
        if point is None:
            return None
        origin = matrix[:3, 3]
        distance = math.hypot(point[0] - origin[0], point[1] - origin[1])
        return ((point[0], point[1]), distance)

    def _get_vl_model(self) -> VlModel:
        with self._lock:
            if self._vl_model is None:
                self._vl_model = QwenChinaVlModel()
            return self._vl_model

    def _get_tracker(self) -> EdgeTAMProcessor:
        with self._lock:
            if self._tracker is None:
                self._tracker = EdgeTAMProcessor()
            return self._tracker

    # -- Misc --

    def _on_color_image(self, image: Image) -> None:
        with self._lock:
            self._latest_image = image

    def _on_pointcloud(self, pointcloud: PointCloud2) -> None:
        with self._lock:
            self._latest_pointcloud = pointcloud

    def _on_odom(self, odom: PoseStamped) -> None:
        with self._lock:
            self._latest_odom = odom

    def _on_costmap(self, costmap: OccupancyGrid) -> None:
        with self._lock:
            self._latest_costmap = costmap

    def _publish_world_pose(
        self, image: Image, position: tuple[float, float]
    ) -> None:
        """Publish the subject's floor position (world frame) for Rerun 3D."""
        self.subject_world_pose.publish(PoseStamped(
            ts=image.ts,
            frame_id=self.config.world_frame_id,
            position=[position[0], position[1], self.config.world_floor_z_m],
            orientation=[0.0, 0.0, 0.0, 1.0],
        ))

    def _publish_debug(self, image: Image, bbox, label: str) -> None:
        frame = image.to_opencv().copy()
        color = (0, 255, 0) if bbox is not None else (0, 0, 255)
        if bbox is not None:
            x1, y1, x2, y2 = (int(round(v)) for v in bbox)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            frame, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        self.debug_image.publish(Image.from_opencv(frame, ts=image.ts))

    def _stop_loop(self) -> None:
        self._should_stop.set()
        with self._lock:
            thread = self._thread
            self._thread = None
        if thread is not None:
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

    def _halt(self) -> None:
        """Stop the robot: zero velocity AND stop the patrol, independently.

        Used by the stop skill, module shutdown, and the loop's fail-safe path.
        Each mechanism is guarded on its own so a failure in one still attempts
        the other, and neither may raise -- either would leave the robot driving.
        """
        try:
            self.cmd_vel.publish(Twist.zero())
        except Exception:  # noqa: BLE001 - last-resort halt must not raise
            logger.exception("greeter: zero-velocity publish failed")
        try:
            patrol = getattr(self, "_patrolling_module_spec", None)
            if patrol is not None and patrol.is_patrolling():
                patrol.stop_patrol()
        except Exception:  # noqa: BLE001 - last-resort halt must not raise
            logger.exception("greeter: stop_patrol failed")

    def _reset_engagement(self) -> None:
        """Clear transient per-run state for a clean wander start.

        Keeps the visited registry, so subjects greeted in an earlier run stay
        skipped across restarts.
        """
        self._machine.reset()
        self._chosen = None
        self._last_position = None
        self._last_snapshot = None
        self._last_trace = None
        self._last_scan_s = 0.0
        self._rescanning = False
        self._roaming = False
        self._roam_anchor_xy = None
        self._roam_progress_s = 0.0

    def _set_error_status(self, message: str) -> None:
        self._publish_status(GreeterSnapshot(
            state="wander", message=message, reason="error"))

    def _publish_status(
        self, snapshot: GreeterSnapshot, obs: GreeterObservation | None = None
    ) -> None:
        """Record + publish a rich phase trace on greeter_phase (phase + why).

        Beyond the phase itself, the trace carries the perception that drove this
        tick (visible / distance), the patrol state, the subject's floor
        position, and the greeted count -- so one stream explains the behavior.
        """
        self._last_snapshot = snapshot
        distance = (
            round(obs.distance_m, 2)
            if obs is not None and obs.distance_m is not None else None
        )
        position = (
            [round(self._last_position[0], 2), round(self._last_position[1], 2)]
            if self._last_position is not None else None
        )
        trace = {
            "state": snapshot.state,
            "message": snapshot.message,
            "entered": snapshot.entered,
            "reason": snapshot.reason,
            "patrolling": self._is_patrolling(),
            "subject_visible": bool(obs.subject_visible) if obs else False,
            "distance_m": distance,
            "subject_xy": position,
            "greeted": self._registry.visited_count,
        }
        self._last_trace = json.dumps(trace, separators=(",", ":"))
        self.greeter_phase.publish(self._last_trace)
        # Persist the gate inputs at each phase change (greeter_phase itself is
        # live LCM only and is not saved to the run log). This puts the
        # distance / subject_xy behind every transition -- e.g. why a
        # greet fired -- into main.jsonl for after-the-fact debugging.
        if snapshot.entered:
            logger.info("greeter phase trace: %s", self._last_trace)

    def _is_patrolling(self) -> bool:
        patrol = getattr(self, "_patrolling_module_spec", None)
        return patrol is not None and patrol.is_patrolling()
