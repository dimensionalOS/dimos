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

"""Capture robot camera frames and upload them to the robomoo app.

Skills:
  - take_picture:      one-shot capture of the current frame.
  - tilt_and_capture:  pitch the body to aim the camera, photograph, re-level.
  - create_lidar_map:  the LiDAR-mapping pass — autonomously explore a new room
                       to build the 2D occupancy / costmap, snapping a photo
                       every few seconds along the way for spatial memory.
  - vr_scan:           the dense camera sweep for 3D / VR (gaussian-splat)
                       reconstruction. Run *after* the room has been mapped.

Each frame is JPEG-encoded and POSTed (with the robot's odom pose) to robomoo's
`/api/robot/frame` (shared-secret bearer token). Configure via env:

    ROBOMOO_URL=https://gateway-...up.railway.app
    ROBOT_INGEST_TOKEN=<secret matching the server>
"""

import math
import os
import threading
import time

import cv2
from dimos_lcm.std_msgs import Bool
import httpx
import numpy as np

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3, make_vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.base import NavigationState
from dimos.navigation.frontier_exploration.frontier_explorer_spec import (
    FrontierExplorerSpec,
)
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.robot.unitree.tilt_spec import TiltSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TakePictureSkillConfig(ModuleConfig):
    robomoo_url: str = os.getenv("ROBOMOO_URL", "")
    ingest_token: str = os.getenv("ROBOT_INGEST_TOKEN", "")


class TakePictureSkill(Module):
    config: TakePictureSkillConfig
    color_image: In[Image]
    odom: In[PoseStamped]
    # Latest occupancy / costmap used by `vr_scan` to plan a perimeter orbit on
    # the already-built map (consumer-only — published by VoxelGridMapper +
    # CostMapper, or merged_map when relocalization is active).
    global_costmap: In[OccupancyGrid]
    # Drive frontier exploration (consumed by WavefrontFrontierExplorer).
    explore_cmd: Out[Bool]
    stop_explore_cmd: Out[Bool]
    # Auto-wired (structurally) to WavefrontFrontierExplorer — lets us gate the
    # capture loop on whether exploration is still running.
    _explorer: FrontierExplorerSpec
    # Auto-wired (structurally) to UnitreeSkillContainer.tilt_body — lets
    # tilt_and_capture / vr_scan aim the body-fixed camera without owning the
    # connection.
    _tilt: TiltSpec
    # Auto-wired (structurally) to ReplanningAStarPlanner — vr_scan sends each
    # orbit waypoint via set_goal and polls is_goal_reached / get_state.
    _navigation: NavigationInterfaceSpec

    @rpc
    def start(self) -> None:
        super().start()
        self._latest: Image | None = None
        self._pose: PoseStamped | None = None
        self._latest_costmap: OccupancyGrid | None = None
        self._capture_thread: threading.Thread | None = None
        self._pan_thread: threading.Thread | None = None
        self._capture_stop = threading.Event()
        # Outstanding fire-and-forget upload threads from take_picture().
        self._uploads: list[threading.Thread] = []
        self._uploads_lock = threading.Lock()
        self.color_image.subscribe(self._on_image)
        self.odom.subscribe(self._on_odom)
        self.global_costmap.subscribe(self._on_costmap)

    @rpc
    def stop(self) -> None:
        self._capture_stop.set()
        for attr in ("_capture_thread", "_pan_thread"):
            thread = getattr(self, attr, None)
            if thread is not None and thread.is_alive():
                thread.join(timeout=5.0)
        with self._uploads_lock:
            uploads = list(self._uploads)
        for t in uploads:
            if t.is_alive():
                t.join(timeout=5.0)
        super().stop()

    def _on_image(self, image: Image) -> None:
        self._latest = image

    def _on_odom(self, pose: PoseStamped) -> None:
        self._pose = pose

    def _on_costmap(self, costmap: OccupancyGrid) -> None:
        self._latest_costmap = costmap

    def _configured(self) -> bool:
        return bool(self.config.robomoo_url and self.config.ingest_token)

    # Encode a given frame and POST it (with pose) to robomoo. Returns the stored
    # key, or None if there's no frame / encode failed. Raises on HTTP error.
    def _upload_frame(
        self,
        frame: Image | None,
        pose: PoseStamped | None,
        note: str = "",
        label: str = "",
        extra: dict[str, str] | None = None,
    ) -> str | None:
        if frame is None:
            return None
        ok, buf = cv2.imencode(".jpg", frame.data)
        if not ok:
            return None

        data: dict[str, str] = {}
        if note:
            data["note"] = note
        if label:
            data["label"] = label
        if pose is not None:
            data["poseX"] = str(pose.position.x)
            data["poseY"] = str(pose.position.y)
        # Extra grouping tags (e.g. run / position / angle for vr_scan).
        if extra:
            data.update(extra)

        resp = httpx.post(
            f"{self.config.robomoo_url.rstrip('/')}/api/robot/frame",
            headers={"Authorization": f"Bearer {self.config.ingest_token}"},
            files={"file": ("frame.jpg", buf.tobytes(), "image/jpeg")},
            data=data,
            timeout=30.0,
        )
        resp.raise_for_status()
        return resp.json().get("key", "")

    # Thin wrapper used by the explore capture loop: upload the latest frame/pose.
    def _upload_current(
        self, note: str = "", label: str = "", extra: dict[str, str] | None = None
    ) -> str | None:
        return self._upload_frame(
            getattr(self, "_latest", None),
            getattr(self, "_pose", None),
            note=note,
            label=label,
            extra=extra,
        )

    @skill
    def take_picture(self, note: str = "") -> SkillResult:
        """Capture a photo from the robot's camera and upload it.

        Use whenever the user asks the robot to take or capture a single picture
        or photo of what it currently sees. `note` is an optional short caption
        to tag the image with (e.g. "kitchen", "plant"). Returns immediately; the
        encode + upload happen in the background.
        """
        frame = getattr(self, "_latest", None)
        if frame is None:
            return SkillResult.fail("NO_FRAME", "No camera frame received yet")
        if not self._configured():
            return SkillResult.fail("NOT_CONFIGURED", "ROBOMOO_URL / ROBOT_INGEST_TOKEN not set")

        # Snapshot the frame + pose now so the background upload sends exactly what
        # the robot saw at call time, not a later frame.
        pose = getattr(self, "_pose", None)

        def _bg() -> None:
            try:
                key = self._upload_frame(frame, pose, note=note, label=note)
                logger.info("take_picture uploaded frame key=%s", key)
            except Exception:
                logger.exception("take_picture upload failed")

        t = threading.Thread(target=_bg, daemon=True, name="take-picture-upload")
        with self._uploads_lock:
            # Drop finished threads so the list doesn't grow unbounded.
            self._uploads = [u for u in self._uploads if u.is_alive()]
            self._uploads.append(t)
        t.start()
        return SkillResult.ok("Picture captured; uploading in the background.")

    @skill
    def tilt_and_capture(
        self,
        pitch_deg: float = -20.0,
        note: str = "",
        settle_s: float = 1.0,
    ) -> SkillResult:
        """Tilt the body to aim the camera, photograph that view, then re-level.

        Use to photograph things above or below the robot's straight-ahead view
        (the camera is body-fixed). NEGATIVE pitch_deg looks UP, positive looks
        DOWN. Runs in the background: tilts, waits `settle_s` for the body to
        settle, captures + uploads the tilted view, then returns the body to
        level. Returns immediately. The robot should be standing first.
        """
        if getattr(self, "_latest", None) is None:
            return SkillResult.fail("NO_FRAME", "No camera frame received yet")
        if not self._configured():
            return SkillResult.fail("NOT_CONFIGURED", "ROBOMOO_URL / ROBOT_INGEST_TOKEN not set")

        def _bg() -> None:
            try:
                self._tilt.tilt_body(pitch_deg=pitch_deg)
                # Wait for the body to physically reach the pose and a fresh
                # camera frame to arrive before snapshotting.
                time.sleep(settle_s)
                key = self._upload_frame(
                    getattr(self, "_latest", None),
                    getattr(self, "_pose", None),
                    note=note,
                    label=note,
                )
                logger.info("tilt_and_capture uploaded frame key=%s", key)
            except Exception:
                logger.exception("tilt_and_capture failed")
            finally:
                # Always return the body to level, even if capture failed.
                try:
                    self._tilt.tilt_body()
                except Exception:
                    logger.exception("tilt_and_capture re-level failed")

        t = threading.Thread(target=_bg, daemon=True, name="tilt-and-capture")
        with self._uploads_lock:
            self._uploads = [u for u in self._uploads if u.is_alive()]
            self._uploads.append(t)
        t.start()
        return SkillResult.ok(
            f"Tilting to pitch={pitch_deg} deg, capturing, then re-leveling (background)."
        )

    @skill
    def create_lidar_map(
        self,
        interval_s: float = 4.0,
        max_duration_s: float = 600.0,
        note: str = "exploring",
    ) -> SkillResult:
        """Build the 2D LiDAR map of a new room by autonomously exploring it.

        Use when the user asks to map / scan the room with LiDAR, or asks the
        robot to explore / wander / look around a new space. Starts autonomous
        frontier exploration (which is what builds the 2D occupancy / costmap
        from LiDAR), and snaps a photo every `interval_s` seconds along the way
        so spatial memory has visual context for the places traversed. Stops
        on its own once the room is fully explored; `max_duration_s` is a
        safety cap. Returns immediately; runs in the background.

        Distinct from `vr_scan`, which is the dense camera sweep for 3D / VR
        reconstruction and is meant to run *after* the room has been mapped.
        """
        if not self._configured():
            return SkillResult.fail("NOT_CONFIGURED", "ROBOMOO_URL / ROBOT_INGEST_TOKEN not set")

        # Cancel any in-flight run before starting a new one.
        self._capture_stop.set()
        if self._capture_thread is not None and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=5.0)
        self._capture_stop = threading.Event()

        self.explore_cmd.publish(Bool(data=True))
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            args=(interval_s, max_duration_s, note),
            daemon=True,
            name="create-lidar-map",
        )
        self._capture_thread.start()

        return SkillResult.ok(
            f"Exploring and capturing a photo every {interval_s:.0f}s until the "
            "room is fully explored."
        )

    def _exploring(self) -> bool:
        try:
            return bool(self._explorer.is_exploration_active())
        except Exception as e:
            logger.warning("is_exploration_active() failed: %s", e)
            return True

    def _capture_loop(self, interval_s: float, max_duration_s: float, note: str) -> None:
        start = time.monotonic()
        grace_s = 6.0  # let exploration spin up before trusting the active flag
        count = 0
        try:
            while not self._capture_stop.is_set():
                elapsed = time.monotonic() - start
                if elapsed > max_duration_s:
                    logger.info("create_lidar_map hit max_duration_s cap")
                    break
                if elapsed > grace_s and not self._exploring():
                    logger.info("exploration finished — stopping capture loop")
                    break
                try:
                    if self._upload_current(note=note, label=note):
                        count += 1
                except Exception as e:
                    logger.warning("create_lidar_map upload failed: %s", e)
                self._capture_stop.wait(interval_s)
        finally:
            try:
                self.stop_explore_cmd.publish(Bool(data=True))
            except Exception:
                pass
            logger.info("create_lidar_map finished: uploaded %d photos", count)

    @skill
    def vr_scan(
        self,
        run_id: str = "",
        robot_radius_m: float = 0.40,
        waypoint_spacing_m: float = 0.50,
        lookahead_m: float = 0.40,
        min_capture_distance_m: float = 0.20,
        pan_amplitude_deg: float = 30.0,
        pan_freq_hz: float = 0.3,
        pitch_amplitude_deg: float = 8.0,
        pitch_freq_hz: float = 0.17,
        min_sharpness: float = 30.0,
        max_waypoints: int = 100,
        max_duration_s: float = 600.0,
        close_loop: bool = True,
        note: str = "vr_scan",
    ) -> SkillResult:
        """Thorough camera sweep of the room for 3D / VR (gaussian-splat) reconstruction.

        Distinct from `create_lidar_map` (which builds the 2D LiDAR map and
        grabs photos opportunistically). This is the dedicated VR-capture pass:
        plan a perimeter orbit on the already-built costmap, walk it
        *continuously* (no stop-start) by pre-empting each waypoint as the
        robot approaches it, and pan the body yaw sinusoidally (±~30°, plus a
        smaller pitch sweep) so frames see the room from varied angles. Snaps
        a photo at each pan extremum (zero angular velocity → no rotational
        blur) provided the robot has moved at least `min_capture_distance_m`
        since the last frame. Blurry frames are dropped via a Laplacian
        sharpness check. Each frame is tagged with the run, the *nearest*
        waypoint (`position`), the heading, and the world pose. Stops when the
        orbit is complete or `max_duration_s` elapses. Returns immediately and
        runs in the background. The robot should be standing first, and a map
        must already exist — run `create_lidar_map` (or load a saved map).
        """
        if getattr(self, "_latest", None) is None:
            return SkillResult.fail("NO_FRAME", "No camera frame received yet")
        if not self._configured():
            return SkillResult.fail("NOT_CONFIGURED", "ROBOMOO_URL / ROBOT_INGEST_TOKEN not set")
        if getattr(self, "_navigation", None) is None:
            return SkillResult.fail(
                "NO_PLANNER",
                "Navigation planner not wired — vr_scan needs a blueprint with "
                "ReplanningAStarPlanner (e.g. unitree-go2-agentic-gemini-rooms).",
            )
        costmap = getattr(self, "_latest_costmap", None)
        if costmap is None:
            return SkillResult.fail(
                "NO_MAP",
                "No map yet — run create_lidar_map first (or load a saved map).",
            )
        pose = getattr(self, "_pose", None)
        if pose is None:
            return SkillResult.fail("NO_POSE", "No odometry received yet.")

        robot_xy = (pose.position.x, pose.position.y)
        waypoints = self._plan_perimeter_orbit(
            costmap, robot_xy, robot_radius_m, waypoint_spacing_m, max_waypoints
        )
        if not waypoints:
            return SkillResult.fail(
                "EMPTY_FREE_SPACE",
                "Couldn't plan a perimeter orbit — the map's free area is too small or "
                "doesn't contain the robot. Map more of the room first.",
            )

        # Close the loop so COLMAP gets a re-imaged start area (huge for global
        # consistency). Only meaningful with at least a triangle.
        if close_loop and len(waypoints) >= 3:
            waypoints = [*waypoints, waypoints[0]]

        run = run_id or f"vr-{int(time.time())}"

        # Cancel any in-flight scan/pan before starting a new one.
        self._capture_stop.set()
        for attr in ("_capture_thread", "_pan_thread"):
            thread = getattr(self, attr, None)
            if thread is not None and thread.is_alive():
                thread.join(timeout=5.0)
        self._capture_stop = threading.Event()
        # Shared state for the capture/pan thread to read.
        self._vr_scan_capture_count = 0

        logger.info(
            "vr_scan planned %d waypoints (run=%s, close_loop=%s)",
            len(waypoints), run, close_loop,
        )

        self._capture_thread = threading.Thread(
            target=self._vr_scan_orbit_loop,
            args=(run, waypoints, lookahead_m, max_duration_s),
            daemon=True,
            name="vr-scan-orbit",
        )
        self._pan_thread = threading.Thread(
            target=self._vr_scan_capture_pan_loop,
            args=(
                run, waypoints,
                pan_amplitude_deg, pan_freq_hz,
                pitch_amplitude_deg, pitch_freq_hz,
                min_capture_distance_m, min_sharpness, note,
            ),
            daemon=True,
            name="vr-scan-capture-pan",
        )
        self._capture_thread.start()
        self._pan_thread.start()

        return SkillResult.ok(
            f"Scanning the room for VR (run={run}): orbiting {len(waypoints)} waypoints "
            f"continuously on the existing map; head panning ±{pan_amplitude_deg:.0f}° / "
            f"±{pitch_amplitude_deg:.0f}° pitch, capturing at pan extrema. "
            "Running in the background."
        )

    def _vr_scan_orbit_loop(
        self,
        run: str,
        waypoints: list[tuple[float, float]],
        lookahead_m: float,
        max_duration_s: float,
    ) -> None:
        """Walk the orbit continuously: pre-empt each waypoint at `lookahead_m`
        by setting the next goal, so the planner threads through without
        stop-start motion. The final waypoint is required to actually arrive."""
        start = time.monotonic()
        visited = 0
        waypoint_timeout_s = 15.0

        def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
            return math.hypot(a[0] - b[0], a[1] - b[1])

        try:
            for wp_idx, (wx, wy) in enumerate(waypoints):
                if self._capture_stop.is_set():
                    break
                if time.monotonic() - start > max_duration_s:
                    logger.info("vr_scan hit max_duration_s cap")
                    break

                is_final = wp_idx == len(waypoints) - 1

                goal = PoseStamped(
                    position=make_vector3(wx, wy, 0.0),
                    orientation=Quaternion.from_euler(make_vector3(0.0, 0.0, 0.0)),
                    frame_id="map",
                )
                try:
                    self._navigation.set_goal(goal)
                except Exception:
                    logger.exception("vr_scan: set_goal failed for waypoint %d", wp_idx)
                    continue

                logger.info(
                    "vr_scan: nav to (%.2f, %.2f) [%d/%d]%s",
                    wx, wy, wp_idx + 1, len(waypoints),
                    " (final)" if is_final else "",
                )
                visited = wp_idx + 1

                goal_t0 = time.monotonic()
                # Small grace so state transitions out of IDLE before we poll.
                self._capture_stop.wait(0.3)

                while not self._capture_stop.is_set():
                    # Continuous orbit: advance as soon as we're near this
                    # waypoint, so the planner can replan to the next without
                    # decelerating to a full stop. The final waypoint requires
                    # actual arrival, so we don't skip closing the loop.
                    cur_pose = getattr(self, "_pose", None)
                    if cur_pose is not None and not is_final:
                        xy = (cur_pose.position.x, cur_pose.position.y)
                        if _dist(xy, (wx, wy)) <= lookahead_m:
                            break

                    # Arrival / abort fallback (covers the case where we arrived
                    # before lookahead triggered, and the final waypoint).
                    try:
                        state = self._navigation.get_state()
                        reached = self._navigation.is_goal_reached()
                    except Exception:
                        logger.exception("vr_scan: nav state query failed")
                        break

                    if state == NavigationState.IDLE:
                        if reached:
                            logger.info("vr_scan: goal reached")
                        else:
                            logger.warning("vr_scan: nav aborted, skipping waypoint")
                        break

                    if time.monotonic() - goal_t0 > waypoint_timeout_s:
                        logger.warning("vr_scan: goal timeout, skipping waypoint")
                        try:
                            self._navigation.cancel_goal()
                        except Exception:
                            logger.exception("vr_scan: cancel_goal failed")
                        break

                    self._capture_stop.wait(0.1)
        finally:
            # Signal the capture/pan thread to stop, then wait for it to flush
            # so the photo count in the final log is accurate.
            self._capture_stop.set()
            pan_thread = getattr(self, "_pan_thread", None)
            if pan_thread is not None and pan_thread is not threading.current_thread():
                pan_thread.join(timeout=10.0)
            count = getattr(self, "_vr_scan_capture_count", 0)
            logger.info(
                "vr_scan finished (run=%s): visited %d/%d waypoints, uploaded %d photos",
                run, visited, len(waypoints), count,
            )

    def _vr_scan_capture_pan_loop(
        self,
        run: str,
        waypoints: list[tuple[float, float]],
        pan_amp_deg: float,
        pan_freq_hz: float,
        pitch_amp_deg: float,
        pitch_freq_hz: float,
        min_capture_distance_m: float,
        min_sharpness: float,
        note: str,
    ) -> None:
        """Combined head-pan + capture trigger.

        At 20 Hz, command body yaw + pitch sinusoidally and trigger captures
        when the yaw VELOCITY changes sign — i.e. at pan extrema, where
        rotational blur is zero. A minimum translation gate prevents duplicate
        captures if the robot is momentarily still. Each frame is tagged with
        the *nearest* waypoint so the server groups frames by physical zone,
        not by goal index (captures fire during transit between waypoints, so
        goal-index grouping would be semantically wrong).

        Yaw clamped to ±0.6 rad inside tilt_body (≈ ±34°), the Go2's safe
        standing envelope. Body yaw is a chassis-relative offset that composes
        with the planner's base motion.
        """
        t0 = time.monotonic()
        interval_s = 0.05  # 20 Hz tick rate
        count = 0
        last_capture_xy: tuple[float, float] | None = None
        last_yaw_vel_sign = 0

        try:
            while not self._capture_stop.is_set():
                t = time.monotonic() - t0
                # Pan + pitch setpoints. Pitch frequency is intentionally
                # incommensurate with pan freq so the Lissajous-like coverage
                # explores varied yaw/pitch combinations.
                yaw = pan_amp_deg * math.sin(2.0 * math.pi * pan_freq_hz * t)
                pitch = pitch_amp_deg * math.sin(2.0 * math.pi * pitch_freq_hz * t)
                yaw_vel = (
                    pan_amp_deg * 2.0 * math.pi * pan_freq_hz
                    * math.cos(2.0 * math.pi * pan_freq_hz * t)
                )
                try:
                    self._tilt.tilt_body(pitch_deg=pitch, yaw_deg=yaw)
                except Exception:
                    logger.exception("vr_scan: tilt_body failed")

                # Detect a yaw-velocity sign change → pan extremum → capture.
                cur_sign = 1 if yaw_vel >= 0.0 else -1
                at_extremum = last_yaw_vel_sign != 0 and cur_sign != last_yaw_vel_sign

                if at_extremum:
                    cur_pose = getattr(self, "_pose", None)
                    if cur_pose is not None:
                        xy = (cur_pose.position.x, cur_pose.position.y)
                        far_enough = (
                            last_capture_xy is None
                            or math.hypot(xy[0] - last_capture_xy[0], xy[1] - last_capture_xy[1])
                            >= min_capture_distance_m
                        )
                        if far_enough:
                            wp_idx = _nearest_waypoint_idx(xy, waypoints) if waypoints else 0
                            angle = round(math.degrees(cur_pose.yaw)) % 360
                            if self._dispatch_capture(
                                run, wp_idx, count, angle, note,
                                min_sharpness=min_sharpness,
                            ):
                                last_capture_xy = xy
                                count += 1
                                self._vr_scan_capture_count = count

                last_yaw_vel_sign = cur_sign
                self._capture_stop.wait(interval_s)
        finally:
            # Best-effort level on the way out so the robot doesn't stay tilted.
            try:
                self._tilt.tilt_body()
            except Exception:
                logger.exception("vr_scan: re-level failed")
            # Final count snapshot for the orbit thread's summary line.
            self._vr_scan_capture_count = count

    def _plan_perimeter_orbit(
        self,
        costmap: OccupancyGrid,
        robot_xy: tuple[float, float],
        robot_radius_m: float,
        spacing_m: float,
        max_waypoints: int,
    ) -> list[tuple[float, float]]:
        """Plan a perimeter-orbit set of waypoints on the costmap's free area.

        Erodes the free-space mask by robot_radius_m so goals stay clear of
        walls, keeps the connected component containing the robot, walks the
        outer contour, and resamples it at ~spacing_m. Returns world (x, y)
        tuples rotated so the closest waypoint is first.
        """
        grid = costmap.grid
        res = float(costmap.resolution)

        # Free mask: 0..49 = free, 50..100 = occupied, -1 = unknown (treat as
        # obstacle so the orbit doesn't drift into unmapped space).
        free = ((grid >= 0) & (grid < 50)).astype(np.uint8) * 255
        if not free.any():
            return []

        # Erode by robot radius so the orbit stays clear of obstacles.
        radius_cells = max(1, math.ceil(robot_radius_m / res))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * radius_cells + 1, 2 * radius_cells + 1)
        )
        safe = cv2.erode(free, kernel)
        if not safe.any():
            return []

        # Keep the connected component containing the robot.
        rg = costmap.world_to_grid(Vector3(robot_xy[0], robot_xy[1], 0.0))
        rx, ry = round(rg.x), round(rg.y)
        n_labels, labels = cv2.connectedComponents(safe, connectivity=8)
        if not (0 <= ry < labels.shape[0] and 0 <= rx < labels.shape[1]):
            return []
        robot_label = int(labels[ry, rx])
        if robot_label == 0:
            # Robot is inside an obstacle / outside the safe zone — fall back to
            # the largest free component (the room).
            biggest, biggest_size = 0, 0
            for lbl in range(1, n_labels):
                size = int(np.count_nonzero(labels == lbl))
                if size > biggest_size:
                    biggest, biggest_size = lbl, size
            if biggest == 0:
                return []
            robot_label = biggest
        component = (labels == robot_label).astype(np.uint8) * 255

        # Outer contour (largest by area; RETR_EXTERNAL ignores interior holes).
        contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return []
        contour = max(contours, key=cv2.contourArea)
        pts = contour.reshape(-1, 2)  # each row = [col(x), row(y)] in grid space
        if len(pts) < 2:
            return []

        # Resample along the contour at ~spacing_m.
        spacing_cells = max(1, round(spacing_m / res))
        sampled = pts[::spacing_cells]
        if len(sampled) > max_waypoints:
            step = max(1, len(sampled) // max_waypoints)
            sampled = sampled[::step]

        # Grid → world.
        waypoints: list[tuple[float, float]] = []
        for p in sampled:
            w = costmap.grid_to_world(Vector3(float(p[0]), float(p[1]), 0.0))
            waypoints.append((float(w.x), float(w.y)))

        # Rotate so the closest waypoint to the robot is first.
        if waypoints:
            nearest_idx = min(
                range(len(waypoints)),
                key=lambda i: (waypoints[i][0] - robot_xy[0]) ** 2
                + (waypoints[i][1] - robot_xy[1]) ** 2,
            )
            waypoints = waypoints[nearest_idx:] + waypoints[:nearest_idx]

        return waypoints

    def _dispatch_capture(
        self,
        run: str,
        position: int,
        image_index: int,
        angle_deg: int,
        note: str,
        min_sharpness: float = 0.0,
    ) -> bool:
        """Snapshot the latest frame/pose now and upload it (fire-and-forget).

        Tagged with run / position / angle (heading) for grouping; `_upload_frame`
        also attaches the world pose (poseX/poseY), which is what the
        reconstruction pipeline cares about. Background upload keeps the capture
        loop from stalling on I/O.

        If `min_sharpness > 0`, runs a Laplacian-variance check on the frame
        first and skips upload (returning False) if it's below threshold —
        prevents motion-blurred frames during fast pan/turn from poisoning the
        reconstruction. A reasonable starting threshold is ~30 for a typical
        webcam-quality stream; tune up if blur is still slipping through.
        Returns True on dispatched, False if skipped.
        """
        frame = getattr(self, "_latest", None)
        if frame is None:
            return False

        if min_sharpness > 0.0:
            try:
                img = frame.data
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
                sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            except Exception:
                logger.exception("sharpness check failed; uploading anyway")
                sharpness = float("inf")
            if sharpness < min_sharpness:
                logger.info(
                    "vr_scan: skipped blurry frame (sharpness=%.1f < %.1f)",
                    sharpness, min_sharpness,
                )
                return False

        pose = getattr(self, "_pose", None)
        extra = {
            "run": run,
            "position": str(position),
            "angle": str(int(angle_deg)),
            "imageIndex": str(image_index),
        }
        label = f"{run}/pos{position:02d}/{int(angle_deg):03d}"

        def _bg() -> None:
            try:
                self._upload_frame(frame, pose, note=note, label=label, extra=extra)
            except Exception:
                logger.exception("vr_scan upload failed")

        t = threading.Thread(target=_bg, daemon=True, name="vr-scan-upload")
        with self._uploads_lock:
            self._uploads = [u for u in self._uploads if u.is_alive()]
            self._uploads.append(t)
        t.start()
        return True


def _nearest_waypoint_idx(
    xy: tuple[float, float], waypoints: list[tuple[float, float]]
) -> int:
    """Index of the waypoint closest to `xy`. Used for `position` tagging so
    frames captured during transit get bucketed by the physical zone they were
    taken in, rather than by the next-goal index they were heading toward.
    """
    return min(
        range(len(waypoints)),
        key=lambda i: (waypoints[i][0] - xy[0]) ** 2 + (waypoints[i][1] - xy[1]) ** 2,
    )
