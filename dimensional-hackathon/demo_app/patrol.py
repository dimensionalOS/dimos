from __future__ import annotations

import asyncio
import logging
import math
from typing import Any

from demo_app.types import Pose, Waypoint

logger = logging.getLogger(__name__)


class PatrolController:
    SPEED_MPS = 0.5

    def __init__(
        self,
        runner,
        waypoints: list[Waypoint],
        web_state: dict[str, Any],
        loop_forever: bool = True,
        scan_turns: int = 4,
        scan_pause_sec: float = 1.0,
        motion_settle_sec: float = 0.05,
        forward_steps_per_cycle: int = 3,
        forward_speed_mps: float = 0.30,
        forward_step_duration_sec: float = 1.25,
        sweep_yaw_radps: float = 0.24,
        sweep_turn_duration_sec: float = 0.28,
    ) -> None:
        self._runner = runner
        self._waypoints = waypoints
        self._web_state = web_state
        self._loop_forever = loop_forever
        self._scan_turns = scan_turns
        self._scan_pause_sec = scan_pause_sec
        self._motion_settle_sec = motion_settle_sec
        self._forward_steps_per_cycle = forward_steps_per_cycle
        self._forward_speed_mps = forward_speed_mps
        self._forward_step_duration_sec = forward_step_duration_sec
        self._sweep_yaw_radps = sweep_yaw_radps
        self._sweep_turn_duration_sec = sweep_turn_duration_sec
        self._task: asyncio.Task | None = None
        self._stop_requested = False
        self._return_home_on_stop = False
        self._current_wp: str | None = None
        self._web_state["patrol_phase"] = "IDLE"

    async def start(self) -> None:
        if self._task is not None and not self._task.done():
            return
        self._stop_requested = False
        self._return_home_on_stop = False
        self._task = asyncio.create_task(self._run())

    async def stop(self, return_home: bool = False) -> None:
        self._stop_requested = True
        self._return_home_on_stop = return_home
        task = self._task
        if task is not None:
            await task

    async def move_relative(self, forward: float, left: float, degrees: float) -> bool:
        pose = self._runner.get_pose()
        yaw_rad = math.radians(pose.yaw_deg)
        cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
        target_x = pose.x + cos_y * forward - sin_y * left
        target_y = pose.y + sin_y * forward + cos_y * left
        target_yaw = pose.yaw_deg + degrees
        dist = math.hypot(target_x - pose.x, target_y - pose.y)
        duration = max(0.75, dist / self.SPEED_MPS) if dist > 0.05 else max(0.75, abs(degrees) / 90.0)
        self._web_state["planned_path"] = [[pose.x, pose.y], [target_x, target_y]]
        self._push_event(
            "move",
            {"forward": forward, "left": left, "degrees": degrees},
        )
        return await asyncio.to_thread(
            self._runner.move_to,
            target_x,
            target_y,
            target_yaw,
            duration,
            lambda: self._stop_requested,
        )

    async def manual_drive(
        self,
        *,
        forward_mps: float = 0.0,
        left_mps: float = 0.0,
        yaw_radps: float = 0.0,
        duration_sec: float = 0.75,
    ) -> bool:
        self._push_event(
            "move",
            {
                "forward_mps": forward_mps,
                "left_mps": left_mps,
                "yaw_radps": yaw_radps,
                "duration_sec": duration_sec,
            },
        )
        return await asyncio.to_thread(
            self._runner.drive_for_duration,
            forward_mps,
            left_mps,
            yaw_radps,
            duration_sec,
        )

    async def status(self) -> str:
        mode = self._web_state.get("mode", "IDLE")
        visited = len(self._web_state.get("visited", []))
        current = self._current_wp or "-"
        return f"mode={mode} current={current} visited={visited}"

    def is_running(self) -> bool:
        return self._task is not None and not self._task.done()

    def current_waypoint(self) -> str:
        return self._current_wp or ""

    async def _run(self) -> None:
        self._set_mode("PATROLLING")
        self._web_state["visited"] = []
        self._push_event("state", {"mode": "PATROLLING"})

        try:
            cycle_index = 0
            while not self._stop_requested:
                cycle_index += 1
                self._current_wp = f"AISLE_{cycle_index}"
                self._web_state["planned_path"] = []

                for step_idx in range(self._forward_steps_per_cycle):
                    if self._stop_requested:
                        break
                    await self._forward_step(step_idx + 1)

                if self._stop_requested:
                    break

                await self._scan_in_place()

                if not self._loop_forever:
                    break

            if self._return_home_on_stop:
                self._set_mode("RETURNING")
                self._push_event("state", {"mode": "RETURNING"})
                await self._return_home()
        finally:
            self._current_wp = None
            self._web_state["planned_path"] = []
            self._set_mode("IDLE")
            self._set_phase("IDLE")
            self._push_event("state", {"mode": "IDLE"})

    async def _move_to_waypoint(self, waypoint: Waypoint) -> bool:
        pose = self._runner.get_pose()
        self._web_state["planned_path"] = [[pose.x, pose.y], [waypoint.pos[0], waypoint.pos[1]]]
        dist = math.hypot(waypoint.pos[0] - pose.x, waypoint.pos[1] - pose.y)
        duration = max(1.0, dist / self.SPEED_MPS)
        logger.info("Moving to %s", waypoint.id)
        return await asyncio.to_thread(
            self._runner.move_to,
            waypoint.pos[0],
            waypoint.pos[1],
            waypoint.yaw_deg,
            duration,
            lambda: self._stop_requested,
        )

    async def _scan_in_place(self) -> None:
        if self._scan_turns <= 0:
            return

        for _ in range(self._scan_turns):
            for phase_name, yaw_radps, duration_sec in (
                ("SWEEP_RIGHT", -self._sweep_yaw_radps, self._sweep_turn_duration_sec),
                ("CENTER_FROM_RIGHT", self._sweep_yaw_radps, self._sweep_turn_duration_sec),
                ("SWEEP_LEFT", self._sweep_yaw_radps, self._sweep_turn_duration_sec),
                ("CENTER_FROM_LEFT", -self._sweep_yaw_radps, self._sweep_turn_duration_sec),
            ):
                if self._stop_requested:
                    return
                self._set_phase(phase_name)
                await self.manual_drive(yaw_radps=yaw_radps, duration_sec=duration_sec)
                await self._settle_motion()
                await asyncio.sleep(self._scan_pause_sec)
        self._set_phase("PATROLLING")

    async def _forward_step(self, step_number: int) -> None:
        if self._stop_requested:
            return
        self._set_phase("FORWARD_STEP")
        pose = self._runner.get_pose()
        yaw_rad = math.radians(pose.yaw_deg)
        step_distance = self._forward_speed_mps * self._forward_step_duration_sec
        target_x = pose.x + math.cos(yaw_rad) * step_distance
        target_y = pose.y + math.sin(yaw_rad) * step_distance
        self._web_state["planned_path"] = [[pose.x, pose.y], [target_x, target_y]]
        await self.manual_drive(
            forward_mps=self._forward_speed_mps,
            duration_sec=self._forward_step_duration_sec,
        )
        await self._settle_motion()
        await asyncio.sleep(0.20)
        self._set_phase("PATROLLING")

    async def _return_home(self) -> None:
        pose = self._runner.get_pose()
        self._web_state["planned_path"] = [[pose.x, pose.y], [0.0, 0.0]]
        await asyncio.to_thread(self._runner.move_to, 0.0, 0.0, 0.0, 5.0, lambda: self._stop_requested)

    def _set_mode(self, mode: str) -> None:
        self._web_state["mode"] = mode

    def _set_phase(self, phase: str) -> None:
        self._web_state["patrol_phase"] = phase

    def _push_event(self, event_type: str, payload: dict[str, Any]) -> None:
        self._web_state.setdefault("event_log", []).append({"type": event_type, "payload": payload})

    async def _settle_motion(self) -> None:
        if self._stop_requested or self._motion_settle_sec <= 0:
            return
        await self.manual_drive(duration_sec=self._motion_settle_sec)


def nearest_waypoint(pose: Pose, waypoints: list[Waypoint]) -> str:
    best_id = ""
    best_dist = float("inf")
    for waypoint in waypoints:
        dist = (pose.x - waypoint.pos[0]) ** 2 + (pose.y - waypoint.pos[1]) ** 2
        if dist < best_dist:
            best_dist = dist
            best_id = waypoint.id
    return best_id
