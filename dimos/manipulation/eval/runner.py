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

"""Skill-mode benchmark runner for the pick-and-place stack.

Drives ``PickAndPlaceModule`` over RPC (no agent): for each trial it runs
``go_init -> scan_objects -> pick -> place -> go_init`` and records one episode.

Portability is the key constraint: the *same* runner must work against both the
post-SkillResult stack (skills return ``SkillResult`` objects) and the pre-PR
stack (skills return plain ``str``). It therefore never imports ``skill_result``
or ``skill_errors`` and duck-types whatever a skill returns
(:func:`normalize_skill_result`). All RPC/hardware imports are lazy (inside
:meth:`BenchmarkRunner.connect`) so this module imports cleanly without hardware.
"""

from __future__ import annotations

from collections.abc import Callable
from datetime import datetime, timezone
import threading
import time
from typing import Any

from dimos.manipulation.eval import report
from dimos.manipulation.eval.recorder import EpisodeRecorder, infer_stages
from dimos.manipulation.eval.suite import OBJECT_CLASS_BY_NAME
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _now_iso() -> str:
    """UTC timestamp, millisecond precision, ``Z`` suffix (e.g. 2026-06-01T...Z)."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def normalize_skill_result(raw: object) -> dict[str, Any]:
    """Collapse any skill return shape into ``{success, error_code, message, duration_ms}``.

    Handles, in order: ``None`` (treated as failure), a ``SkillResult``-like
    object (duck-typed on ``success``/``error_code``/``message`` — never
    ``isinstance``, so it works pre-PR), a ``dict``, a pre-PR ``str`` (failure
    when it starts with ``"Error:"``), and a final truthiness fallback.

    ``duration_ms`` here is a placeholder; the runner overwrites it with its own
    ``perf_counter`` measurement so timing is uniform across stack versions.
    """
    if raw is None:
        return {
            "success": False,
            "error_code": "EXECUTION_FAILED",
            "message": "no result returned",
            "duration_ms": 0.0,
        }
    if hasattr(raw, "success") and hasattr(raw, "error_code") and hasattr(raw, "message"):
        return {
            "success": bool(raw.success),
            "error_code": getattr(raw, "error_code", None),
            "message": str(getattr(raw, "message", "")),
            "duration_ms": 0.0,
        }
    if isinstance(raw, dict):
        message = str(raw.get("message", ""))
        success = bool(raw.get("success", not message.startswith("Error:")))
        return {
            "success": success,
            "error_code": raw.get("error_code"),
            "message": message,
            "duration_ms": 0.0,
        }
    if isinstance(raw, str):
        return {
            "success": not raw.startswith("Error:"),
            "error_code": None,
            "message": raw,
            "duration_ms": 0.0,
        }
    return {"success": bool(raw), "error_code": None, "message": str(raw), "duration_ms": 0.0}


class BenchmarkRunner:
    """Runs benchmark trials against a live ``PickAndPlaceModule`` over RPC."""

    def __init__(
        self,
        recorder: EpisodeRecorder,
        hardware: str = "sim",
        place_z_default: float = 0.05,
        inter_episode_delay_s: float = 2.0,
        scan_warmup_s: float = 3.0,
    ) -> None:
        self._recorder = recorder
        self._hardware = hardware
        self._place_z_default = place_z_default
        self._inter_episode_delay_s = inter_episode_delay_s
        # Dwell after go_init (arm settled at the scan pose) before scan_objects so
        # perception accumulates enough frames to promote objects to permanent.
        self._scan_warmup_s = scan_warmup_s
        self._client: Any = None
        self._last_record: dict[str, Any] | None = None

    # -- connection ---------------------------------------------------------

    def connect(self, timeout_s: float = 10.0) -> None:
        """Create the RPC client and verify the server answers within ``timeout_s``.

        The RPC proxy has no per-call timeout and the default is 120s, so a dead
        server would block. We probe ``get_state()`` in a daemon thread and join
        with the deadline; if it does not return in time we raise ``RuntimeError``.
        Heavy imports happen here so the module stays import-clean without hardware.
        """
        from dimos.core.rpc_client import RPCClient
        from dimos.manipulation.pick_and_place_module import PickAndPlaceModule

        self._client = RPCClient(None, PickAndPlaceModule)

        probe: dict[str, Any] = {}

        def _probe() -> None:
            try:
                probe["state"] = self._client.get_state()
            except Exception as exc:
                probe["error"] = exc

        thread = threading.Thread(target=_probe, name="eval-readiness-probe", daemon=True)
        thread.start()
        thread.join(timeout_s)
        if thread.is_alive():
            raise RuntimeError(
                f"Manipulation server not ready: get_state() did not return within "
                f"{timeout_s:.0f}s. Is the sim running "
                f"('dimos run xarm-perception-sim-agent')?"
            )
        if "error" in probe:
            raise RuntimeError(f"Manipulation server probe failed: {probe['error']!r}")
        logger.info("Connected to manipulation server (state=%s)", probe.get("state"))

    def prepare(self, retries: int = 5) -> bool:
        """Stage the arm before a run: joint-space ``go_home`` (retried past RRT
        flakiness), then capture that pose as the ``go_init`` target.

        The sim arm can boot in a near-singular/low pose; ``scan_objects``->``go_init``
        otherwise returns to it and the Cartesian safety-lift fails to plan. Joint-space
        ``go_home`` plans reliably out of the singularity. Returns True once home is
        reached. On real hardware (arm already sane) it just re-homes once.
        """
        if self._client is None:
            raise RuntimeError("call connect() before prepare()")
        for attempt in range(1, retries + 1):
            result = normalize_skill_result(self._client.go_home())
            logger.info(
                "prepare: go_home attempt %d/%d -> success=%s (%s)",
                attempt,
                retries,
                result["success"],
                result["message"],
            )
            if result["success"]:
                try:
                    self._client.set_init_joints_to_current()
                except Exception:
                    logger.warning("prepare: set_init_joints_to_current failed", exc_info=True)
                return True
        logger.error("prepare: go_home did not succeed after %d attempts", retries)
        return False

    # -- single episode -----------------------------------------------------

    def _timed_call(self, fn: Callable[[], object]) -> dict[str, Any]:
        """Call a skill, measure wall-clock, and return the normalized result."""
        t0 = time.perf_counter()
        raw = fn()
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        result = normalize_skill_result(raw)
        result["duration_ms"] = round(elapsed_ms, 1)
        return result

    def _assemble(
        self,
        trial: dict[str, Any],
        mode: str,
        stages: dict[str, Any],
        pick: dict[str, Any] | None,
        place: dict[str, Any] | None,
        task_success: bool,
        cycle_ms: float,
    ) -> dict[str, Any]:
        target = list(trial["target_position"])
        object_name = trial["object_name"]
        return {
            "timestamp_iso": _now_iso(),
            "hardware": self._hardware,
            "mode": mode,
            "scene_id": trial["scene_id"],
            "object_name": object_name,
            "object_class": OBJECT_CLASS_BY_NAME.get(object_name, object_name),
            "target_position": [round(float(v), 4) for v in target],
            "stages": stages,
            "pick_result": pick,
            "place_result": place,
            "task_success": task_success,
            "cycle_time_ms": cycle_ms,
            "placement_error_m": None,
            "agent_calls": None,
            "agent_retries": None,
            "agent_first_skill_correct": None,
        }

    def run_episode(self, trial: dict[str, Any], mode: str = "skill") -> dict[str, Any]:
        """Run one trial and record one episode. Returns the recorded dict.

        On any uncaught exception the episode is recorded as a failed
        ``EXECUTION_FAILED`` episode and the exception is re-raised.
        """
        object_name = trial["object_name"]
        target = list(trial["target_position"])
        ep_t0 = time.perf_counter()
        scan_result: dict[str, Any] | None = None
        pick_result: dict[str, Any] | None = None
        place_result: dict[str, Any] | None = None
        try:
            # Clear the previous episode's perception obstacles so a placed object
            # left in the planning world can't cause COLLISION_AT_START. Best-effort.
            try:
                self._client.clear_perception_obstacles()
            except Exception:
                logger.warning("clear_perception_obstacles failed", exc_info=True)
            self._client.go_init()
            if self._scan_warmup_s > 0:
                time.sleep(self._scan_warmup_s)
            scan_result = self._timed_call(self._client.scan_objects)
            if scan_result["success"]:
                pick_result = self._timed_call(lambda: self._client.pick(object_name))
                if pick_result["success"]:
                    x, y = float(target[0]), float(target[1])
                    z = float(target[2]) if len(target) >= 3 else self._place_z_default
                    place_result = self._timed_call(lambda: self._client.place(x, y, z))

            # Return to a safe pose; never let cleanup fail the episode.
            try:
                self._client.go_init()
            except Exception:
                logger.warning("go_init cleanup failed", exc_info=True)

            stages = infer_stages(pick_result, place_result, scan_result)
            task_success = bool(place_result and place_result["success"])
            cycle_ms = round((time.perf_counter() - ep_t0) * 1000.0, 1)
            episode = self._assemble(
                trial, mode, stages, pick_result, place_result, task_success, cycle_ms
            )
            self._last_record = self._recorder.record(episode)
            return self._last_record
        except Exception:
            cycle_ms = round((time.perf_counter() - ep_t0) * 1000.0, 1)
            failure = {
                "success": False,
                "error_code": "EXECUTION_FAILED",
                "message": "uncaught harness exception",
                "duration_ms": cycle_ms,
            }
            stages = infer_stages(failure, None, scan_result)
            episode = self._assemble(trial, mode, stages, failure, None, False, cycle_ms)
            self._last_record = self._recorder.record(episode)
            raise

    # -- suite --------------------------------------------------------------

    @staticmethod
    def _progress_line(index: int, total: int, episode: dict[str, Any]) -> str:
        if episode.get("task_success"):
            status = "PASS"
        else:
            status = f"FAIL {report.episode_error_code(episode) or 'UNKNOWN'}"
        return (
            f"[ep {index}/{total}] {episode.get('object_name')} | "
            f"{episode.get('scene_id')} | {status} | {episode.get('cycle_time_ms'):.0f}ms"
        )

    def run_suite(
        self, trials: list[dict[str, Any]], n_repeats: int = 1, mode: str = "skill"
    ) -> list[dict[str, Any]]:
        """Run every trial ``n_repeats`` times, recording and reporting each episode."""
        all_trials = list(trials) * n_repeats
        total = len(all_trials)
        episodes: list[dict[str, Any]] = []
        for index, trial in enumerate(all_trials, start=1):
            if index > 1 and self._inter_episode_delay_s > 0:
                time.sleep(self._inter_episode_delay_s)
            try:
                episode = self.run_episode(trial, mode=mode)
            except Exception as exc:
                logger.error("episode %d raised (recorded as EXECUTION_FAILED): %s", index, exc)
                episode = self._last_record or self._assemble(
                    trial, mode, infer_stages(None, None, None), None, None, False, 0.0
                )
            episodes.append(episode)
            print(self._progress_line(index, total, episode))
        self._print_summary(episodes)
        return episodes

    @staticmethod
    def _print_summary(episodes: list[dict[str, Any]]) -> None:
        total = len(episodes)
        passed = sum(1 for ep in episodes if ep.get("task_success"))
        distribution = report.error_code_distribution(episodes)
        top = max(distribution.items(), key=lambda kv: kv[1])[0] if distribution else "-"
        print(
            f"\nSummary: {total} episodes | {passed} pass | {total - passed} fail "
            f"| top error: {top}"
        )
