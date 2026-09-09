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

"""Idle scheduler: when nothing has happened for a while, wake the agent with a task."""

import asyncio
from pathlib import Path
import time
from typing import Any

import yaml

from .events import EventBus
from .store import Store

CONFIG_PATH = Path(__file__).resolve().parent.parent / "tasks.yaml"
TASK_TIMEOUT_S = 300.0
DAY_S = 86400.0
TICK_S = 1.0

DEFAULTS: dict[str, Any] = {
    "idle_after_s": 300,
    "cooldown_after_wake_s": 180,
    # a person coming back into view is only news after this long away
    "seen_event_gap_s": 600,
    "tasks": {
        "follow_up": {
            "chat_age_min_s": 1200,
            "chat_age_max_s": 10800,
            "max_per_person_per_day": 2,
            "min_gap_per_person_s": 2700,
        },
        "greet_known": {
            "seen_within_s": 120,
            "no_chat_for_s": 1800,
            "max_per_person_per_day": 2,
        },
        "explore": {},
    },
}


class Config:
    """tasks.yaml, reloaded whenever the file changes on disk."""

    def __init__(self, path: Path = CONFIG_PATH) -> None:
        self.path = path
        self._mtime = -1.0
        self.values = dict(DEFAULTS)
        self.reload()

    def reload(self) -> None:
        try:
            mtime = self.path.stat().st_mtime
        except OSError:
            return
        if mtime == self._mtime:
            return
        loaded = yaml.safe_load(self.path.read_text()) or {}
        merged = dict(DEFAULTS)
        merged.update({k: v for k, v in loaded.items() if k != "tasks"})
        merged["tasks"] = {**DEFAULTS["tasks"], **(loaded.get("tasks") or {})}
        self.values = merged
        self._mtime = mtime

    def __getitem__(self, key: str) -> Any:
        return self.values[key]

    def task(self, name: str) -> dict[str, Any]:
        return self.values["tasks"].get(name) or {}


class Scheduler:
    def __init__(self, store: Store, bus: EventBus, config: Config | None = None) -> None:
        self.store = store
        self.bus = bus
        self.config = config or Config()
        self.last_activity = time.time()
        self.last_wake = 0.0
        self._next_id = self._highest_task_number() + 1

    # --- activity -------------------------------------------------------

    def touch(self) -> None:
        """Any phone message, agent write, or delivered event counts as activity."""
        self.last_activity = time.time()

    # --- open task bookkeeping -----------------------------------------

    def open_task(self) -> Any:
        return self.store.one("SELECT * FROM tasks WHERE closed_at IS NULL ORDER BY created_at")

    def close_task(self, task_id: str, outcome: str, note: str | None = None) -> bool:
        row = self.store.one("SELECT * FROM tasks WHERE task_id = ?", (task_id,))
        if row is None:
            return False
        self.store.write(
            "UPDATE tasks SET closed_at = ?, outcome = ?, note = ? WHERE task_id = ?",
            (time.time(), outcome, note, task_id),
        )
        return True

    def _highest_task_number(self) -> int:
        best = 0
        for row in self.store.query("SELECT task_id FROM tasks"):
            tail = str(row["task_id"]).removeprefix("t_")
            if tail.isdigit():
                best = max(best, int(tail))
        return best

    # --- the loop -------------------------------------------------------

    async def run(self) -> None:
        while True:
            try:
                self.tick()
            except Exception as exc:  # a bad tasks.yaml must not kill the server
                print(f"[scheduler] {exc}")
            await asyncio.sleep(TICK_S)

    def tick(self) -> None:
        self.config.reload()
        now = time.time()

        open_task = self.open_task()
        if open_task is not None:
            if now - open_task["created_at"] > TASK_TIMEOUT_S:
                self.close_task(open_task["task_id"], "not_found", "timed out")
            return

        if now - self.last_activity < self.config["idle_after_s"]:
            return
        if now - self.last_wake < self.config["cooldown_after_wake_s"]:
            return

        wake = self.pick_task(now)
        if wake is None:
            return
        self.store.write(
            "INSERT INTO tasks (task_id, task, person_id, created_at) VALUES (?, ?, ?, ?)",
            (wake["task_id"], wake["task"], wake.get("person_id"), now),
        )
        self.last_wake = now
        self.bus.publish(wake)

    def pick_task(self, now: float) -> dict[str, Any] | None:
        people = self.store.people_overview()
        return self._follow_up(now, people) or self._greet_known(now, people) or self._explore()

    def _new_id(self) -> str:
        task_id = f"t_{self._next_id}"
        self._next_id += 1
        return task_id

    def _count_today(self, person_id: str, task: str, now: float) -> int:
        row = self.store.one(
            "SELECT COUNT(*) AS n FROM tasks WHERE person_id = ? AND task = ? AND created_at > ?",
            (person_id, task, now - DAY_S),
        )
        return row["n"] if row else 0

    def _last_task_ts(self, person_id: str, task: str) -> float:
        row = self.store.one(
            "SELECT MAX(created_at) AS ts FROM tasks WHERE person_id = ? AND task = ?",
            (person_id, task),
        )
        return (row["ts"] if row and row["ts"] else 0.0) or 0.0

    def _follow_up(self, now: float, people: list[dict[str, Any]]) -> dict[str, Any] | None:
        cfg = self.config.task("follow_up")
        for p in people:
            if not p["last_chat_ts"]:
                continue
            age = now - p["last_chat_ts"]
            if not cfg.get("chat_age_min_s", 0) <= age <= cfg.get("chat_age_max_s", 0):
                continue
            if self._count_today(p["person_id"], "follow_up", now) >= cfg.get(
                "max_per_person_per_day", 0
            ):
                continue
            if now - self._last_task_ts(p["person_id"], "follow_up") < cfg.get(
                "min_gap_per_person_s", 0
            ):
                continue
            return {
                "type": "wake",
                "task_id": self._new_id(),
                "task": "follow_up",
                "person_id": p["person_id"],
                "name": p["name"],
                "last_chat_minutes_ago": round(age / 60),
                "last_seen_pose": p["last_seen_pose"],
                "history": self.store.last_messages(p["person_id"], 6),
            }
        return None

    def _greet_known(self, now: float, people: list[dict[str, Any]]) -> dict[str, Any] | None:
        cfg = self.config.task("greet_known")
        for p in people:
            if not p["last_seen_ts"]:
                continue
            if now - p["last_seen_ts"] > cfg.get("seen_within_s", 0):
                continue
            last_chat = p["last_chat_ts"] or 0.0
            if now - last_chat < cfg.get("no_chat_for_s", 0):
                continue
            if self._count_today(p["person_id"], "greet_known", now) >= cfg.get(
                "max_per_person_per_day", 0
            ):
                continue
            return {
                "type": "wake",
                "task_id": self._new_id(),
                "task": "greet_known",
                "person_id": p["person_id"],
                "name": p["name"],
                "last_seen_pose": p["last_seen_pose"],
            }
        return None

    def _explore(self) -> dict[str, Any] | None:
        if "explore" not in self.config["tasks"]:
            return None
        return {"type": "wake", "task_id": self._new_id(), "task": "explore"}
