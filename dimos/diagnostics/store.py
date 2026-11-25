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

"""Filesystem-backed diagnostics API; callers cannot choose paths outside its root."""

from __future__ import annotations

import json
from pathlib import Path
import re
from typing import Any


def validate_run_id(run_id: str) -> None:
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", run_id):
        raise ValueError("invalid diagnostics run id")


class DiagnosticsStore:
    def __init__(self, root: Path) -> None:
        self.root = root.expanduser().resolve()

    def run(self, run_id: str) -> Path:
        validate_run_id(run_id)
        path = (self.root / run_id).resolve()
        if not path.is_relative_to(self.root):
            raise ValueError("diagnostics path escapes configured root")
        return path

    def statuses(self, run_id: str) -> list[dict[str, Any]]:
        result = []
        for path in sorted(self.run(run_id).glob("*/status.json")):
            if path.resolve().is_relative_to(self.root) and path.stat().st_size <= 4 * 1024 * 1024:
                result.append(json.loads(path.read_text()))
        return result

    def incident(self, run_id: str, process: str, incident: str) -> Path:
        validate_run_id(process)
        if not re.fullmatch(r"incident-[A-Za-z0-9_-]+\.jsonl", incident):
            raise ValueError("invalid incident id")
        run = self.run(run_id)
        path = (run / process / incident).resolve()
        if not path.is_relative_to(run):
            raise ValueError("incident escapes run")
        if not path.is_file():
            raise FileNotFoundError(incident)
        if path.stat().st_size > 80 * 1024 * 1024:
            raise ValueError("incident exceeds read budget")
        return path

    def incidents(self, run_id: str) -> list[dict[str, Any]]:
        return [
            {"id": str(path.relative_to(self.run(run_id))), "bytes": path.stat().st_size}
            for path in sorted(self.run(run_id).glob("*/incident-*.jsonl"))
            if path.resolve().is_relative_to(self.root)
        ]

    def request(self, run_id: str, action: str, reason: str = "manual") -> int:
        if action not in ("stop", "mark", "flush"):
            raise ValueError("unknown capture action")
        count = 0
        for path in self.run(run_id).glob("*/status.json"):
            if not path.resolve().is_relative_to(self.root):
                continue
            tmp = path.parent / "control.tmp"
            tmp.write_text(json.dumps({"action": action, "reason": reason[:512]}))
            tmp.replace(path.parent / "control.json")
            count += 1
        return count
