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

"""One host's explicit diagnostic session; served through the existing DimOS gateway."""

from __future__ import annotations

import fcntl
import json
from pathlib import Path
import re
import time
from typing import Any, Literal
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field, field_validator

from dimos.diagnostics.store import DiagnosticsStore, validate_run_id

SESSION_SCHEMA = "dimos.diagnostics.session/v1"


class NetworkSettings(BaseModel):
    model_config = ConfigDict(extra="forbid")
    interface: str = Field(min_length=1, max_length=64)
    peers: list[str] = Field(default_factory=list, max_length=8)
    interval_s: float = Field(default=2, ge=1, le=60)

    @field_validator("interface")
    @classmethod
    def valid_interface(cls, value: str) -> str:
        if not re.fullmatch(r"[A-Za-z0-9_.:-]+", value) or value.startswith("-"):
            raise ValueError("invalid interface")
        return value

    @field_validator("peers")
    @classmethod
    def valid_peers(cls, values: list[str]) -> list[str]:
        for value in values:
            if not re.fullmatch(r"[A-Za-z0-9_.:-]{1,253}", value) or value.startswith("-"):
                raise ValueError("invalid peer address")
        return values


class SessionSettings(BaseModel):
    model_config = ConfigDict(extra="forbid")
    run_id: str
    mode: Literal["trace"] = "trace"
    network: NetworkSettings | None = None

    @field_validator("run_id")
    @classmethod
    def valid_run(cls, value: str) -> str:
        validate_run_id(value)
        return value


class Requirement(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str
    kind: Literal["python", "native", "gateway", "browser", "network"] | None = None
    event: str | None = None
    minimum: int = Field(default=1, ge=1, le=1024)
    max_age_s: float = Field(default=10, ge=1, le=120)


class SessionStore:
    def __init__(self, root: Path) -> None:
        self.store = DiagnosticsStore(root)
        self.root = self.store.root

    def active(self) -> dict[str, Any] | None:
        path = self.root / "session.json"
        if not path.exists():
            return None
        if path.is_symlink() or path.stat().st_size > 4096:
            raise ValueError("invalid session descriptor")
        value: dict[str, Any] = json.loads(path.read_text())
        SessionSettings.model_validate({k: value[k] for k in ("run_id", "mode")})
        if value.get("schema") != SESSION_SCHEMA:
            raise ValueError("incompatible session descriptor")
        return value

    def start(self, settings: SessionSettings) -> dict[str, Any]:
        self.root.mkdir(parents=True, exist_ok=True)
        with (self.root / ".session.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            active = self.active()
            if active is not None:
                if any(active.get(k) != v for k, v in settings.model_dump().items()):
                    raise ValueError("another diagnostics session is active; stop it first")
                return active
            # A stopped run is immutable: delayed browser retries must not enter a new session.
            run = self.store.run(settings.run_id)
            if (run / "session.json").exists():
                raise ValueError("run id already used; choose a new run id")
            run.mkdir(parents=True, exist_ok=True)
            value = {
                "schema": SESSION_SCHEMA,
                **settings.model_dump(),
                "started_ns": time.time_ns(),
                "generation": uuid4().hex,
            }
            data = json.dumps(value)
            (run / "session.json").write_text(data)
            temp = self.root / "session.tmp"
            temp.write_text(data)
            temp.replace(self.root / "session.json")
            return value

    def control(self, run_id: str, action: str, reason: str = "manual") -> int:
        validate_run_id(run_id)
        if action not in ("stop", "mark", "flush"):
            raise ValueError("invalid capture action")
        self.root.mkdir(parents=True, exist_ok=True)
        with (self.root / ".session.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            active = self.active()
            if active is None or active["run_id"] != run_id:
                raise ValueError("session is not active")
            count = self.store.request(run_id, action, reason)
            if action == "stop":
                (self.root / "session.json").unlink()
            return count

    def readiness(self, run_id: str, requirements: list[Requirement]) -> dict[str, Any]:
        now = time.time_ns()
        active = self.active()
        processes = self.store.statuses(run_id)
        artifacts = self.store.incidents(run_id)
        durable = {item["id"].split("/")[0] for item in artifacts if item["bytes"] > 0}
        checks = []
        for required in requirements:
            matched = []
            for process in processes:
                age = (now - process.get("at_ns", 0)) / 1e9
                kind = process.get("collector")
                if kind is None:
                    identity = process.get("process", "")
                    kind = (
                        "gateway"
                        if identity.startswith("gateway-")
                        else "native"
                        if identity.startswith("native-")
                        else "python"
                    )
                if (
                    process.get("state") != "collecting"
                    or process.get("run_id") != run_id
                    or age < 0
                    or age > required.max_age_s
                    or process.get("io_errors", 0)
                    or process.get("process") not in durable
                    or (required.kind and kind != required.kind)
                ):
                    continue
                if required.event:
                    observed = process.get("last_events", {}).get(required.event, 0)
                    if not 0 <= (now - observed) / 1e9 <= required.max_age_s:
                        continue
                matched.append(process["process"])
            checks.append(
                {
                    **required.model_dump(),
                    "observed": len(matched),
                    "processes": matched,
                    "ready": len(matched) >= required.minimum,
                }
            )
        enabled = active is not None and active["run_id"] == run_id
        return {
            "schema": SESSION_SCHEMA,
            "run_id": run_id,
            "active": enabled,
            "ready": enabled and bool(checks) and all(c["ready"] for c in checks),
            "checks": checks,
            "processes": processes,
            "incidents": len(artifacts),
            "coverage": "requested collectors and durable observations only; not latency qualification",
        }
