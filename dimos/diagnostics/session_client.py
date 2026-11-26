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

"""Coordinate existing authenticated host gateways; no SSH, daemon or robot commands."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from pydantic import BaseModel, ConfigDict, Field, field_validator
import requests

from dimos.diagnostics.session import NetworkSettings, Requirement, SessionSettings
from dimos.diagnostics.store import validate_run_id


class SessionHost(BaseModel):
    model_config = ConfigDict(extra="forbid")
    url: str
    required: list[Requirement] = Field(min_length=1, max_length=128)
    token_env: str | None = None
    network: NetworkSettings | None = None

    @field_validator("url")
    @classmethod
    def valid_url(cls, value: str) -> str:
        url = urlsplit(value)
        if (
            url.scheme not in ("http", "https")
            or not url.hostname
            or url.username
            or url.password
            or url.query
            or url.fragment
        ):
            raise ValueError("use an HTTP(S) diagnostics URL without credentials or query")
        return value.rstrip("/")


class SessionPlan(SessionSettings):
    hosts: dict[str, SessionHost] = Field(min_length=1, max_length=16)

    @field_validator("hosts")
    @classmethod
    def names(cls, value: dict[str, SessionHost]) -> dict[str, SessionHost]:
        for name in value:
            validate_run_id(name)
        if len({host.url for host in value.values()}) != len(value):
            raise ValueError("each host endpoint must be unique")
        return value


class SessionClient:
    def __init__(self, plan: SessionPlan, headers: dict[str, dict[str, str]] | None = None) -> None:
        self.plan = plan
        self.headers = headers or {}

    def request(self, host: str, method: str, path: str, **kwargs: Any) -> Any:
        with requests.request(
            method,
            self.plan.hosts[host].url + path,
            headers=self.headers.get(host, {}),
            timeout=(3, 10),
            **kwargs,
        ) as response:
            response.raise_for_status()
            return response.json()

    def each(self, method: str, path: str, bodies: dict[str, Any] | None = None) -> dict[str, Any]:
        def call(name: str) -> tuple[str, Any]:
            try:
                return name, self.request(
                    name, method, path, **({"json": bodies[name]} if bodies else {})
                )
            except (requests.RequestException, ValueError) as error:
                # Never echo authorization headers or a server's HTML error page.
                return name, {
                    "error": type(error).__name__,
                    "status": getattr(getattr(error, "response", None), "status_code", None),
                }

        with ThreadPoolExecutor(max_workers=len(self.plan.hosts)) as pool:
            return dict(pool.map(call, self.plan.hosts))

    def start(self) -> dict[str, Any]:
        # Preflight every endpoint before changing any. A partial failure is reported,
        # never erased by starting the blueprint anyway.
        preflight = self.each("GET", "/session")
        if any(
            "error" in result
            or result.get("schema") != "dimos.diagnostics.session/v1"
            or (result.get("session") and result["session"]["run_id"] != self.plan.run_id)
            for result in preflight.values()
        ):
            return {"started": False, "hosts": preflight}
        body = {"run_id": self.plan.run_id, "mode": self.plan.mode}
        results = self.each(
            "PUT",
            "/session",
            {
                name: {**body, "network": host.network.model_dump() if host.network else None}
                for name, host in self.plan.hosts.items()
            },
        )
        return {
            "started": all("error" not in result for result in results.values()),
            "hosts": results,
        }

    def status(self) -> dict[str, Any]:
        results = self.each(
            "POST",
            f"/{self.plan.run_id}/readiness",
            {
                name: {"requirements": [r.model_dump() for r in host.required]}
                for name, host in self.plan.hosts.items()
            },
        )
        return {
            "run_id": self.plan.run_id,
            "ready": all(result.get("ready", False) for result in results.values()),
            "hosts": results,
        }

    def control(self, action: str, reason: str = "manual") -> dict[str, Any]:
        return self.each(
            "POST",
            f"/{self.plan.run_id}/control",
            {name: {"action": action, "reason": reason} for name in self.plan.hosts},
        )

    def collect(self, destination: Path, budget: int = 1024**3) -> dict[str, Any]:
        if destination.exists():
            raise ValueError("bundle destination already exists")
        destination.mkdir(parents=True)
        result: dict[str, Any] = {"run_id": self.plan.run_id, "hosts": {}, "bytes": 0}
        for name in self.plan.hosts:
            host_result: dict[str, Any] = {"files": [], "errors": []}
            result["hosts"][name] = host_result
            try:
                incidents = self.request(name, "GET", f"/{self.plan.run_id}/incidents")
                for incident in incidents:
                    process, filename = incident["id"].split("/")
                    validate_run_id(process)
                    if not filename.startswith("incident-") or not filename.endswith(".jsonl"):
                        raise ValueError("invalid incident name")
                    validate_run_id(filename[9:-6])
                    target = destination / name / process / filename
                    target.parent.mkdir(parents=True, exist_ok=True)
                    url = (
                        self.plan.hosts[name].url
                        + f"/{self.plan.run_id}/incidents/{process}/{filename}"
                    )
                    with requests.get(
                        url, headers=self.headers.get(name, {}), stream=True, timeout=(3, 10)
                    ) as response:
                        response.raise_for_status()
                        temporary = target.with_suffix(".partial")
                        try:
                            with temporary.open("xb") as output:
                                size = 0
                                for chunk in response.iter_content(65536):
                                    size += len(chunk)
                                    result["bytes"] += len(chunk)
                                    if size > 80 * 1024 * 1024 or result["bytes"] > budget:
                                        raise ValueError("bundle exceeds collection budget")
                                    output.write(chunk)
                            with temporary.open() as source:
                                manifest = json.loads(source.readline(65536))
                            if (
                                manifest.get("run_id") != self.plan.run_id
                                or manifest.get("process") != process
                            ):
                                raise ValueError("incident identity mismatch")
                            temporary.replace(target)
                            host_result["files"].append(incident["id"])
                        finally:
                            temporary.unlink(missing_ok=True)
            except (requests.RequestException, ValueError, OSError, KeyError) as error:
                host_result["errors"].append(type(error).__name__)
        result["complete"] = all(h["files"] and not h["errors"] for h in result["hosts"].values())
        (destination / "collection.json").write_text(json.dumps(result, indent=2))
        (destination / "readiness.json").write_text(json.dumps(self.status(), indent=2))
        return result
