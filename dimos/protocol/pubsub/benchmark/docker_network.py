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

"""Docker bridge and symmetric network-impairment lifecycle."""

from __future__ import annotations

from contextlib import AbstractContextManager
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
from typing import Any, cast

from dimos.protocol.pubsub.benchmark.model import NETWORK_CONDITIONS, TrialSpec

DEFAULT_BENCHMARK_IMAGE = "dimos-transport-benchmark:local"
PUBLISHER_ADDRESS = "10.88.0.10"
SUBSCRIBER_ADDRESS = "10.88.0.20"
_LCM_RECEIVE_BUFFER_BYTES = "67108864"
_LINK_MTU_BYTES = "1500"


def _run(argv: list[str], *, timeout: float = 120.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(argv, check=True, capture_output=True, text=True, timeout=timeout)


class DockerNetworkTrial(AbstractContextManager["DockerNetworkTrial"]):
    """Own two endpoint containers and their dedicated impaired bridge."""

    def __init__(
        self, spec: TrialSpec, work_dir: Path, image: str = DEFAULT_BENCHMARK_IMAGE
    ) -> None:
        self.spec = spec
        self.work_dir = work_dir.resolve()
        self.image = image
        identity = hashlib.sha256(str(self.work_dir).encode()).hexdigest()[:12]
        self.network_name = f"dimos-bench-{identity}"
        self.publisher_container = f"{self.network_name}-publisher"
        self.subscriber_container = f"{self.network_name}-subscriber"

    def preflight(self) -> None:
        if platform.system() != "Linux":
            raise RuntimeError("The emulated transport benchmark requires Linux")
        if shutil.which("docker") is None:
            raise RuntimeError("Docker is not installed or is not on PATH")
        _run(["docker", "info", "--format", "{{.OSType}}"])
        _run(["docker", "image", "inspect", self.image])

    def _run_endpoint(self, name: str, address: str) -> None:
        _run(
            [
                "docker",
                "run",
                "--detach",
                "--name",
                name,
                "--network",
                self.network_name,
                "--ip",
                address,
                "--cap-add",
                "NET_ADMIN",
                "--mount",
                f"type=bind,source={self.work_dir},target=/results",
                self.image,
                "sleep",
                "infinity",
            ],
            timeout=300.0,
        )
        _run(
            [
                "docker",
                "exec",
                name,
                "sysctl",
                "-w",
                f"net.core.rmem_max={_LCM_RECEIVE_BUFFER_BYTES}",
                f"net.core.rmem_default={_LCM_RECEIVE_BUFFER_BYTES}",
            ]
        )

    @staticmethod
    def _netem_args(spec: TrialSpec) -> list[str]:
        conditions = NETWORK_CONDITIONS[spec.profile]
        argv = ["netem"]
        if conditions.delay_ms:
            argv.extend(["delay", f"{conditions.delay_ms:g}ms", f"{conditions.jitter_ms:g}ms"])
        if conditions.loss_pct:
            argv.extend(["loss", "random", f"{conditions.loss_pct:g}%"])
            if conditions.loss_correlation_pct:
                argv.append(f"{conditions.loss_correlation_pct:g}%")
        if conditions.rate_mbit_s:
            argv.extend(["rate", f"{conditions.rate_mbit_s}mbit"])
        return argv

    def apply_profile(self) -> None:
        for container in (self.publisher_container, self.subscriber_container):
            _run(
                [
                    "docker",
                    "exec",
                    container,
                    "tc",
                    "qdisc",
                    "replace",
                    "dev",
                    "eth0",
                    "root",
                    *self._netem_args(self.spec),
                ]
            )

    def __enter__(self) -> DockerNetworkTrial:
        self.preflight()
        self.work_dir.mkdir(parents=True, exist_ok=True)
        _run(
            [
                "docker",
                "network",
                "create",
                "--driver",
                "bridge",
                "--subnet",
                "10.88.0.0/24",
                "--opt",
                f"com.docker.network.driver.mtu={_LINK_MTU_BYTES}",
                self.network_name,
            ]
        )
        try:
            self._run_endpoint(self.publisher_container, PUBLISHER_ADDRESS)
            self._run_endpoint(self.subscriber_container, SUBSCRIBER_ADDRESS)
            self.apply_profile()
            _run(
                [
                    "docker",
                    "exec",
                    self.publisher_container,
                    "ping",
                    "-c",
                    "1",
                    SUBSCRIBER_ADDRESS,
                ]
            )
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def qdisc_state(self) -> dict[str, str]:
        return {
            role: _run(
                ["docker", "exec", container, "tc", "qdisc", "show", "dev", "eth0"]
            ).stdout.strip()
            for role, container in (
                ("publisher", self.publisher_container),
                ("subscriber", self.subscriber_container),
            )
        }

    def link_state(self) -> dict[str, str]:
        return {
            role: _run(
                [
                    "docker",
                    "exec",
                    container,
                    "ip",
                    "-details",
                    "link",
                    "show",
                    "eth0",
                ]
            ).stdout.strip()
            for role, container in (
                ("publisher", self.publisher_container),
                ("subscriber", self.subscriber_container),
            )
        }

    def network_state(self) -> dict[str, Any]:
        completed = _run(["docker", "network", "inspect", self.network_name])
        return cast("dict[str, Any]", json.loads(completed.stdout)[0])

    def __exit__(self, *exc: Any) -> None:
        for container in (self.publisher_container, self.subscriber_container):
            subprocess.run(
                ["docker", "rm", "--force", container],
                check=False,
                capture_output=True,
                text=True,
                timeout=60.0,
            )
        subprocess.run(
            ["docker", "network", "rm", self.network_name],
            check=False,
            capture_output=True,
            text=True,
            timeout=60.0,
        )
