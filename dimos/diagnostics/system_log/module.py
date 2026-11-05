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

"""Everything worth having when debugging a recording, on one stream, from the native ``system_log`` module.

``system_log`` carries JSON records, each with a "type" and "t" (unix seconds):

- health: CPU (total, per core, MHz), load, memory, temperatures (on Apple silicon the GPU's from the SMC; channels
  reading below absolute zero are listed as unreadable), power rails/supplies/battery, fans, GPUs,
  Wi-Fi (signal, SSID, bitrates, address), routes, network and disk throughput, Pi throttling
- health_dump: every sample since the last dump, at ``sample_hz``, when an alert starts, the network changes or
  ``bad_event`` fires
- alert: a condition starting (CPU pegged, memory low, weak or no Wi-Fi, something hot, Pi throttled)
- processes: the busiest processes, every ``process_period_s`` and with every dump
- dimos_log: one line of the run's *.jsonl logs, parsed
- module_stats: per-worker/per-module CPU and memory from the resource monitor (run with `--dtop`), forwarded by this wrapper
- whatever a robot publishes on ``robot_health`` (battery, motor temperatures, ...), passed through
"""

from __future__ import annotations

import json
import time
from typing import TYPE_CHECKING, Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.String import String


class SystemLogConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/diagnostics/system_log/rust"
    executable: str = "result/bin/system_log"
    # "." in a git checkout enters the whole repo, so the flake can read ../../../../native/rust (tracked files only).
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    sample_hz: float = Field(default=4.0, ge=0.1, le=100.0)
    publish_period_s: float = Field(default=1.0, ge=0.01, le=3600.0)
    slow_period_s: float = Field(default=5.0, ge=0.1, le=3600.0)
    # a subprocess (iw, nvidia-smi, ...) still running after this is killed, so it can't stall sampling
    command_timeout_s: float = Field(default=2.0, ge=0.1, le=600.0)
    process_period_s: float = Field(default=10.0, ge=0.1, le=3600.0)
    top_processes: int = Field(default=12, ge=1, le=1000)
    buffer_s: float = Field(default=120.0, ge=1.0, le=3600.0)
    log_poll_s: float = Field(default=0.5, ge=0.01, le=60.0)
    # the run's log directory (DIMOS_RUN_LOG_DIR) when empty
    log_dir: str = ""
    cmdline_chars: int = Field(default=200, ge=0, le=100000)
    # seconds after start that the one "identity" record (host, user, MACs, addresses, ...) is published
    identity_delay_s: float = Field(default=30.0, ge=0.0, le=86400.0)
    # an alert (and a dump of the ring) when one of these is crossed
    cpu_alert_percent: float = 95.0
    memory_alert_percent: float = 95.0
    weak_signal_dbm: float = -80.0
    hot_celsius: float = 90.0
    # where the resource monitor (`--dtop`) publishes; read here in Python, so not sent to the binary
    resource_stats_topic: str = "/resource_stats"

    def to_config_dict(self) -> dict[str, Any]:
        return {k: v for k, v in super().to_config_dict().items() if k != "resource_stats_topic"}


class SystemLog(NativeModule):
    """Machine health, a high-rate ring dumped when something goes wrong, module usage and dimos' logs on ``system_log``."""

    config: SystemLogConfig

    robot_health: In[String]
    bad_event: In[Bool]
    system_log: Out[String]

    _resource_stats: PubSubTransport[dict[str, Any]] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if self._resource_stats is not None:
            return
        # The resource monitor publishes pickled dicts the binary can't read, so per-module usage is forwarded here.
        resource_stats: PubSubTransport[dict[str, Any]] = make_transport(
            self.config.resource_stats_topic
        )
        resource_stats.subscribe(self._on_resource_stats)
        resource_stats.start()
        self._resource_stats = resource_stats

    @rpc
    def stop(self) -> None:
        if self._resource_stats is not None:
            self._resource_stats.stop()
            self._resource_stats = None
        super().stop()

    def _on_resource_stats(self, stats: dict[str, Any], _topic: Any = None) -> None:
        record = {"type": "module_stats", "t": time.time(), **stats}
        self.system_log.publish(String(json.dumps(record)))


if TYPE_CHECKING:
    SystemLog()
