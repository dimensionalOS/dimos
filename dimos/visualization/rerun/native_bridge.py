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

from typing import Any, Literal

from dimos.core.native_module import LogFormat, NativeModule, NativeModuleConfig


class NativeRerunConfig(NativeModuleConfig):
    cwd: str | None = "../../../native/rust"
    executable: str = "target/release/dimos-rerun-bridge"
    build_command: str | None = "cargo build --release -p dimos-rerun-bridge"
    stdin_config: bool = True
    log_format: LogFormat = LogFormat.TEXT

    backend: Literal["lcm", "zenoh"]
    connect_url: str
    entity_prefix: str
    lcm_url: str
    max_hz: dict[str, float]
    python_patterns: list[str]
    recording_id: str
    zenoh_connect: list[str]
    zenoh_listen: list[str]
    zenoh_mode: str


def start_native_rerun_bridge(**config: Any) -> NativeModule:
    class NativeRerunProcess(NativeModule):
        config: NativeRerunConfig

    process = NativeRerunProcess(**config)
    process.build()
    process.start()
    return process
