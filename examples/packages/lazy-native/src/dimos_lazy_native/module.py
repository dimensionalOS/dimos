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

"""Two independently prepared binaries in one source-only Python distribution."""

from pathlib import Path

from dimos.core.native_module import LogFormat, NativeModule, NativeModuleConfig


if "source_package" not in NativeModuleConfig.model_fields:
    raise ImportError("This source-only example requires a dimOS host with source_package support")


class PackageProbeConfig(NativeModuleConfig):
    source_package: str | None = "dimos_lazy_native"
    source_dir: str | None = "native"
    executable: str = "target/release/package_probe"
    build_command: str | None = (
        "cargo build --release --locked --offline --bin package_probe --target-dir target"
    )
    message_file: str = str(Path(__file__).resolve().parent / "resources" / "message.txt")
    log_format: LogFormat = LogFormat.TEXT


class PackageProbe(NativeModule):
    config: PackageProbeConfig


class OtherProbeConfig(PackageProbeConfig):
    executable: str = "target/release/package_other"
    build_command: str | None = (
        "cargo build --release --locked --offline --bin package_other --target-dir target"
    )


class OtherProbe(NativeModule):
    config: OtherProbeConfig
