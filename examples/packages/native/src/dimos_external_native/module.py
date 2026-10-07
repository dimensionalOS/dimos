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

"""A native declaration owned entirely by this installed package."""

from pathlib import Path

from dimos.core.native_module import LogFormat, NativeModule, NativeModuleConfig

_PACKAGE = Path(__file__).resolve().parent


class PackageProbeConfig(NativeModuleConfig):
    executable: str = str(_PACKAGE / "bin" / "package_probe")
    message_file: str = str(_PACKAGE / "resources" / "message.txt")
    log_format: LogFormat = LogFormat.TEXT


class PackageProbe(NativeModule):
    config: PackageProbeConfig
