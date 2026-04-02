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

from typing import Literal, TypeAlias

ViewerBackend: TypeAlias = Literal["rerun", "foxglove", "none"]
RerunOpenOption: TypeAlias = Literal["none", "web", "native", "both"]

RERUN_OPEN_DEFAULT: RerunOpenOption = "native"
RERUN_ENABLE_WEB = True
RERUN_GRPC_PORT = 9876
RERUN_WEB_PORT = 9090
