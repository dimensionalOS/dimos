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

import platform

# viser[urdf] has no installable wheel set on linux-aarch64 (see the `tests`
# dependency group marker in pyproject.toml), so these tests cannot be collected
# there: their module-level imports pull in viser. Mirror the skipif_aarch64
# convention (dimos/conftest.py) at collection time. test_state.py is
# intentionally NOT listed -- it imports only viser.state (no viser dependency)
# and runs on every platform.
collect_ignore = (
    [
        "test_gui_status.py",
        "test_operation_worker.py",
        "test_viser_visualization.py",
        "test_visualizer_lifecycle.py",
    ]
    if platform.machine() == "aarch64"
    else []
)
