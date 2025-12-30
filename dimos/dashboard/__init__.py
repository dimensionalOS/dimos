# Copyright 2025 Dimensional Inc.
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

"""Dashboard module for visualization and monitoring.

Rerun Initialization:
    Main process (e.g., blueprints.build) starts Rerun server automatically.
    Worker modules connect to the server via connect_rerun().

Usage in modules:
    import rerun as rr
    from dimos.dashboard.rerun_init import connect_rerun

    class MyModule(Module):
        def start(self):
            super().start()
            connect_rerun()  # Connect to Rerun server
            rr.log("my/entity", my_data.to_rerun())
"""

from dimos.dashboard.rerun_init import connect_rerun, init_rerun_server, shutdown_rerun

__all__ = ["connect_rerun", "init_rerun_server", "shutdown_rerun"]
