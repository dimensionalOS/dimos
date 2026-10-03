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

from __future__ import annotations

from importlib import import_module
import sys


def cli_main() -> None:
    if len(sys.argv) > 1 and sys.argv[1] == "network":
        import_module("dimos.cli.commands.network").network_app(
            args=sys.argv[2:], prog_name="dimos network"
        )
        return
    # The legacy CLI imports heavy robot/perception dependencies. Preflight
    # deliberately runs without initializing those modules or GlobalConfig.
    # Preserve legacy startup ordering, including native thread-pool settings.
    import_module("dimos.cli.dimos").cli_main()
