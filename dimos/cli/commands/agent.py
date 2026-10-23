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

"""`dimos agent` commands for the Python runtime."""

import typer

agent_app = typer.Typer(help="Manage the Python agent runtime")


@agent_app.command("load")
def load_cmd() -> None:
    """Ensure the relay bridge, HarnessRuntime and McpServer are running.

    Prints one JSON line: {"loaded": [...], "already": [...]}.
    """
    # The runtime imports the porcelain and module stack; keep `dimos --help` fast.
    from dimos.agents.harness.runtime.load import main

    raise typer.Exit(main())
