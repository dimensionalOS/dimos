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

"""Console-script entry point (``[project.scripts] dimos``).

Deliberately tiny: module workers use the multiprocessing ``forkserver`` start
method, and every worker re-runs the console script as ``__mp_main__`` before
it can deploy a module. The script only imports this module, so the 0.6 s
import of ``dimos.cli.dimos`` (typer plus every command) is paid once, in the
parent, inside ``main()``.
"""


def main() -> None:
    # dimos.cli.dimos (0.6 s): imported here so the __mp_main__ re-run stays free.
    from dimos.cli.dimos import cli_main

    cli_main()
