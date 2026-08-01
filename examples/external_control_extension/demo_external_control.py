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

"""Run the external ControlCoordinator extension example."""

from __future__ import annotations

import logging

from dimos_external_control_extension.blueprints import run_demo


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    run_demo(target_writes=3, timeout=2.0)


if __name__ == "__main__":
    main()
