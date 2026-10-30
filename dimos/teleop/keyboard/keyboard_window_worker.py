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

"""Executable entry point for the keyboard window subprocess."""

import argparse
from multiprocessing.connection import Connection

from dimos.teleop.keyboard.keyboard_window import run_window


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection-fd", type=int, required=True)
    parser.add_argument("--linear-speed", type=float, required=True)
    parser.add_argument("--angular-speed", type=float, required=True)
    args = parser.parse_args()
    run_window(Connection(args.connection_fd), args.linear_speed, args.angular_speed)


if __name__ == "__main__":
    main()
