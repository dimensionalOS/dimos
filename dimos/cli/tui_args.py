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

import argparse
from typing import get_args

from dimos.core.global_config import TransportBackend


def parse_tui_args(description: str) -> argparse.Namespace:
    """Parse standalone chat TUI options before starting UI work."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("mode", nargs="?", choices=["web"], help="Serve the TUI in a browser")
    parser.add_argument(
        "--transport", choices=get_args(TransportBackend), help="Override the configured transport"
    )
    return parser.parse_args()
