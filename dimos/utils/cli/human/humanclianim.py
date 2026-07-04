#!/usr/bin/env python3
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

import sys
import threading
import time

from terminaltexteffects import Color  # type: ignore[attr-defined]
from terminaltexteffects.effects.effect_expand import Expand

from dimos.utils.cli import theme

_humancli_main = None
_import_complete = threading.Event()

print(theme.ACCENT)


def import_cli_in_background() -> None:
    """Import the heavy CLI modules in the background"""
    global _humancli_main
    try:
        from dimos.utils.cli.human.humancli import main as humancli_main

        _humancli_main = humancli_main
    except Exception as e:
        print(f"Failed to import CLI: {e}")
    finally:
        _import_complete.set()


def run_banner_animation() -> None:
    ascii_art = "\n" + theme.ascii_logo.replace("\n", "\n ")

    # Clear screen before starting animation
    print("\033[2J\033[H", end="", flush=True)

    effect = Expand(ascii_art)
    effect.effect_config.expand_direction = "center"  # type: ignore[attr-defined]
    effect.effect_config.final_gradient_stops = (Color(theme.ACCENT),)  # type: ignore[attr-defined]

    # Run the animation
    with effect.terminal_output() as terminal:  # type: ignore[attr-defined]
        for frame in effect:  # type: ignore[attr-defined]
            terminal.print(frame)

    # Brief pause to see the final frame
    time.sleep(0.5)

    # Clear screen for Textual to take over
    print("\033[2J\033[H", end="")


def main() -> None:
    # Start importing CLI in background (this is slow)
    import_thread = threading.Thread(target=import_cli_in_background, daemon=True)
    import_thread.start()

    # Run the animation while imports happen (if not in web mode)
    if not (len(sys.argv) > 1 and sys.argv[1] == "web"):
        run_banner_animation()

    _import_complete.wait(timeout=10)

    if _humancli_main:
        _humancli_main()
    else:
        # Fallback if threaded import failed
        from dimos.utils.cli.human.humancli import main as humancli_main

        humancli_main()


if __name__ == "__main__":
    main()
