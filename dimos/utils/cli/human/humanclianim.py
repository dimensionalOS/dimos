#!/usr/bin/env python3
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

import os
import sys
import threading
import time

from terminaltexteffects import Color

from dimos.utils.cli import theme

# Global to store the imported main function
_humancli_main = None
_import_complete = threading.Event()

print(theme.ACCENT)


def import_cli_in_background():
    """Import the heavy CLI modules in the background"""
    global _humancli_main
    try:
        from dimos.utils.cli.human.humancli import main as humancli_main

        _humancli_main = humancli_main
    except Exception as e:
        print(f"Failed to import CLI: {e}")
    finally:
        _import_complete.set()


def get_effect_config(effect_name):
    """Get hardcoded configuration for a specific effect"""
    # Hardcoded configs for each effect
    configs = {
        "randomsequence": {
            "speed": 0.075,
            "final_gradient_stops": [Color(theme.ACCENT)],
        },
        "slide": {"direction": "left", "final_gradient_stops": [Color(theme.ACCENT)]},
        "sweep": {"direction": "left"},
        "print": {
            "print_speed": 10,
            "print_head_return_speed": 10,
            "final_gradient_stops": [Color(theme.ACCENT)],
        },
        "pour": {"pour_speed": 9},
        "matrix": {"rain_symbols": "01", "rain_fall_speed_range": (4, 7)},
        "decrypt": {"typing_speed": 5, "decryption_speed": 3},
        "burn": {"fire_chars": "█", "flame_color": "ffffff"},
        "expand": {"expand_direction": "center"},
        "scattered": {"movement_speed": 0.5},
        "rain": {"rain_symbols": "░▒▓█", "rain_fall_speed_range": (5, 10)},
    }

    return configs.get(effect_name, {})


def run_banner_animation():
    """Run the ASCII banner animation before launching Textual"""

    # Check if we should animate
    animation_style = os.environ.get("DIMOS_BANNER_ANIMATION", "print").lower()

    if animation_style == "none":
        return  # Skip animation

    from terminaltexteffects.effects.effect_burn import Burn
    from terminaltexteffects.effects.effect_decrypt import Decrypt
    from terminaltexteffects.effects.effect_expand import Expand
    from terminaltexteffects.effects.effect_matrix import Matrix
    from terminaltexteffects.effects.effect_overflow import Overflow
    from terminaltexteffects.effects.effect_pour import Pour
    from terminaltexteffects.effects.effect_print import Print
    from terminaltexteffects.effects.effect_rain import Rain
    from terminaltexteffects.effects.effect_random_sequence import RandomSequence
    from terminaltexteffects.effects.effect_scattered import Scattered
    from terminaltexteffects.effects.effect_slide import Slide
    from terminaltexteffects.effects.effect_sweep import Sweep

    # The DIMENSIONAL ASCII art
    ascii_art = """

   ██████╗ ██╗███╗   ███╗███████╗███╗   ██╗███████╗██╗ ██████╗ ███╗   ██╗ █████╗ ██╗
   ██╔══██╗██║████╗ ████║██╔════╝████╗  ██║██╔════╝██║██╔═══██╗████╗  ██║██╔══██╗██║
   ██║  ██║██║██╔████╔██║█████╗  ██╔██╗ ██║███████╗██║██║   ██║██╔██╗ ██║███████║██║
   ██║  ██║██║██║╚██╔╝██║██╔══╝  ██║╚██╗██║╚════██║██║██║   ██║██║╚██╗██║██╔══██║██║
   ██████╔╝██║██║ ╚═╝ ██║███████╗██║ ╚████║███████║██║╚██████╔╝██║ ╚████║██║  ██║███████╗
   ╚═════╝ ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝
"""

    # Choose effect based on style
    effect_map = {
        "slide": Slide,
        "sweep": Sweep,
        "print": Print,
        "pour": Pour,
        "burn": Burn,
        "matrix": Matrix,
        "rain": Rain,
        "scattered": Scattered,
        "expand": Expand,
        "decrypt": Decrypt,
        "overflow": Overflow,
        "randomsequence": RandomSequence,
    }

    EffectClass = effect_map.get(animation_style, Slide)

    # Clear screen before starting animation
    print("\033[2J\033[H", end="", flush=True)

    # Get effect configuration
    effect_config = get_effect_config(animation_style)

    # Create and run the effect with config
    effect = EffectClass(ascii_art)
    for key, value in effect_config.items():
        setattr(effect.effect_config, key, value)

    # Run the animation - terminal.print() handles all screen management
    with effect.terminal_output() as terminal:
        for frame in effect:
            terminal.print(frame)

    # Brief pause to see the final frame
    time.sleep(0.5)

    # Clear screen for Textual to take over
    print("\033[2J\033[H", end="")


def main():
    """Main entry point - run animation then launch the real CLI"""

    # Start importing CLI in background (this is slow)
    import_thread = threading.Thread(target=import_cli_in_background, daemon=True)
    import_thread.start()

    # Run the animation while imports happen (if not in web mode)
    if not (len(sys.argv) > 1 and sys.argv[1] == "web"):
        run_banner_animation()

    # Wait for import to complete
    _import_complete.wait(timeout=10)  # Max 10 seconds wait

    # Launch the real CLI
    if _humancli_main:
        _humancli_main()
    else:
        # Fallback if threaded import failed
        from dimos.utils.cli.human.humancli import main as humancli_main

        humancli_main()


if __name__ == "__main__":
    main()
