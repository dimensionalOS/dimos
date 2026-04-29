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

"""End-to-end test for the virtual-Arduino path.

Boots the ``arduino_twist_echo_virtual`` blueprint — which spins up a
``TestPublisher`` module, compiles the sketch via ``arduino-cli``,
launches ``qemu-system-avr`` with the ELF, and runs the ``arduino_bridge``
subprocess — then waits for several Twist messages to round-trip
``TestPublisher.cmd_out → twist_command → QEMU → twist_echo → TestPublisher.echo_in``.

This is the only test that actually exercises QEMU + the bridge binary +
a compiled AVR sketch as one unit, so it's the primary regression guard
for the ``virtual=True`` runtime feature.

Requirements:
    - ``nix`` must be on PATH.  ``ArduinoModule`` resolves ``arduino-cli``,
      ``avrdude``, and ``qemu-system-avr`` through the ``dimos_arduino_tools``
      flake output, so only ``nix`` itself needs to be installed — the
      Arduino toolchain is materialized on demand via ``nix build``.

Run with::

    uv run pytest dimos/hardware/arduino/examples/arduino_twist_echo/test_e2e_virtual.py -v -m "slow and tool"
"""

from __future__ import annotations

import shutil
import time
from types import MappingProxyType

import pytest

# Heavy imports are deferred past the skip check so a machine without the
# Arduino toolchain can still collect this file cheaply.
pytestmark = [pytest.mark.slow, pytest.mark.tool]


def _missing_binaries() -> list[str]:
    # Only `nix` is required on PATH; the rest of the toolchain
    # (arduino-cli, avrdude, qemu-system-avr) is resolved through the
    # arduino flake's `dimos_arduino_tools` package inside
    # ``ArduinoModule`` — no user-facing `nix develop` required.
    return ["nix"] if shutil.which("nix") is None else []


# Budget for the full pipeline: first-run `nix build` can be minutes on a
# cold store, sketch compile ~30s, QEMU boot ~1s.  We poll for echoes up to
# this total wall-clock time before giving up.
_BUILD_AND_RUN_TIMEOUT_S = 300.0
# After the pipeline is running, how many echoes we require before calling
# the test a pass.  Three gives us confidence that it's not a fluke of the
# first publish happening to race the bridge startup.
_REQUIRED_ECHOES = 3


def test_virtual_arduino_round_trip() -> None:
    """Full coordinator + QEMU + bridge round-trip.

    Passes iff ``TestPublisher.echo_count()`` reaches ``_REQUIRED_ECHOES``
    within the timeout, which can only happen if all of these worked:

    1. ``ArduinoModule.build()`` generated ``dimos_arduino.h`` correctly
    2. ``arduino-cli`` compiled the sketch against the Arduino LCM headers
    3. ``nix build .#arduino_bridge`` produced the bridge binary
    4. ``qemu-system-avr`` booted the ELF and announced a PTY
    5. The bridge opened the PTY, resolved fingerprints, and subscribed to LCM
    6. ``TestPublisher`` published a Twist on ``twist_command``
    7. The bridge serialized it into a DSP frame and wrote it to the PTY
    8. The AVR sketch decoded it with the Arduino-side C helpers
    9. The sketch re-encoded the Twist and sent it back on the echo topic
    10. The bridge parsed the inbound frame, prepended the LCM fingerprint,
        and published it on ``twist_echo``
    11. ``TestPublisher.echo_in`` received and counted it
    """
    if _missing_binaries():
        pytest.skip(
            "Virtual Arduino e2e test requires `nix` on PATH — the rest of "
            "the Arduino toolchain (arduino-cli, avrdude, qemu-system-avr) "
            "is fetched via `nix build .#dimos_arduino_tools` inside "
            "ArduinoModule.  Install Nix and re-run."
        )

    # Deferred imports — pulling in ModuleCoordinator spins up a lot of
    # machinery, no reason to pay for it on a skip.
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.hardware.arduino.examples.arduino_twist_echo.blueprint import (
        arduino_twist_echo_virtual,
    )
    from dimos.hardware.arduino.examples.arduino_twist_echo.test_publisher import (
        TestPublisher,
    )

    # Disable Rerun so tests don't try to launch a viewer.
    build_overrides = MappingProxyType({"g": {"viewer": "none"}})

    coordinator = ModuleCoordinator.build(arduino_twist_echo_virtual, build_overrides.copy())
    try:
        publisher = coordinator.get_instance(TestPublisher)
        assert publisher is not None, (
            "TestPublisher not found in coordinator — blueprint wiring regressed"
        )

        deadline = time.monotonic() + _BUILD_AND_RUN_TIMEOUT_S
        last_count = 0
        while time.monotonic() < deadline:
            count = publisher.echo_count()
            if count != last_count:
                # Useful signal when running -v: shows echoes trickling in
                # during the wait, so a human watching can tell whether it's
                # the build or the runtime that's slow.
                print(f"[e2e] echoes received: {count}")
                last_count = count
            if count >= _REQUIRED_ECHOES:
                return
            time.sleep(0.25)

        pytest.fail(
            f"Virtual Arduino round-trip did not reach {_REQUIRED_ECHOES} echoes "
            f"within {_BUILD_AND_RUN_TIMEOUT_S:.0f}s (got {last_count}). "
            f"Check that the bridge built, the sketch compiled, and QEMU booted — "
            f"inspect the coordinator logs for details."
        )
    finally:
        coordinator.stop()
