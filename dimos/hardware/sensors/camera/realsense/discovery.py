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

"""Which RealSense is which, by USB product id.

With two cameras attached librealsense hands back whichever it enumerated first, so a
blueprint that wants a particular model has to name a serial. Reading the serial off the
USB bus gets it without opening the device, so a blueprint can ask for "the D455" and
still work on a machine where the serials differ:

    python -m dimos.hardware.sensors.camera.realsense.discovery
"""

from __future__ import annotations

from functools import cache
from pathlib import Path
import platform
import plistlib
import subprocess
from typing import Any

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

INTEL_VENDOR_ID = 0x8086

# USB product ids, one model per id. A model absent from here is one nobody has needed
# to single out yet, not one that cannot be found.
PRODUCT_IDS = {
    "D405": 0x0B5B,
    "D415": 0x0AD3,
    "D435": 0x0B07,
    "D435i": 0x0B3A,
    "D455": 0x0B5C,
}

IOREG_TIMEOUT_SECONDS = 10.0


def find_serials(model: str) -> list[str]:
    """Serials of every attached camera of ``model``, in bus order."""
    if model not in PRODUCT_IDS:
        raise ValueError(f"Unknown RealSense model {model!r}; known: {sorted(PRODUCT_IDS)}")
    return _serials_by_product_id().get(PRODUCT_IDS[model], [])


def find_serial(model: str) -> str | None:
    """The serial of the one attached ``model``, or None if there is not exactly one.

    Ambiguity resolves to None rather than to a guess: picking arbitrarily between two
    cameras is the failure this exists to prevent.
    """
    serials = find_serials(model)
    if not serials:
        return None
    if len(serials) > 1:
        logger.warning(
            f"{len(serials)} {model} cameras attached ({', '.join(serials)}); "
            "name the one you want with serial_number"
        )
        return None
    return serials[0]


@cache
def _serials_by_product_id() -> dict[int, list[str]]:
    try:
        if platform.system() == "Darwin":
            return _scan_ioreg()
        return _scan_sysfs()
    except Exception as error:
        logger.warning(f"Could not enumerate USB cameras: {error!r}")
        return {}


SYSFS_USB_DEVICES = Path("/sys/bus/usb/devices")


def _scan_sysfs(root: Path = SYSFS_USB_DEVICES) -> dict[int, list[str]]:
    found: dict[int, list[str]] = {}
    for device in sorted(root.glob("*")):
        try:
            if int((device / "idVendor").read_text(), 16) != INTEL_VENDOR_ID:
                continue
            product_id = int((device / "idProduct").read_text(), 16)
            serial = (device / "serial").read_text().strip()
        except (OSError, ValueError):
            continue
        if serial:
            found.setdefault(product_id, []).append(serial)
    return found


def _scan_ioreg() -> dict[int, list[str]]:
    # The IOUSB plane rather than a device class: the class name differs by silicon.
    plist = subprocess.run(
        ["ioreg", "-a", "-p", "IOUSB", "-l"],
        capture_output=True,
        timeout=IOREG_TIMEOUT_SECONDS,
        check=True,
    ).stdout
    if not plist.strip():
        return {}
    return _parse_ioreg(plistlib.loads(plist))


def _parse_ioreg(root: Any) -> dict[int, list[str]]:
    found: dict[int, list[str]] = {}
    for entry in _ioreg_entries(root):
        if entry.get("idVendor") != INTEL_VENDOR_ID:
            continue
        product_id = entry.get("idProduct")
        serial = entry.get("USB Serial Number")
        if isinstance(product_id, int) and isinstance(serial, str) and serial:
            found.setdefault(product_id, []).append(serial)
    return found


def _ioreg_entries(node: Any) -> list[dict[str, Any]]:
    """ioreg nests devices under their hub in IORegistryEntryChildren, so walk the tree."""
    if isinstance(node, list):
        return [entry for child in node for entry in _ioreg_entries(child)]
    if isinstance(node, dict):
        nested = _ioreg_entries(node.get("IORegistryEntryChildren", []))
        return [node, *nested]
    return []


def main() -> None:
    for model in sorted(PRODUCT_IDS):
        for serial in find_serials(model):
            print(f"{model}\t{serial}")


if __name__ == "__main__":
    main()
