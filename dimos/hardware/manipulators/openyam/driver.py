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

"""OpenYAM (Anvil Robotics) CAN driver helpers. SI units throughout.

The OpenYAM uses Damiao DM-series servo motors in MIT mode — the same wire
protocol as OpenArm — so the frame codec and bus class are reused from
``dimos.hardware.manipulators.openarm.driver``. This module adds what is
YAM-specific:

- the default motor layout (6 arm joints + gripper on one bus), and
- transport resolution, so the same adapter runs over Linux SocketCAN or a
  userspace backend (``gs_usb``/``slcan``) on macOS, where SocketCAN does
  not exist.

Motor layout matches the topology in the canonical Linux-side integration
(PR #3129, ``OpenYamDamiaoAdapter``): Damiao DM4340 x3 + DM4310 x3 on send
IDs 1..6 (reply = send | 0x10), DM4310 gripper on 0x08/0x18, 1 Mbit/s
classical CAN. Opening the gripper *decreases* motor position. Not yet
validated against physical hardware from this driver — verify with
``dimos/robot/manipulators/openyam/scripts/openyam_can_probe.py`` before
enabling motors, and override via adapter kwargs where reality differs.
Wrong motor *types* mis-scale velocity/torque quantization (position range
is identical across DM types); wrong *IDs* mean no replies at all.
"""

from __future__ import annotations

import sys

from dimos.hardware.manipulators.openarm.driver import (
    CTRL_MODE_MIT,
    DamiaoMotor,
    MotorState,
    MotorType,
    OpenArmBus,
)

__all__ = [
    "CANABLE_PRODUCT_ID",
    "CANABLE_VENDOR_ID",
    "CTRL_MODE_MIT",
    "DEFAULT_ARM_MOTOR_TYPES",
    "DEFAULT_BITRATE",
    "DEFAULT_GRIPPER_MOTOR_TYPE",
    "DEFAULT_GRIPPER_SEND_ID",
    "DamiaoMotor",
    "MotorState",
    "MotorType",
    "YamBus",
    "gs_usb_mac_bus_factory",
    "resolve_transport",
]

# The Damiao protocol is arm-agnostic; the bus class carries no OpenArm
# specifics beyond its name.
YamBus = OpenArmBus

DEFAULT_BITRATE = 1_000_000

# Presumed BOM (I2RT YAM Standard layout): three high-torque shoulder motors,
# three wrist motors. Override with adapter kwarg ``arm_motor_types`` once the
# actual unit is probed.
DEFAULT_ARM_MOTOR_TYPES: list[MotorType] = [
    MotorType.DM4340,  # joint1
    MotorType.DM4340,  # joint2
    MotorType.DM4340,  # joint3
    MotorType.DM4310,  # joint4
    MotorType.DM4310,  # joint5
    MotorType.DM4310,  # joint6
]
# Gripper per PR #3129's OpenYamDamiaoAdapter: DM4310 at 0x08/0x18, and
# opening the gripper decreases motor position.
DEFAULT_GRIPPER_SEND_ID = 0x08
DEFAULT_GRIPPER_MOTOR_TYPE = MotorType.DM4310

# candlelight-firmware CANable 2.0, the dongle Anvil ships with the OpenYAM
CANABLE_VENDOR_ID = 0x1D50
CANABLE_PRODUCT_ID = 0x606F


def resolve_transport(
    address: str,
    *,
    interface: str | None = None,
    bitrate: int = DEFAULT_BITRATE,
    bus_index: int = 0,
) -> tuple[str, str, dict[str, object]]:
    """Map an ``address`` string to (interface, channel, bus_kwargs) for can.Bus.

    Explicit ``interface`` always wins. Otherwise:

    - ``canN``/``vcanN`` → ``socketcan`` on Linux; on macOS (no SocketCAN)
      falls through to ``gs_usb``, which drives candlelight-firmware adapters
      (e.g. CANable 2.0) via libusb.
    - ``gs_usb`` or ``gs_usb:N`` → ``gs_usb`` device N.
    - a serial device path (``/dev/tty*``, ``COM*``) → ``slcan`` (fallback for
      dongles running CDC/serial firmware instead of candlelight).
    - anything else is passed through verbatim as a python-can channel.

    SocketCAN takes its bitrate from ``ip link``; userspace backends need it
    passed in-process, so it lands in ``bus_kwargs`` for those.
    """
    if interface is None:
        if address.startswith("gs_usb"):
            interface = "gs_usb"
        elif address.startswith("/dev/tty") or address.upper().startswith("COM"):
            interface = "slcan"
        elif address.startswith(("can", "vcan")) and sys.platform != "linux":
            interface = "gs_usb"
        elif address.startswith(("can", "vcan")):
            interface = "socketcan"
        else:
            interface = "socketcan"

    if interface == "gs_usb":
        # channel like "gs_usb:1" selects device index 1
        if address.startswith("gs_usb") and ":" in address:
            bus_index = int(address.split(":", 1)[1])
        return interface, address, {"index": bus_index, "bitrate": bitrate}
    if interface == "slcan":
        return interface, address, {"bitrate": bitrate}
    return interface, address, {}


def gs_usb_mac_bus_factory(
    *,
    bitrate: int = DEFAULT_BITRATE,
    vendor_id: int = CANABLE_VENDOR_ID,
    product_id: int = CANABLE_PRODUCT_ID,
):
    """Return a zero-arg factory building the shared libusb gs_usb bus.

    This is the macOS transport: python-can's stock ``gs_usb`` backend
    mis-handles candlelight adapters from userspace (kernel-driver detach,
    hardcoded TX endpoint, unfiltered TX echoes), while
    ``dimos.hardware.can.gs_usb_bus.GsUsbMacBus`` handles those quirks and
    is hardware-validated at a 250 Hz control loop on the Galaxea A1Z.
    """

    def _factory():
        from dimos.hardware.can.gs_usb_bus import GsUsbMacBus

        return GsUsbMacBus(
            "gs_usb",
            vendor_id=vendor_id,
            product_id=product_id,
            bitrate=bitrate,
        )

    return _factory


def make_yam_motors(
    dof: int,
    *,
    arm_motor_types: list[MotorType] | None = None,
    motor_ids: list[int] | None = None,
    recv_id_offset: int = 0x10,
) -> list[DamiaoMotor]:
    """Build the arm-joint motor list (gripper excluded)."""
    types = list(arm_motor_types) if arm_motor_types is not None else list(DEFAULT_ARM_MOTOR_TYPES)
    ids = list(motor_ids) if motor_ids is not None else list(range(1, dof + 1))
    if len(types) != dof:
        raise ValueError(f"arm_motor_types must have {dof} entries, got {len(types)}")
    if len(ids) != dof:
        raise ValueError(f"motor_ids must have {dof} entries, got {len(ids)}")
    return [
        DamiaoMotor(sid, mt, recv_id=sid | recv_id_offset)
        for sid, mt in zip(ids, types, strict=True)
    ]
