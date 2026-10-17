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

"""Fire GMSL cameras from MIIVII's hardware trigger (FSYNC), so every triggered camera exposes together.

In trigger mode a camera takes one frame per pulse, so the trigger rate is the frame rate.
The trigger is only reachable through MIIVII's closed C++ SDK, whose config-only
``MvGmslCamera`` constructor hands the config to GmslServer. Run it in its own process
(``python -m dimos.hardware.sensors.camera.utils.fsync_trigger HZ LINK_MASK``): the SDK opens
the GPU, leaves a reader thread behind, and a crash in it would take the caller down with it.
The setting lasts until reboot.
"""

from __future__ import annotations

import ctypes
import os
import subprocess
import sys

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MIIVII_GMSL_SDK = "/opt/miivii/lib/libmvgmslcamera_noopencv.so"


class _SyncConfig(ctypes.Structure):
    """The SDK's ``sync_out_a_cfg_client_t``."""

    _fields_ = [
        ("sync_camera_num", ctypes.c_uint8),
        ("sync_freq", ctypes.c_uint8),
        ("sync_camera_bit_draw", ctypes.c_uint8),
        ("async_camera_num", ctypes.c_uint8),
        ("async_freq", ctypes.c_uint8),
        ("async_camera_bit_draw", ctypes.c_uint8),
        ("async_camera_pos", ctypes.c_uint8 * 8),
    ]


def _send_trigger_config(hz: int, link_mask: int) -> None:
    """Call ``miivii::MvGmslCamera(sync_out_a_cfg_client_t)``; never destructed, as its destructor crashes."""
    camera = ctypes.create_string_buffer(1 << 16)
    config = _SyncConfig(bin(link_mask).count("1"), hz, link_mask)
    ctypes.CDLL(MIIVII_GMSL_SDK)._ZN6miivii12MvGmslCameraC1E23sync_out_a_cfg_client_t(
        camera, config
    )


def trigger_gmsl_cameras(hz: int, link_mask: int) -> None:
    """Fire the cameras on the GMSL links in ``link_mask`` from one hardware trigger at ``hz``, so their frames start within ~30 us."""
    if not os.path.exists(MIIVII_GMSL_SDK):
        logger.warning("no MIIVII GMSL SDK at %s; cameras stay free-running", MIIVII_GMSL_SDK)
        return
    try:
        subprocess.run(
            [sys.executable, "-m", __name__, str(hz), str(link_mask)], check=True, timeout=10
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
        logger.warning("camera trigger not set (%s); cameras stay free-running", error)


if __name__ == "__main__":
    _send_trigger_config(int(sys.argv[1]), int(sys.argv[2], 0))
