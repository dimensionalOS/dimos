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

"""Mock G1 LowState publisher — provides synthetic rt/lowstate at a target rate.

Stands in for either real G1 hardware or Unitree's mujoco bridge so the
G1WholeBodyConnection Module + transport seam can be validated end-to-end
without hardware.

Usage (typically from test_g1_module.py):
    pub = MockG1LowStatePublisher(rate_hz=500, mode_machine=42)
    pub.start()
    ...
    pub.stop()

Note: ChannelFactoryInitialize is a per-process singleton. This publisher
calls it on construction; if the same process needs to do other DDS work
(e.g., subscribe to rt/lowcmd to verify command round-trip), instantiate
this publisher BEFORE creating other DDS participants.
"""

from __future__ import annotations

import logging
import threading
from threading import Thread
import time

logger = logging.getLogger(__name__)

_NUM_MOTORS = 29
_NUM_MOTOR_SLOTS = 35


class MockG1LowStatePublisher:
    """Publishes synthetic LowState_ messages on rt/lowstate at a target rate.

    Args:
        rate_hz: Publish rate (Hz).
        mode_machine: Value to put in LowState.mode_machine. MUST be non-zero
            so UnitreeG1LowLevelAdapter.connect() completes (it captures the
            first non-None mode_machine and exits the wait loop).
        network_interface: DDS NIC, default 0.
    """

    def __init__(
        self,
        rate_hz: float = 500.0,
        mode_machine: int = 42,
        network_interface: str = "",
    ) -> None:
        self._rate_hz = rate_hz
        self._mode_machine = mode_machine
        self._network_interface = network_interface
        self._stop = threading.Event()
        self._thread: Thread | None = None
        self._pub = None  # type: ignore[var-annotated]
        self._sequence = 0

    def start(self) -> None:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        # cyclonedds requires a string NIC; passing an int trips AttributeError on
        # participant init. Only pass when explicitly set. Wrap in try/except since
        # the call also raises if the factory was already initialized in this process.
        try:
            if self._network_interface:
                ChannelFactoryInitialize(0, self._network_interface)
            else:
                ChannelFactoryInitialize(0)
        except Exception as e:
            logger.debug(f"ChannelFactoryInitialize raised (likely already init'd): {e}")

        self._pub = ChannelPublisher("rt/lowstate", LowState_)
        self._pub.Init()

        self._thread = Thread(target=self._loop, name="mock-g1-lowstate", daemon=True)
        self._thread.start()
        logger.info(f"MockG1LowStatePublisher started @ {self._rate_hz} Hz")

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None
        logger.info(f"MockG1LowStatePublisher stopped after {self._sequence} messages")

    @property
    def messages_published(self) -> int:
        return self._sequence

    def _loop(self) -> None:
        # Use the default factory (mirrors how the adapter constructs LowCmd_) —
        # LowState_() directly would require all 9 positional fields.
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_

        msg = unitree_hg_msg_dds__LowState_()
        msg.mode_machine = self._mode_machine

        # IMU defaults (identity quaternion, zero everything else)
        msg.imu_state.quaternion[0] = 1.0
        msg.imu_state.quaternion[1] = 0.0
        msg.imu_state.quaternion[2] = 0.0
        msg.imu_state.quaternion[3] = 0.0
        for j in range(3):
            msg.imu_state.gyroscope[j] = 0.0
            msg.imu_state.accelerometer[j] = 0.0
            msg.imu_state.rpy[j] = 0.0

        period = 1.0 / self._rate_hz
        next_tick = time.perf_counter()

        while not self._stop.is_set():
            seq = self._sequence
            # Sweep value per motor — lets downstream verify ordering + freshness
            for i in range(_NUM_MOTORS):
                msg.motor_state[i].q = float(i) + 0.001 * seq
                msg.motor_state[i].dq = 0.0
                msg.motor_state[i].tau_est = 0.0

            try:
                self._pub.Write(msg)
                self._sequence += 1
            except Exception as e:
                logger.error(f"Mock publish failed: {e}")
                return

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # We fell behind — reset cadence to avoid runaway catch-up
                next_tick = time.perf_counter()


__all__ = ["MockG1LowStatePublisher"]
