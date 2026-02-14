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

"""
iPhone Teleoperation Module.

Receives twist commands via LCM from the Deno WebSocket bridge,
applies dead-man's switch logic, and publishes TwistStamped commands.
"""

from dataclasses import dataclass
import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import TwistStamped
from dimos.teleop.base import TeleopProtocol
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class IPhoneTeleopConfig(ModuleConfig):
    """Configuration for iPhone Teleoperation Module."""

    control_loop_hz: float = 50.0
    timeout_s: float = 0.2  # Dead-man's switch timeout


class IPhoneTeleopModule(Module[IPhoneTeleopConfig], TeleopProtocol):
    """iPhone Teleoperation Module for mobile base velocity control.

    Receives twist commands from the iPhone web app via the Deno LCM bridge.
    Uses a dead-man's switch: if no twist is received within timeout_s,
    automatically publishes zero velocity.

    Implements TeleopProtocol.

    Outputs:
        - twist_output: TwistStamped (velocity command for robot)
    """

    default_config = IPhoneTeleopConfig

    # Input from Deno bridge
    iphone_twist: In[TwistStamped]

    # Output: velocity command to robot
    twist_output: Out[TwistStamped]

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        self._is_engaged: bool = False
        self._current_twist: TwistStamped | None = None
        self._last_twist_time: float = 0.0
        self._lock = threading.RLock()

        # Control loop
        self._control_loop_thread: threading.Thread | None = None
        self._control_loop_running = False

        logger.info("IPhoneTeleopModule initialized")

    # -------------------------------------------------------------------------
    # Public RPC Methods
    # -------------------------------------------------------------------------

    @rpc
    def start(self) -> None:
        """Start the iPhone teleoperation module."""
        super().start()

        if self.iphone_twist and self.iphone_twist.transport:  # type: ignore[attr-defined]
            self._disposables.add(Disposable(self.iphone_twist.subscribe(self._on_twist)))  # type: ignore[attr-defined]
            logger.info("Subscribed to: iphone_twist")
        else:
            logger.warning("Stream 'iphone_twist' has no transport — skipping")

        self._start_control_loop()
        logger.info("iPhone Teleoperation Module started")

    @rpc
    def stop(self) -> None:
        """Stop the iPhone teleoperation module."""
        logger.info("Stopping iPhone Teleoperation Module...")
        self._stop_control_loop()
        super().stop()

    @rpc
    def engage(self, hand: Any = None) -> bool:
        """Engage teleoperation."""
        with self._lock:
            self._is_engaged = True
            logger.info("iPhone teleop engaged")
        return True

    @rpc
    def disengage(self, hand: Any = None) -> None:
        """Disengage teleoperation."""
        with self._lock:
            self._is_engaged = False
            self._current_twist = None
            logger.info("iPhone teleop disengaged")

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _on_twist(self, msg: TwistStamped) -> None:
        """Callback for incoming twist from the iPhone web app."""
        with self._lock:
            self._current_twist = msg
            self._last_twist_time = time.perf_counter()
            if not self._is_engaged:
                self._is_engaged = True
                logger.info("iPhone teleop engaged (twist received)")

    # -------------------------------------------------------------------------
    # Control Loop
    # -------------------------------------------------------------------------

    def _start_control_loop(self) -> None:
        """Start the control loop thread."""
        if self._control_loop_running:
            return

        self._control_loop_running = True
        self._control_loop_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="IPhoneTeleopControlLoop",
        )
        self._control_loop_thread.start()
        logger.info(f"Control loop started at {self.config.control_loop_hz} Hz")

    def _stop_control_loop(self) -> None:
        """Stop the control loop thread."""
        self._control_loop_running = False
        if self._control_loop_thread is not None:
            self._control_loop_thread.join(timeout=1.0)
            self._control_loop_thread = None
        logger.info("Control loop stopped")

    def _control_loop(self) -> None:
        """Main control loop: publish twist at fixed rate with dead-man's switch.

        Holds self._lock for the entire iteration so overridable methods
        don't need to acquire it themselves.
        """
        period = 1.0 / self.config.control_loop_hz

        while self._control_loop_running:
            loop_start = time.perf_counter()
            try:
                with self._lock:
                    # Dead-man's switch: auto-disengage if no twist received recently
                    if self._is_engaged and self._last_twist_time > 0:
                        elapsed = time.perf_counter() - self._last_twist_time
                        if elapsed > self.config.timeout_s:
                            self._is_engaged = False
                            self._current_twist = None
                            logger.info("iPhone teleop disengaged (timeout)")

                    if self._should_publish():
                        output_twist = self._get_output_twist()
                        if output_twist is not None:
                            self._publish_msg(output_twist)
            except Exception:
                logger.exception("Error in iPhone teleop control loop")

            elapsed = time.perf_counter() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # -------------------------------------------------------------------------
    # Overridable Control Loop Methods
    # -------------------------------------------------------------------------

    def _should_publish(self) -> bool:
        """Check if we should publish a twist command.

        Override to add custom conditions.
        Default: Returns True if engaged.
        """
        return self._is_engaged

    def _get_output_twist(self) -> TwistStamped | None:
        """Get the twist to publish.

        Override to customize twist computation (e.g., apply scaling, filtering).
        Default: Returns the latest twist from the phone.
        """
        return self._current_twist

    def _publish_msg(self, output_msg: TwistStamped) -> None:
        """Publish twist command.

        Override to customize output (e.g., apply limits, remap axes).
        """
        self.twist_output.publish(output_msg)


class VisualizingIPhoneTeleopModule(IPhoneTeleopModule):
    """iPhone teleop with Rerun visualization.

    Logs twist linear/angular components as scalar time series to Rerun.
    Useful for debugging and development.
    """

    def _publish_msg(self, output_msg: TwistStamped) -> None:
        """Publish twist and log to Rerun."""
        super()._publish_msg(output_msg)

        try:
            import rerun as rr

            base = "world/teleop/iphone"
            rr.log(f"{base}/linear_x", rr.Scalars(output_msg.linear.x))  # type: ignore[attr-defined]
            rr.log(f"{base}/linear_y", rr.Scalars(output_msg.linear.y))  # type: ignore[attr-defined]
            rr.log(f"{base}/angular_z", rr.Scalars(output_msg.angular.z))  # type: ignore[attr-defined]
            rr.log(f"{base}/engaged", rr.Scalars(float(self._is_engaged)))  # type: ignore[attr-defined]
        except Exception:
            pass


iphone_teleop_module = IPhoneTeleopModule.blueprint
visualizing_iphone_teleop_module = VisualizingIPhoneTeleopModule.blueprint

__all__ = [
    "IPhoneTeleopConfig",
    "IPhoneTeleopModule",
    "VisualizingIPhoneTeleopModule",
    "iphone_teleop_module",
    "visualizing_iphone_teleop_module",
]
