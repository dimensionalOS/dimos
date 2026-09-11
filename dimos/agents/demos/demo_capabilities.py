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

from __future__ import annotations

from threading import Event, Lock, Thread
import time
from typing import Any

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module

CAP_PAYLOAD = "payload"

DEMO_CAPABILITIES_PROMPT = """
You are controlling a simulated warehouse inspection robot.

Use the available tools to satisfy user requests. The tools are demos: they
sleep, stream status updates, and report timestamps so a human can inspect tool
ordering and capability conflicts.

When a user asks for multiple independent actions "at the same time", call the
requested tools in the same agent turn.

Some tools hold exclusive capabilities. If a tool returns "Cannot start ...",
report the exact conflict unless the user explicitly asked you to stop the
blocking tool and retry.

Don't say things like "Let me know if there's anything else you'd like to do!"
People will prompt you when they want. You don't need to ask for a prompt.
"""


def _format_time(t: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t % 1 * 1000):03d}"


def _stamp(label: str, start: float, end: float) -> str:
    return f"{label} (started {_format_time(start)}, finished {_format_time(end)})"


class DemoSensors(Module):
    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def read_battery(self) -> str:
        """Read the simulated robot battery percentage."""
        start = time.time()
        time.sleep(1.0)
        end = time.time()
        return _stamp("Battery is 83%", start, end)

    @skill
    def read_temperature(self) -> str:
        """Read the simulated cargo bay temperature."""
        start = time.time()
        time.sleep(5.0)
        end = time.time()
        return _stamp("Cargo bay temperature is 21.6 C", start, end)

    @skill
    def capture_photo(self) -> str:
        """Capture a simulated inspection photo."""
        start = time.time()
        time.sleep(2.0)
        end = time.time()
        return _stamp("Captured cam0.jpg", start, end)

    @skill
    def speak(self, text: str) -> str:
        """Say a short message through the simulated robot speaker.

        Args:
            text: The words the robot should say.
        """
        now = time.time()
        return _stamp(f'Robot said: "{text}"', now, now)


class DemoRobotActions(Module):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = Lock()
        self._patrol_stop = Event()
        self._patrol_thread: Thread | None = None
        self._lap_stop = Event()
        self._lap_thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        self._stop_patrol()
        self._stop_lap()
        super().stop()

    @skill(uses=[CAP_MOVEMENT])
    def turn_in_place(self, degrees: float) -> str:
        """Turn the simulated robot in place.

        Args:
            degrees: Signed turn amount in degrees. Positive turns left, negative turns right.
        """
        start = time.time()
        time.sleep(2.0)
        end = time.time()
        return _stamp(f"Turned {degrees:.1f} degrees in place", start, end)

    @skill(uses=[CAP_PAYLOAD])
    def weigh_payload(self, item: str) -> str:
        """Weigh an item on the simulated payload scale.

        Args:
            item: Name of the item to weigh.
        """
        start = time.time()
        time.sleep(2.0)
        end = time.time()
        return _stamp(f"{item} weighs 4.2 kg", start, end)

    @skill(uses=[CAP_PAYLOAD])
    def secure_payload(self, item: str) -> str:
        """Secure an item in the simulated payload bay.

        Args:
            item: Name of the item to secure.
        """
        start = time.time()
        time.sleep(3.0)
        end = time.time()
        return _stamp(f"{item} is secured in the payload bay", start, end)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def start_patrol(self) -> str:
        """Start a simulated warehouse patrol that streams waypoint updates."""
        time.sleep(1.0)
        # Open (or re-stamp, on a same-tool takeover) the tool-stream before the
        # "already running" return so the movement hold is always carried by a
        # live stream.
        self.start_tool("start_patrol")
        with self._lock:
            if self._patrol_thread is not None and self._patrol_thread.is_alive():
                return "Patrol is already running. Use stop_patrol to stop it."

            self._patrol_stop.clear()
            thread = Thread(target=self._patrol_loop, name="demo-patrol", daemon=True)
            self._patrol_thread = thread
            thread.start()
        return "Patrol started."

    @skill
    def stop_patrol(self) -> str:
        """Stop the simulated warehouse patrol."""
        self._stop_patrol()
        return "Patrol stopped."

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def do_a_lap(self) -> str:
        """Do one simulated patrol lap, stream progress, then stop automatically."""
        # Open (or re-stamp, on a same-tool takeover) the tool-stream before the
        # "already running" return so the movement hold is always carried by a
        # live stream.
        self.start_tool("do_a_lap")
        with self._lock:
            if self._lap_thread is not None and self._lap_thread.is_alive():
                return "Lap is already running."

            self._lap_stop.clear()
            thread = Thread(target=self._lap_loop, name="demo-lap", daemon=True)
            self._lap_thread = thread
            thread.start()
        return "Lap started. It will stop automatically."

    def _stop_patrol(self) -> None:
        thread: Thread | None
        with self._lock:
            self._patrol_stop.set()
            thread = self._patrol_thread

        if thread is not None:
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        with self._lock:
            if self._patrol_thread is thread:
                self._patrol_thread = None
        self.stop_tool("start_patrol")

    def _stop_lap(self) -> None:
        thread: Thread | None
        with self._lock:
            self._lap_stop.set()
            thread = self._lap_thread

        if thread is not None:
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        with self._lock:
            if self._lap_thread is thread:
                self._lap_thread = None
        self.stop_tool("do_a_lap")

    def _patrol_loop(self) -> None:
        waypoint = 1
        try:
            while not self._patrol_stop.wait(timeout=5.0):
                self.tool_update("start_patrol", f"visiting waypoint {waypoint}")
                waypoint += 1
        finally:
            self.stop_tool("start_patrol")

    def _lap_loop(self) -> None:
        try:
            for step in range(1, 5):
                if self._lap_stop.wait(timeout=6.5):
                    return
                self.tool_update("do_a_lap", f"lap checkpoint {step} of 4")
        finally:
            with self._lock:
                self._lap_thread = None
            self.stop_tool("do_a_lap")


class DemoMonitoring(Module):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = Lock()
        self._scan_stop = Event()
        self._scan_thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        self._stop_environment_scan()
        super().stop()

    @skill(lifecycle="background")
    def start_environment_scan(self) -> str:
        """Start a simulated environment scan that streams air-quality updates."""
        with self._lock:
            if self._scan_thread is not None and self._scan_thread.is_alive():
                return "Environment scan is already running."

            self._scan_stop.clear()
            self.start_tool("start_environment_scan")
            thread = Thread(target=self._scan_loop, name="demo-environment-scan", daemon=True)
            self._scan_thread = thread
            thread.start()
        return "Environment scan started."

    @skill
    def stop_environment_scan(self) -> str:
        """Stop the simulated environment scan."""
        self._stop_environment_scan()
        return "Environment scan stopped."

    def _stop_environment_scan(self) -> None:
        thread: Thread | None
        with self._lock:
            self._scan_stop.set()
            thread = self._scan_thread

        if thread is not None:
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        with self._lock:
            if self._scan_thread is thread:
                self._scan_thread = None
        self.stop_tool("start_environment_scan")

    def _scan_loop(self) -> None:
        reading = 1
        try:
            while not self._scan_stop.wait(timeout=4.7):
                pm25 = 5 + reading
                co2 = 410 + reading * 3
                self.tool_update(
                    "start_environment_scan",
                    f"air reading {reading}: PM2.5={pm25} ug/m3, CO2={co2} ppm",
                )
                reading += 1
        finally:
            self.stop_tool("start_environment_scan")


demo_capabilities = autoconnect(
    DemoSensors.blueprint(),
    DemoRobotActions.blueprint(),
    DemoMonitoring.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=DEMO_CAPABILITIES_PROMPT),
)
