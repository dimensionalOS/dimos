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
import signal
import subprocess
import time

import lcm
import pytest

from dimos.core.transport import pLCMTransport
from dimos.protocol.service.lcmservice import LCMService


class LCMSpy(LCMService):
    messages: dict[str, list[bytes]] = {}

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.l = lcm.LCM()

    def start(self) -> None:
        super().start()
        if self.l:
            self.l.subscribe(".*", self.msg)

    def wait_for_topic(self, topic: str, timeout: float = 30.0) -> list[bytes]:
        start_time = time.time()
        while time.time() - start_time < timeout:
            if topic in self.messages:
                return self.messages[topic]
            time.sleep(0.1)
        raise TimeoutError(f"Timeout waiting for topic {topic}")

    def wait_for_message_content(
        self, topic: str, content_contains: bytes, timeout: float = 30.0
    ) -> None:
        start_time = time.time()
        while time.time() - start_time < timeout:
            if topic in self.messages:
                for msg in self.messages[topic]:
                    if content_contains in msg:
                        return
            time.sleep(0.1)
        raise TimeoutError(f"Timeout waiting for message content on topic {topic}")

    def stop(self) -> None:
        super().stop()

    def msg(self, topic, data) -> None:
        self.messages.setdefault(topic, []).append(data)


class DimosRobotCall:
    process: subprocess.Popen | None

    def __init__(self) -> None:
        self.process = None

    def start(self) -> None:
        self.process = subprocess.Popen(
            ["dimos", "run", "demo-skill"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def stop(self) -> None:
        if self.process is None:
            return

        try:
            # Send the kill signal (SIGTERM for graceful shutdown)
            self.process.send_signal(signal.SIGTERM)

            # Record the time when we sent the kill signal
            shutdown_start = time.time()

            # Wait for the process to terminate with a 30-second timeout
            try:
                self.process.wait(timeout=30)
                shutdown_duration = time.time() - shutdown_start

                # Verify it shut down in time
                assert shutdown_duration <= 30, (
                    f"Process took {shutdown_duration:.2f} seconds to shut down, "
                    f"which exceeds the 30-second limit"
                )
            except subprocess.TimeoutExpired:
                # If we reach here, the process didn't terminate in 30 seconds
                self.process.kill()  # Force kill
                self.process.wait()  # Clean up
                raise AssertionError(
                    "Process did not shut down within 30 seconds after receiving SIGTERM"
                )

        except Exception:
            # Clean up if something goes wrong
            if self.process.poll() is None:  # Process still running
                self.process.kill()
                self.process.wait()
            raise


@pytest.fixture
def lcm_spy():
    lcm_spy = LCMSpy()
    lcm_spy.start()
    yield lcm_spy
    lcm_spy.stop()


@pytest.fixture
def dimos_robot_call():
    dimos_robot_call = DimosRobotCall()
    dimos_robot_call.start()
    yield dimos_robot_call
    dimos_robot_call.stop()


@pytest.fixture
def human_input():
    transport = pLCMTransport("/human_input")
    transport.lcm.start()

    def send_human_input(message: str) -> None:
        transport.publish(message)

    yield send_human_input

    transport.lcm.stop()


@pytest.mark.skipif(bool(os.getenv("CI")), reason="LCM spy doesn't work in CI.")
@pytest.mark.skipif(not os.getenv("OPENAI_API_KEY"), reason="OPENAI_API_KEY not set.")
def test_dimos_robot_demo_e2e(lcm_spy, dimos_robot_call, human_input) -> None:
    lcm_spy.wait_for_topic("/rpc/DemoCalculatorSkill/set_LlmAgent_register_skills/res")
    lcm_spy.wait_for_topic("/rpc/HumanInput/start/res")
    lcm_spy.wait_for_message_content("/agent", b"AIMessage")

    human_input("what is 52983 + 587237")

    lcm_spy.wait_for_message_content("/agent", b"640220")

    assert "/rpc/DemoCalculatorSkill/sum_numbers/req" in lcm_spy.messages
    assert "/rpc/DemoCalculatorSkill/sum_numbers/res" in lcm_spy.messages
