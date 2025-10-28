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

# Copyright 2025 Dimensional Inc.

"""Comprehensive tests for Unitree B1 connection module Timer implementation."""

# TODO: These tests are reaching too much into `conn` by setting and shutting
# down threads manually. That code is already in the connection module, and
# should be used and tested. Additionally, tests should always use `try-finally`
# to clean up even if the test fails.

import threading
import time

from dimos.msgs.geometry_msgs import TwistStamped, Vector3
from dimos.msgs.std_msgs.Int32 import Int32

from .connection import MockB1ConnectionModule


class TestB1Connection:
    """Test suite for B1 connection module with Timer implementation."""

    def test_watchdog_actually_zeros_commands(self) -> None:
        """Test that watchdog thread zeros commands after timeout."""
        conn = MockB1ConnectionModule(ip="127.0.0.1", port=9090)
        conn.running = True
        conn.watchdog_running = True
        conn.send_thread = threading.Thread(target=conn._send_loop, daemon=True)
        conn.send_thread.start()
        conn.watchdog_thread = threading.Thread(target=conn._watchdog_loop, daemon=True)
        conn.watchdog_thread.start()

        # Send a forward command
        twist_stamped = TwistStamped(
            ts=time.time(),
            frame_id="base_link",
            linear=Vector3(1.0, 0, 0),
            angular=Vector3(0, 0, 0),
        )
        conn.handle_twist_stamped(twist_stamped)

        # Verify command is set
        assert conn._current_cmd.ly == 1.0
        assert conn._current_cmd.mode == 2
        assert not conn.timeout_active

        # Wait for watchdog timeout (200ms + buffer)
        time.sleep(0.3)

        # Verify commands were zeroed by watchdog
        assert conn._current_cmd.ly == 0.0
        assert conn._current_cmd.lx == 0.0
        assert conn._current_cmd.rx == 0.0
        assert conn._current_cmd.ry == 0.0
        assert conn._current_cmd.mode == 2  # Mode maintained
        assert conn.timeout_active

        conn.running = False
        conn.watchdog_running = False
        conn.send_thread.join(timeout=0.5)
        conn.watchdog_thread.join(timeout=0.5)
        conn._close_module()

    def test_watchdog_resets_on_new_command(self) -> None:
        """Test that watchdog timeout resets when new command arrives."""
        conn = MockB1ConnectionModule(ip="127.0.0.1", port=9090)
        conn.running = True
        conn.watchdog_running = True
        conn.send_thread = threading.Thread(target=conn._send_loop, daemon=True)
        conn.send_thread.start()
        conn.watchdog_thread = threading.Thread(target=conn._watchdog_loop, daemon=True)
        conn.watchdog_thread.start()

        # Send first command
        twist1 = TwistStamped(
            ts=time.time(),
            frame_id="base_link",
            linear=Vector3(1.0, 0, 0),
            angular=Vector3(0, 0, 0),
        )
        conn.handle_twist_stamped(twist1)
        assert conn._current_cmd.ly == 1.0

        # Wait 150ms (not enough to trigger timeout)
        time.sleep(0.15)

        # Send second command before timeout
        twist2 = TwistStamped(
            ts=time.time(),
            frame_id="base_link",
            linear=Vector3(0.5, 0, 0),
            angular=Vector3(0, 0, 0),
        )
        conn.handle_twist_stamped(twist2)

        # Command should be updated and no timeout
        assert conn._current_cmd.ly == 0.5
        assert not conn.timeout_active

        # Wait another 150ms (total 300ms from second command)
        time.sleep(0.15)
        # Should still not timeout since we reset the timer
        assert not conn.timeout_active
        assert conn._current_cmd.ly == 0.5

        conn.running = False
        conn.watchdog_running = False
        conn.send_thread.join(timeout=0.5)
        conn.watchdog_thread.join(timeout=0.5)
        conn._close_module()

    def test_watchdog_thread_efficiency(self) -> None:
        """Test that watchdog uses only one thread regardless of command rate."""
        conn = MockB1ConnectionModule(ip="127.0.0.1", port=9090)
        conn.running = True
        conn.watchdog_running = True
        conn.send_thread = threading.Thread(target=conn._send_loop, daemon=True)
        conn.send_thread.start()
        conn.watchdog_thread = threading.Thread(target=conn._watchdog_loop, daemon=True)
        conn.watchdog_thread.start()

        # Count threads before sending commands
        initial_thread_count = threading.active_count()

        # Send many commands rapidly (would create many Timer threads in old implementation)
        for i in range(50):
            twist = TwistStamped(
                ts=time.time(),
                frame_id="base_link",
                linear=Vector3(i * 0.01, 0, 0),
                angular=Vector3(0, 0, 0),
            )
            conn.handle_twist_stamped(twist)
            time.sleep(0.01)  # 100Hz command rate

        # Thread count should be same (no new threads created)
        final_thread_count = threading.active_count()
        assert final_thread_count == initial_thread_count, "No new threads should be created"

        conn.running = False
        conn.watchdog_running = False
        conn.send_thread.join(timeout=0.5)
        conn.watchdog_thread.join(timeout=0.5)
        conn._close_module()

    def test_watchdog_with_send_loop_blocking(self) -> None:
        """Test that watchdog still works if send loop blocks."""
        conn = MockB1ConnectionModule(ip="127.0.0.1", port=9090)

        # Mock the send loop to simulate blocking
        original_send_loop = conn._send_loop
        block_event = threading.Event()

        def blocking_send_loop() -> None:
            # Block immediately
            block_event.wait()
            # Then run normally
            original_send_loop()

        conn._send_loop = blocking_send_loop
        conn.running = True
        conn.watchdog_running = True
        conn.send_thread = threading.Thread(target=conn._send_loop, daemon=True)
        conn.send_thread.start()
        conn.watchdog_thread = threading.Thread(target=conn._watchdog_loop, daemon=True)
        conn.watchdog_thread.start()

        # Send command
        twist = TwistStamped(
            ts=time.time(),
            frame_id="base_link",
            linear=Vector3(1.0, 0, 0),
            angular=Vector3(0, 0, 0),
        )
        conn.handle_twist_stamped(twist)
        assert conn._current_cmd.ly == 1.0

        # Wait for watchdog timeout
        time.sleep(0.3)

        # Watchdog should have zeroed commands despite blocked send loop
        assert conn._current_cmd.ly == 0.0
        assert conn.timeout_active

        # Unblock send loop
        block_event.set()
        conn.running = False
        conn.watchdog_running = False
        conn.send_thread.join(timeout=0.5)
        conn.watchdog_thread.join(timeout=0.5)
        conn._close_module()

    def test_continuous_commands_prevent_timeout(self) -> None:
        """Test that continuous commands prevent watchdog timeout."""
        conn = MockB1ConnectionModule(ip="127.0.0.1", port=9090)
        conn.running = True
        conn.watchdog_running = True
        conn.send_thread = threading.Thread(target=conn._send_loop, daemon=True)
        conn.send_thread.start()
        conn.watchdog_thread = threading.Thread(target=conn._watchdog_loop, daemon=True)
        conn.watchdog_thread.start()

        # Send commands continuously for 500ms (should prevent timeout)
        start = time.time()
        commands_sent = 0
        while time.time() - start < 0.5:
            twist = TwistStamped(
                ts=time.time(),
                frame_id="base_link",
                linear=Vector3(0.5, 0, 0),
                angular=Vector3(0, 0, 0),
            )
            conn.handle_twist_stamped(twist)
            commands_sent += 1
            time.sleep(0.05)  # 50ms between commands (well under 200ms timeout)

        # Should never timeout
        assert not conn.timeout_active, "Should not timeout with continuous commands"
        assert conn._current_cmd.ly == 0.5, "Commands should still be active"
        assert commands_sent >= 9, f"Should send at least 9 commands in 500ms, sent {commands_sent}"

        conn.running = False
        conn.watchdog_running = False
        conn.send_thread.join(timeout=0.5)
        conn.watchdog_thread.join(timeout=0.5)
        conn._close_module()

    def test_watchdog_timing_accuracy(self) -> None:
        """Test that watchdog zeros commands at approximately 200ms."""
        conn = MockB1ConnectionModule(ip="127.0.0.1", port=9090)
        conn.running = True
        conn.watchdog_running = True
        conn.send_thread = threading.Thread(target=conn._send_loop, daemon=True)
        conn.send_thread.start()
        conn.watchdog_thread = threading.Thread(target=conn._watchdog_loop, daemon=True)
        conn.watchdog_thread.start()

        # Send command and record time
        start_time = time.time()
        twist = TwistStamped(
            ts=time.time(),
            frame_id="base_link",
            linear=Vector3(1.0, 0, 0),
            angular=Vector3(0, 0, 0),
        )
        conn.handle_twist_stamped(twist)

        # Wait for timeout checking periodically
        timeout_time = None
        while time.time() - start_time < 0.5:
            if conn.timeout_active:
                timeout_time = time.time()
                break
            time.sleep(0.01)

        assert timeout_time is not None, "Watchdog should timeout within 500ms"

        # Check timing (should be close to 200ms + up to 50ms watchdog interval)
        elapsed = timeout_time - start_time
        print(f"\nWatchdog timeout occurred at exactly {elapsed:.3f} seconds")
        assert 0.19 <= elapsed <= 0.3, f"Watchdog timed out at {elapsed:.3f}s, expected ~0.2-0.25s"

        conn.running = False
        conn.watchdog_running = False
        conn.send_thread.join(timeout=0.5)
        conn.watchdog_thread.join(timeout=0.5)
        conn._close_module()

    def test_mode_changes_with_watchdog(self) -> None:
        """Test that mode changes work correctly with watchdog."""
        conn = MockB1ConnectionModule(ip="127.0.0.1", port=9090)
        conn.running = True
        conn.watchdog_running = True
        conn.send_thread = threading.Thread(target=conn._send_loop, daemon=True)
        conn.send_thread.start()
        conn.watchdog_thread = threading.Thread(target=conn._watchdog_loop, daemon=True)
        conn.watchdog_thread.start()

        # Give threads time to initialize
        time.sleep(0.05)

        # Send walk command
        twist = TwistStamped(
            ts=time.time(),
            frame_id="base_link",
            linear=Vector3(1.0, 0, 0),
            angular=Vector3(0, 0, 0),
        )
        conn.handle_twist_stamped(twist)
        assert conn.current_mode == 2
        assert conn._current_cmd.ly == 1.0

        # Wait for timeout first (0.2s timeout + 0.15s margin for reliability)
        time.sleep(0.35)
        assert conn.timeout_active
        assert conn._current_cmd.ly == 0.0  # Watchdog zeroed it

        # Now change mode to STAND
        mode_msg = Int32()
        mode_msg.data = 1  # STAND
        conn.handle_mode(mode_msg)
        assert conn.current_mode == 1
        assert conn._current_cmd.mode == 1
        # timeout_active stays true since we didn't send new movement commands

        conn.running = False
        conn.watchdog_running = False
        conn.send_thread.join(timeout=0.5)
        conn.watchdog_thread.join(timeout=0.5)
        conn._close_module()

    def test_watchdog_stops_movement_when_commands_stop(self) -> None:
        """Verify watchdog zeros commands when packets stop being sent."""
        conn = MockB1ConnectionModule(ip="127.0.0.1", port=9090)
        conn.running = True
        conn.watchdog_running = True
        conn.send_thread = threading.Thread(target=conn._send_loop, daemon=True)
        conn.send_thread.start()
        conn.watchdog_thread = threading.Thread(target=conn._watchdog_loop, daemon=True)
        conn.watchdog_thread.start()

        # Simulate sending movement commands for a while
        for _i in range(5):
            twist = TwistStamped(
                ts=time.time(),
                frame_id="base_link",
                linear=Vector3(1.0, 0, 0),
                angular=Vector3(0, 0, 0.5),  # Forward and turning
            )
            conn.handle_twist_stamped(twist)
            time.sleep(0.05)  # Send at 20Hz

        # Verify robot is moving
        assert conn._current_cmd.ly == 1.0
        assert conn._current_cmd.lx == -0.25  # angular.z * 0.5 -> lx (for turning)
        assert conn.current_mode == 2  # WALK mode
        assert not conn.timeout_active

        # Wait for watchdog to detect timeout (200ms + buffer)
        time.sleep(0.3)

        assert conn.timeout_active, "Watchdog should have detected timeout"
        assert conn._current_cmd.ly == 0.0, "Forward velocity should be zeroed"
        assert conn._current_cmd.lx == 0.0, "Lateral velocity should be zeroed"
        assert conn._current_cmd.rx == 0.0, "Rotation X should be zeroed"
        assert conn._current_cmd.ry == 0.0, "Rotation Y should be zeroed"
        assert conn.current_mode == 2, "Mode should stay as WALK"

        # Verify recovery works - send new command
        twist = TwistStamped(
            ts=time.time(),
            frame_id="base_link",
            linear=Vector3(0.5, 0, 0),
            angular=Vector3(0, 0, 0),
        )
        conn.handle_twist_stamped(twist)

        # Give watchdog time to detect recovery
        time.sleep(0.1)

        assert not conn.timeout_active, "Should recover from timeout"
        assert conn._current_cmd.ly == 0.5, "Should accept new commands"

        conn.running = False
        conn.watchdog_running = False
        conn.send_thread.join(timeout=0.5)
        conn.watchdog_thread.join(timeout=0.5)
        conn._close_module()

    def test_rapid_command_thread_safety(self) -> None:
        """Test thread safety with rapid commands from multiple threads."""
        conn = MockB1ConnectionModule(ip="127.0.0.1", port=9090)
        conn.running = True
        conn.watchdog_running = True
        conn.send_thread = threading.Thread(target=conn._send_loop, daemon=True)
        conn.send_thread.start()
        conn.watchdog_thread = threading.Thread(target=conn._watchdog_loop, daemon=True)
        conn.watchdog_thread.start()

        # Count initial threads
        initial_threads = threading.active_count()

        # Send commands from multiple threads rapidly
        def send_commands(thread_id) -> None:
            for _i in range(10):
                twist = TwistStamped(
                    ts=time.time(),
                    frame_id="base_link",
                    linear=Vector3(thread_id * 0.1, 0, 0),
                    angular=Vector3(0, 0, 0),
                )
                conn.handle_twist_stamped(twist)
                time.sleep(0.01)

        threads = []
        for i in range(3):
            t = threading.Thread(target=send_commands, args=(i,))
            threads.append(t)
            t.start()

        for t in threads:
            t.join()

        # Thread count should only increase by the 3 sender threads we created
        # No additional Timer threads should be created
        final_threads = threading.active_count()
        assert final_threads <= initial_threads, "No extra threads should be created by watchdog"

        # Commands should still work correctly
        assert conn._current_cmd.ly >= 0, "Last command should be set"
        assert not conn.timeout_active, "Should not be in timeout with recent commands"

        conn.running = False
        conn.watchdog_running = False
        conn.send_thread.join(timeout=0.5)
        conn.watchdog_thread.join(timeout=0.5)
        conn._close_module()
