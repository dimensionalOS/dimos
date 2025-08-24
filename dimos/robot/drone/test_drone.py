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

"""Core unit tests for drone module."""

import unittest
from unittest.mock import MagicMock, patch
import time

from dimos.robot.drone.mavlink_connection import MavlinkConnection, FakeMavlinkConnection
from dimos.robot.drone.dji_video_stream import FakeDJIVideoStream
from dimos.robot.drone.connection_module import DroneConnectionModule
from dimos.msgs.geometry_msgs import Vector3
import numpy as np
import os


class TestMavlinkProcessing(unittest.TestCase):
    """Test MAVLink message processing and coordinate conversions."""

    def test_mavlink_message_processing(self):
        """Test that MAVLink messages trigger correct odom/tf publishing."""
        conn = MavlinkConnection("udp:0.0.0.0:14550")

        # Mock the mavlink connection
        conn.mavlink = MagicMock()
        conn.connected = True

        # Track what gets published
        published_odom = []
        conn._odom_subject.on_next = lambda x: published_odom.append(x)

        # Create ATTITUDE message and process it
        attitude_msg = MagicMock()
        attitude_msg.get_type.return_value = "ATTITUDE"
        attitude_msg.to_dict.return_value = {
            "mavpackettype": "ATTITUDE",
            "roll": 0.1,
            "pitch": 0.2,  # Positive pitch = nose up in MAVLink
            "yaw": 0.3,  # Positive yaw = clockwise in MAVLink
        }

        # Mock recv_match to return our message once then None
        def recv_side_effect(*args, **kwargs):
            if not hasattr(recv_side_effect, "called"):
                recv_side_effect.called = True
                return attitude_msg
            return None

        conn.mavlink.recv_match = MagicMock(side_effect=recv_side_effect)

        # Process the message
        conn.update_telemetry(timeout=0.01)

        # Check telemetry was updated
        self.assertEqual(conn.telemetry["ATTITUDE"]["roll"], 0.1)
        self.assertEqual(conn.telemetry["ATTITUDE"]["pitch"], 0.2)
        self.assertEqual(conn.telemetry["ATTITUDE"]["yaw"], 0.3)

        # Check odom was published with correct coordinate conversion
        self.assertEqual(len(published_odom), 1)
        pose = published_odom[0]

        # Verify NED to ROS conversion happened
        # ROS uses different conventions: positive pitch = nose down, positive yaw = counter-clockwise
        # So we expect sign flips in the quaternion conversion
        self.assertIsNotNone(pose.orientation)

    def test_position_integration(self):
        """Test velocity integration for indoor flight positioning."""
        conn = MavlinkConnection("udp:0.0.0.0:14550")
        conn.mavlink = MagicMock()
        conn.connected = True

        # Initialize position tracking
        conn._position = {"x": 0.0, "y": 0.0, "z": 0.0}
        conn._last_update = time.time()

        # Create GLOBAL_POSITION_INT with velocities
        pos_msg = MagicMock()
        pos_msg.get_type.return_value = "GLOBAL_POSITION_INT"
        pos_msg.to_dict.return_value = {
            "mavpackettype": "GLOBAL_POSITION_INT",
            "lat": 0,
            "lon": 0,
            "alt": 0,
            "relative_alt": 1000,  # 1m in mm
            "vx": 100,  # 1 m/s North in cm/s
            "vy": 200,  # 2 m/s East in cm/s
            "vz": 0,
            "hdg": 0,
        }

        def recv_side_effect(*args, **kwargs):
            if not hasattr(recv_side_effect, "called"):
                recv_side_effect.called = True
                return pos_msg
            return None

        conn.mavlink.recv_match = MagicMock(side_effect=recv_side_effect)

        # Process with known dt
        old_time = conn._last_update
        conn.update_telemetry(timeout=0.01)
        dt = conn._last_update - old_time

        # Check position was integrated from velocities
        # vx=1m/s North → +X in ROS
        # vy=2m/s East → -Y in ROS (Y points West)
        expected_x = 1.0 * dt  # North velocity
        expected_y = -2.0 * dt  # East velocity (negated for ROS)

        self.assertAlmostEqual(conn._position["x"], expected_x, places=2)
        self.assertAlmostEqual(conn._position["y"], expected_y, places=2)

    def test_ned_to_ros_coordinate_conversion(self):
        """Test NED to ROS coordinate system conversion for all axes."""
        conn = MavlinkConnection("udp:0.0.0.0:14550")
        conn.mavlink = MagicMock()
        conn.connected = True

        # Initialize position
        conn._position = {"x": 0.0, "y": 0.0, "z": 0.0}
        conn._last_update = time.time()

        # Test with velocities in all directions
        # NED: North-East-Down
        # ROS: X(forward/North), Y(left/West), Z(up)
        pos_msg = MagicMock()
        pos_msg.get_type.return_value = "GLOBAL_POSITION_INT"
        pos_msg.to_dict.return_value = {
            "mavpackettype": "GLOBAL_POSITION_INT",
            "lat": 0,
            "lon": 0,
            "alt": 5000,  # 5m altitude in mm
            "relative_alt": 5000,
            "vx": 300,  # 3 m/s North (NED)
            "vy": 400,  # 4 m/s East (NED)
            "vz": -100,  # 1 m/s Up (negative in NED for up)
            "hdg": 0,
        }

        def recv_side_effect(*args, **kwargs):
            if not hasattr(recv_side_effect, "called"):
                recv_side_effect.called = True
                return pos_msg
            return None

        conn.mavlink.recv_match = MagicMock(side_effect=recv_side_effect)

        # Process message
        old_time = conn._last_update
        conn.update_telemetry(timeout=0.01)
        dt = conn._last_update - old_time

        # Verify coordinate conversion:
        # NED North (vx=3) → ROS +X
        # NED East (vy=4) → ROS -Y (ROS Y points West/left)
        # NED Down (vz=-1, up) → ROS +Z (ROS Z points up)

        # Position should integrate with converted velocities
        self.assertGreater(conn._position["x"], 0)  # North → positive X
        self.assertLess(conn._position["y"], 0)  # East → negative Y
        self.assertEqual(conn._position["z"], 5.0)  # Altitude from relative_alt (5000mm = 5m)

        # Check X,Y velocity integration (Z is set from altitude, not integrated)
        self.assertAlmostEqual(conn._position["x"], 3.0 * dt, places=2)
        self.assertAlmostEqual(conn._position["y"], -4.0 * dt, places=2)


class TestReplayMode(unittest.TestCase):
    """Test replay mode functionality."""

    def test_fake_mavlink_connection(self):
        """Test FakeMavlinkConnection replays messages correctly."""
        with patch("dimos.utils.testing.TimedSensorReplay") as mock_replay:
            # Mock the replay stream
            mock_stream = MagicMock()
            mock_messages = [
                {"mavpackettype": "ATTITUDE", "roll": 0.1, "pitch": 0.2, "yaw": 0.3},
                {"mavpackettype": "HEARTBEAT", "type": 2, "base_mode": 193},
            ]

            # Make stream emit our messages
            mock_replay.return_value.stream.return_value.subscribe = lambda callback: [
                callback(msg) for msg in mock_messages
            ]

            conn = FakeMavlinkConnection("replay")

            # Check messages are available
            msg1 = conn.mavlink.recv_match()
            self.assertIsNotNone(msg1)
            self.assertEqual(msg1.get_type(), "ATTITUDE")

            msg2 = conn.mavlink.recv_match()
            self.assertIsNotNone(msg2)
            self.assertEqual(msg2.get_type(), "HEARTBEAT")

    def test_fake_video_stream_no_throttling(self):
        """Test FakeDJIVideoStream returns replay stream directly."""
        with patch("dimos.utils.testing.TimedSensorReplay") as mock_replay:
            mock_stream = MagicMock()
            mock_replay.return_value.stream.return_value = mock_stream

            stream = FakeDJIVideoStream(port=5600)
            result_stream = stream.get_stream()

            # Verify stream is returned directly without throttling
            self.assertEqual(result_stream, mock_stream)

    def test_connection_module_replay_mode(self):
        """Test connection module uses Fake classes in replay mode."""
        with patch("dimos.robot.drone.mavlink_connection.FakeMavlinkConnection") as mock_fake_conn:
            with patch("dimos.robot.drone.dji_video_stream.FakeDJIVideoStream") as mock_fake_video:
                # Mock the fake connection
                mock_conn_instance = MagicMock()
                mock_conn_instance.connected = True
                mock_conn_instance.odom_stream.return_value.subscribe = MagicMock()
                mock_conn_instance.status_stream.return_value.subscribe = MagicMock()
                mock_conn_instance.telemetry_stream.return_value.subscribe = MagicMock()
                mock_fake_conn.return_value = mock_conn_instance

                # Mock the fake video
                mock_video_instance = MagicMock()
                mock_video_instance.start.return_value = True
                mock_video_instance.get_stream.return_value.subscribe = MagicMock()
                mock_fake_video.return_value = mock_video_instance

                # Create module with replay connection string
                module = DroneConnectionModule(connection_string="replay")
                module.video = MagicMock()
                module.movecmd = MagicMock()
                module.tf = MagicMock()

                # Start should use Fake classes
                result = module.start()

                self.assertTrue(result)
                mock_fake_conn.assert_called_once_with("replay")
                mock_fake_video.assert_called_once()

    def test_connection_module_replay_with_messages(self):
        """Test connection module in replay mode receives and processes messages."""
        import os

        os.environ["DRONE_CONNECTION"] = "replay"

        with patch("dimos.utils.testing.TimedSensorReplay") as mock_replay:
            # Set up MAVLink replay stream
            mavlink_messages = [
                {"mavpackettype": "HEARTBEAT", "type": 2, "base_mode": 193},
                {"mavpackettype": "ATTITUDE", "roll": 0.1, "pitch": 0.2, "yaw": 0.3},
                {
                    "mavpackettype": "GLOBAL_POSITION_INT",
                    "lat": 377810501,
                    "lon": -1224069671,
                    "alt": 0,
                    "relative_alt": 1000,
                    "vx": 100,
                    "vy": 0,
                    "vz": 0,
                    "hdg": 0,
                },
            ]

            # Set up video replay stream
            video_frames = [
                np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8),
                np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8),
            ]

            def create_mavlink_stream():
                stream = MagicMock()

                def subscribe(callback):
                    print("\n[TEST] MAVLink replay stream subscribed")
                    for msg in mavlink_messages:
                        print(f"[TEST] Replaying MAVLink: {msg['mavpackettype']}")
                        callback(msg)

                stream.subscribe = subscribe
                return stream

            def create_video_stream():
                stream = MagicMock()

                def subscribe(callback):
                    print("[TEST] Video replay stream subscribed")
                    for i, frame in enumerate(video_frames):
                        print(
                            f"[TEST] Replaying video frame {i + 1}/{len(video_frames)}, shape: {frame.shape}"
                        )
                        callback(frame)

                stream.subscribe = subscribe
                return stream

            # Configure mock replay to return appropriate streams
            def replay_side_effect(store_name):
                print(f"[TEST] TimedSensorReplay created for: {store_name}")
                mock = MagicMock()
                if "mavlink" in store_name:
                    mock.stream.return_value = create_mavlink_stream()
                elif "video" in store_name:
                    mock.stream.return_value = create_video_stream()
                return mock

            mock_replay.side_effect = replay_side_effect

            # Create and start connection module
            module = DroneConnectionModule(connection_string="replay")

            # Mock publishers to track what gets published
            published_odom = []
            published_video = []
            published_status = []

            module.odom = MagicMock(
                publish=lambda x: (
                    published_odom.append(x),
                    print(
                        f"[TEST] Published odom: position=({x.position.x:.2f}, {x.position.y:.2f}, {x.position.z:.2f})"
                    ),
                )
            )
            module.video = MagicMock(
                publish=lambda x: (
                    published_video.append(x),
                    print(
                        f"[TEST] Published video frame with shape: {x.data.shape if hasattr(x, 'data') else 'unknown'}"
                    ),
                )
            )
            module.status = MagicMock(
                publish=lambda x: (
                    published_status.append(x),
                    print(
                        f"[TEST] Published status: {x.data[:50]}..."
                        if hasattr(x, "data")
                        else "[TEST] Published status"
                    ),
                )
            )
            module.telemetry = MagicMock()
            module.tf = MagicMock()
            module.movecmd = MagicMock()

            print("\n[TEST] Starting connection module in replay mode...")
            result = module.start()

            # Give time for messages to process
            import time

            time.sleep(0.1)

            print(f"\n[TEST] Module started: {result}")
            print(f"[TEST] Total odom messages published: {len(published_odom)}")
            print(f"[TEST] Total video frames published: {len(published_video)}")
            print(f"[TEST] Total status messages published: {len(published_status)}")

            # Verify module started and is processing messages
            self.assertTrue(result)
            self.assertIsNotNone(module.connection)
            self.assertIsNotNone(module.video_stream)

            # Should have published some messages
            self.assertGreater(
                len(published_odom) + len(published_video) + len(published_status),
                0,
                "No messages were published in replay mode",
            )


if __name__ == "__main__":
    unittest.main()
