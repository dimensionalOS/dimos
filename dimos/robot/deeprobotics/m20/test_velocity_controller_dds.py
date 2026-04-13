from unittest.mock import MagicMock, patch

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.deeprobotics.m20.velocity_controller_dds import (
    NAV_CMD_BRIDGE_PORT,
    M20VelocityController,
    NavCmd,
)


def test_publish_once_retries_after_initial_bridge_connect_failure() -> None:
    first_socket = MagicMock()
    second_socket = MagicMock()
    first_socket.connect.side_effect = TimeoutError("bridge unavailable")

    with patch(
        "dimos.robot.deeprobotics.m20.velocity_controller_dds.socket.socket",
        side_effect=[first_socket, second_socket],
    ):
        controller = M20VelocityController(aos_host="10.0.0.5")
        controller._init_publisher()

        assert controller._socket is None

        cmd = NavCmd.from_twist(Twist(linear=(0.4, 0.0, 0.0), angular=(0.0, 0.0, 0.2)))
        controller._publish_once(cmd)

    second_socket.connect.assert_called_once_with(("10.0.0.5", NAV_CMD_BRIDGE_PORT))
    second_socket.sendall.assert_called_once()
