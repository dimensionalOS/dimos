from unittest.mock import MagicMock, patch

from dimos.core.transport import LCMTransport


class _FakeMsg:
    msg_name = "test.FakeMsg"


def test_lcm_transport_broadcast_does_not_start_receiver_loop() -> None:
    mock_pubsub = MagicMock()

    with patch("dimos.core.transport.LCM", return_value=mock_pubsub):
        transport = LCMTransport("/cmd_vel", _FakeMsg)
        msg = object()

        transport.broadcast(None, msg)

    mock_pubsub.publish.assert_called_once_with(transport.topic, msg)
    mock_pubsub.start.assert_not_called()
    assert transport._started is False


def test_lcm_transport_subscribe_starts_receiver_loop() -> None:
    mock_pubsub = MagicMock()

    with patch("dimos.core.transport.LCM", return_value=mock_pubsub):
        transport = LCMTransport("/cmd_vel", _FakeMsg)

        transport.subscribe(lambda _msg: None)

    mock_pubsub.start.assert_called_once()
    mock_pubsub.subscribe.assert_called_once()
    assert transport._started is True
