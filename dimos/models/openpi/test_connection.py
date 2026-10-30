# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

import numpy as np
from openpi_client import msgpack_numpy
import pytest

from dimos.models.openpi.connection import OpenPIConnection


def test_inference_uses_openpi_wire_format_and_timeout(mocker):
    socket = mocker.Mock()
    socket.recv.side_effect = [
        msgpack_numpy.packb({"action_shape": [16, 14]}),
        msgpack_numpy.packb({"actions": np.zeros((16, 14))}),
    ]
    mocker.patch("dimos.models.openpi.connection.connect", return_value=socket)
    client = OpenPIConnection("127.0.0.1", 8000, 2.0)
    try:
        result = client.infer({"state": np.arange(14)})
        np.testing.assert_array_equal(result["actions"], np.zeros((16, 14)))
        np.testing.assert_array_equal(
            msgpack_numpy.unpackb(socket.send.call_args.args[0])["state"], np.arange(14)
        )
        socket.recv.assert_called_with(timeout=2.0)
    finally:
        client.close()


def test_timeout_closes_connection_to_discard_late_reply(mocker):
    socket = mocker.Mock()
    socket.recv.side_effect = [msgpack_numpy.packb({}), TimeoutError("inference timeout")]
    mocker.patch("dimos.models.openpi.connection.connect", return_value=socket)
    client = OpenPIConnection("127.0.0.1", 8000, 2.0)
    with pytest.raises(TimeoutError):
        client.infer({"state": np.zeros(14)})
    socket.close.assert_called_once()
