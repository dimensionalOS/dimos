# Copyright 2026 Dimensional Inc.
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

import pickle

import pytest

from dimos.protocol.rpc.rpc_utils import RemoteError, deserialize_exception
from dimos.protocol.rpc.zenohrpc import EXCEPTION_ENCODING, ZenohRPC


@pytest.fixture
def rpc():
    service = ZenohRPC()
    yield service
    service.stop()


def test_invalid_rpc_payload_returns_an_error_instead_of_timing_out(rpc, mocker):
    query = mocker.Mock()
    query.attachment = None
    query.payload.to_bytes.return_value = b"not a pickle"
    handler = mocker.Mock()

    rpc._execute_rpc(handler, "demo/move", query)

    handler.assert_not_called()
    query.reply_err.assert_called_once()
    encoded = query.reply_err.call_args.args[0]
    error = deserialize_exception(pickle.loads(encoded))
    assert isinstance(error, RemoteError)
    assert error.remote_type == "_pickle.UnpicklingError"
    assert query.reply_err.call_args.kwargs["encoding"] == EXCEPTION_ENCODING
    query.drop.assert_called_once_with()
