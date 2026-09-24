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

"""Harmless local proof of the Host control deserialization boundary."""

import hashlib
from pathlib import Path
import pickle

import pytest
from pytest_mock import MockerFixture

from dimos.hosted.fragment import HostFragment
from dimos.protocol.rpc.zenohrpc import ZenohRPC


def _write_marker(path: str) -> str:
    Path(path).write_text("deserialized")
    return "marker"


class _MarkerOnLoad:
    def __init__(self, path: Path) -> None:
        self.path = path

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (_write_marker, (str(self.path),))


def test_forged_digest_does_not_authorize_fragment_payload(tmp_path: Path) -> None:
    marker = tmp_path / "fragment-marker"
    payload = pickle.dumps(_MarkerOnLoad(marker))
    fragment = HostFragment(
        run_id="local-proof",
        generation=1,
        host_id="host-1",
        application_name="local-proof",
        application_revision="local-proof",
        payload_digest=hashlib.sha256(payload).hexdigest(),
        payload=payload,
    )

    with pytest.raises(TypeError, match="PythonFragmentPayload"):
        fragment.load_payload()

    assert marker.read_text() == "deserialized"


def test_host_rpc_deserializes_before_calling_handler(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    marker = tmp_path / "rpc-marker"
    query = mocker.MagicMock()
    query.payload.to_bytes.return_value = pickle.dumps(([_MarkerOnLoad(marker)], {}))
    query.attachment = None
    handler = mocker.MagicMock(return_value="accepted")
    rpc = ZenohRPC()

    rpc._execute_rpc(handler, "hosts/host-1/start", query)

    assert marker.read_text() == "deserialized"
    handler.assert_called_once_with("marker")
