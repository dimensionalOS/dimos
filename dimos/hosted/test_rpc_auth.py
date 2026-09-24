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

"""Host RPC authentication tests with local, harmless payloads."""

import base64
from pathlib import Path
import pickle

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
import pytest
from pytest_mock import MockerFixture

from dimos.hosted.rpc_auth import HostRpcAuth
from dimos.protocol.rpc.zenohrpc import ZenohRPC


def _write_marker(path: str) -> str:
    Path(path).write_text("deserialized")
    return "marker"


class _MarkerOnLoad:
    def __init__(self, path: Path) -> None:
        self.path = path

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (_write_marker, (str(self.path),))


@pytest.fixture
def auth_pair() -> tuple[HostRpcAuth, HostRpcAuth]:
    client_key = Ed25519PrivateKey.generate()
    server_key = Ed25519PrivateKey.generate()
    client = HostRpcAuth(
        signing_key=client_key,
        verification_key=server_key.public_key(),
    )
    server = HostRpcAuth(
        signing_key=server_key,
        verification_key=client_key.public_key(),
    )
    return client, server


def test_unsigned_request_is_rejected_before_pickle_loads(
    tmp_path: Path, mocker: MockerFixture, auth_pair: tuple[HostRpcAuth, HostRpcAuth]
) -> None:
    marker = tmp_path / "rpc-marker"
    query = mocker.MagicMock()
    query.payload.to_bytes.return_value = pickle.dumps(([_MarkerOnLoad(marker)], {}))
    query.attachment = None
    handler = mocker.MagicMock()
    _, server = auth_pair
    rpc = ZenohRPC(payload_auth=server)

    rpc._execute_rpc(handler, "hosts/host-1/start", query)

    assert not marker.exists()
    handler.assert_not_called()
    query.reply_err.assert_called_once()


def test_signed_request_runs_handler_and_signs_reply(
    mocker: MockerFixture, auth_pair: tuple[HostRpcAuth, HostRpcAuth]
) -> None:
    client, server = auth_pair
    name = "hosts/host-1/start"
    query = mocker.MagicMock()
    query.payload.to_bytes.return_value = client.seal("request", name, pickle.dumps(([42], {})))
    query.attachment = None
    handler = mocker.MagicMock(return_value="accepted")
    rpc = ZenohRPC(payload_auth=server)

    rpc._execute_rpc(handler, name, query)

    handler.assert_called_once_with(42)
    signed_reply = query.reply.call_args.args[1]
    assert pickle.loads(client.open("reply", name, signed_reply)) == "accepted"


def test_unsigned_reply_is_rejected_before_pickle_loads(
    tmp_path: Path, mocker: MockerFixture, auth_pair: tuple[HostRpcAuth, HostRpcAuth]
) -> None:
    marker = tmp_path / "reply-marker"
    reply = mocker.MagicMock()
    reply.err = None
    reply.ok.payload.to_bytes.return_value = pickle.dumps(_MarkerOnLoad(marker))
    session = mocker.MagicMock()
    session.get.side_effect = lambda _key, callback, **_kwargs: callback(reply)
    mocker.patch(
        "dimos.protocol.rpc.zenohrpc.zenoh.handlers.Callback",
        side_effect=lambda callback, drop: callback,
    )
    client, _ = auth_pair
    rpc = ZenohRPC(payload_auth=client)
    mocker.patch.object(rpc, "_session", session)
    received: list[object] = []

    rpc.call_cb("hosts/host-1/describe", ([], {}), received.append)

    assert not marker.exists()
    assert len(received) == 1
    assert isinstance(received[0], ValueError)


def test_signature_binds_method_and_payload(
    auth_pair: tuple[HostRpcAuth, HostRpcAuth],
) -> None:
    client, server = auth_pair
    signed = client.seal("request", "hosts/host-1/start", b"payload")

    with pytest.raises(ValueError, match="signature"):
        server.open("request", "hosts/host-2/start", signed)
    with pytest.raises(ValueError, match="signature"):
        server.open("request", "hosts/host-1/start", signed + b"tampered")


def test_host_and_client_environment_keys_have_separate_roles(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client_key = Ed25519PrivateKey.generate()
    server_key = Ed25519PrivateKey.generate()
    monkeypatch.setenv(
        "DIMOS_HOST_CLIENT_SIGNING_KEY",
        base64.b64encode(client_key.private_bytes_raw()).decode(),
    )
    monkeypatch.setenv(
        "DIMOS_HOST_CLIENT_VERIFY_KEY",
        base64.b64encode(client_key.public_key().public_bytes_raw()).decode(),
    )
    monkeypatch.setenv(
        "DIMOS_HOST_SERVER_SIGNING_KEY",
        base64.b64encode(server_key.private_bytes_raw()).decode(),
    )
    monkeypatch.setenv(
        "DIMOS_HOST_SERVER_VERIFY_KEY",
        base64.b64encode(server_key.public_key().public_bytes_raw()).decode(),
    )

    client = HostRpcAuth.from_client_env()
    server = HostRpcAuth.from_host_env()

    assert (
        server.open(
            "request", "hosts/host-1/start", client.seal("request", "hosts/host-1/start", b"start")
        )
        == b"start"
    )
    assert (
        client.open(
            "reply", "hosts/host-1/start", server.seal("reply", "hosts/host-1/start", b"ok")
        )
        == b"ok"
    )
