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

from concurrent.futures import ThreadPoolExecutor
from threading import Event

import cv2
import msgpack
import numpy as np
import pytest
import zmq

from dimos.robot.unitree.g1.sonic_policy_client import SonicPolicyClient

KEY = "observation.images.egocentric"
INFO = {"image_keys": [KEY], "n_obs": 1, "shapes": {KEY: [3, 8, 12]}}


@pytest.fixture
def request_args():
    image = np.zeros((8, 12, 3), dtype=np.uint8)
    image[:] = [230, 80, 20]
    return dict(
        task_id="pick-1",
        sequence=3,
        observed_at=10.25,
        states=np.arange(28, dtype=np.float32)[None],
        images=[{KEY: image}],
        task="pick drink bottle from the table and handover",
    )


def test_real_wire_preserves_rgb_state_and_observation_identity(tmp_path, request_args):
    endpoint = f"ipc://{tmp_path}/policy.sock"
    ready = Event()
    actions = np.arange(156, dtype=np.float32).reshape(2, 78) / 1000

    def serve():
        with zmq.Context() as ctx, ctx.socket(zmq.REP) as sock:
            sock.setsockopt(zmq.LINGER, 0)
            sock.bind(endpoint)
            ready.set()
            assert sock.poll(2000)
            assert msgpack.unpackb(sock.recv(), raw=False) == {"op": "info"}
            sock.send(msgpack.packb(INFO, use_bin_type=True))
            assert sock.poll(2000)
            request = msgpack.unpackb(sock.recv(), raw=False)
            sock.send(
                msgpack.packb(
                    dict(chunk=actions.tobytes(), shape=[2, 78], latency_s=0.03), use_bin_type=True
                )
            )
            return request

    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(serve)
        assert ready.wait(2)
        client = SonicPolicyClient(endpoint, timeout_seconds=1)
        info = client.get_info()
        chunk = client.infer(info=info, **request_args)
        request = future.result(timeout=2)
    assert (chunk.task_id, chunk.sequence, chunk.observed_at) == ("pick-1", 3, 10.25)
    np.testing.assert_array_equal(chunk.actions, actions)
    assert chunk.server_latency_seconds == 0.03
    assert chunk.roundtrip_seconds > 0
    assert request["states"] == [list(range(28))]
    assert request["task"] == request_args["task"]
    assert "rtc" not in request
    decoded = cv2.imdecode(np.frombuffer(request["images"][0][KEY], np.uint8), cv2.IMREAD_COLOR)
    np.testing.assert_allclose(decoded[0, 0], [230, 80, 20], atol=3)


@pytest.mark.parametrize(
    "change",
    [
        {"n_obs": 0},
        {"n_obs": True},
        {"image_keys": [KEY, KEY]},
        {"shapes": {KEY: [1, 8, 12]}},
        {"shapes": {KEY: [3, -1, 12]}},
        {"shapes": {}},
    ],
)
def test_rejects_incompatible_server_info(mocker, change):
    client = SonicPolicyClient("ipc:///unused")
    mocker.patch.object(client, "_request", return_value=INFO | change)
    with pytest.raises(ValueError):
        client.get_info()


@pytest.mark.parametrize(
    "reply",
    [
        {"error": "model failed"},
        {"shape": [2, 28], "chunk": bytes(224), "latency_s": 0.1},
        {"shape": [0, 78], "chunk": b"", "latency_s": 0.1},
        {"shape": [True, 78], "chunk": bytes(312), "latency_s": 0.1},
        {"shape": [2, 78], "chunk": bytes(312), "latency_s": 0.1},
        {"shape": [1, 78], "chunk": np.full(78, np.nan, np.float32).tobytes(), "latency_s": 0.1},
        {"shape": [1, 78], "chunk": bytes(312), "latency_s": -1},
    ],
)
def test_bad_predictions_never_become_chunks(mocker, request_args, reply):
    client = SonicPolicyClient("ipc:///unused")
    ask = mocker.patch.object(client, "_request", return_value=INFO)
    info = client.get_info()
    ask.return_value = reply
    with pytest.raises((ValueError, RuntimeError)):
        client.infer(info=info, **request_args)


@pytest.mark.parametrize(
    "change",
    [
        {"states": np.zeros((1, 29))},
        {"states": np.full((1, 28), np.inf)},
        {"images": [{KEY: np.zeros((8, 12, 3), dtype=np.float32)}]},
        {"images": [{}]},
        {"observed_at": float("nan")},
        {"sequence": True},
    ],
)
def test_bad_observations_are_rejected_before_network(mocker, request_args, change):
    client = SonicPolicyClient("ipc:///unused")
    ask = mocker.patch.object(client, "_request", return_value=INFO)
    info = client.get_info()
    ask.reset_mock()
    with pytest.raises(ValueError):
        client.infer(info=info, **(request_args | change))
    ask.assert_not_called()


def test_timeout_does_not_poison_next_request(tmp_path):
    endpoint = f"ipc://{tmp_path}/missing.sock"
    client = SonicPolicyClient(endpoint, timeout_seconds=0.02)
    with pytest.raises(TimeoutError):
        client.get_info()
    # A second request must time out normally, not fail with REQ's wrong-state error.
    with pytest.raises(TimeoutError):
        client.get_info()
