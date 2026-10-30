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

"""The native renderer wire contract and ownership of its one subprocess."""

from concurrent.futures import ThreadPoolExecutor, wait
from contextlib import suppress
import json
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time

import numpy as np
import pytest

from dimos.sim2.unreal.client import (
    IDENTITY_FIELDS,
    MAX_HEADER_BYTES,
    RendererClient,
    RendererProcess,
)


def encode_header(value):
    payload = json.dumps(value, separators=(",", ":")).encode()
    return struct.pack("<I", len(payload)) + payload


def receive_exact(connection, size):
    result = bytearray()
    while len(result) < size:
        chunk = connection.recv(size - len(result))
        if not chunk:
            raise ConnectionError("test renderer disconnected")
        result.extend(chunk)
    return bytes(result)


def receive_command(connection):
    size = struct.unpack("<I", receive_exact(connection, 4))[0]
    return json.loads(receive_exact(connection, size))


@pytest.fixture
def workers():
    with ThreadPoolExecutor(max_workers=2) as executor:
        yield executor


@pytest.fixture
def peers():
    connection, renderer = socket.socketpair()
    connection.settimeout(1)
    renderer.settimeout(1)
    client = RendererClient(connection)
    try:
        yield client, renderer
    finally:
        client.close()
        renderer.close()


@pytest.fixture
def capture_command():
    return {
        "episode": 1,
        "sequence": 7,
        "physics_tick": 120,
        "sim_time": 0.24,
        "timestamp": 1750000000.25,
        "poses": [],
        "camera": {"width": 2, "height": 1},
    }


@pytest.fixture
def frame_header(capture_command):
    return {
        "type": "frame",
        **{key: capture_command[key] for key in IDENTITY_FIELDS},
        "width": 2,
        "height": 1,
        "rgba_bytes": 8,
        "depth_bytes": 8,
        "depth_unit": "m",
        "depth_kind": "axial",
        "capture_ms": 3.5,
    }


def test_capture_reassembles_fragmented_aligned_rgba_and_metric_depth(
    workers, peers, capture_command, frame_header
):
    client, renderer = peers
    pixels = bytes([255, 0, 10, 255, 20, 30, 40, 255])
    distances = struct.pack("<ff", 1.25, float("inf"))

    def render():
        command = receive_command(renderer)
        payload = encode_header(frame_header) + pixels + distances
        for start in range(0, len(payload), 3):
            renderer.sendall(payload[start : start + 3])
        return command

    response = workers.submit(render)
    frame = client.capture(capture_command)

    assert response.result(timeout=2) == {**capture_command, "type": "capture"}
    assert frame.header == frame_header
    np.testing.assert_array_equal(frame.rgba, [[[255, 0, 10, 255], [20, 30, 40, 255]]])
    np.testing.assert_array_equal(frame.depth, [[1.25, float("inf")]])
    assert frame.depth.dtype == np.float32
    assert frame.roundtrip_seconds >= 0


@pytest.mark.parametrize("key", IDENTITY_FIELDS)
def test_capture_rejects_images_from_another_snapshot(peers, capture_command, frame_header, key):
    client, renderer = peers
    renderer.sendall(encode_header({**frame_header, key: frame_header[key] + 1}))

    with pytest.raises(ValueError, match="different physics snapshot"):
        client.capture(capture_command)


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"episode": True}, "different physics snapshot"),
        ({"sequence": 7.0}, "different physics snapshot"),
        ({"width": 3}, "wrong image dimensions"),
        ({"height": True}, "wrong image dimensions"),
        ({"width": 2.0}, "wrong image dimensions"),
        ({"rgba_bytes": 7}, "payload sizes"),
        ({"depth_bytes": 8.0}, "payload sizes"),
        ({"depth_unit": "cm"}, "depth in meters"),
        ({"depth_kind": "range"}, "depth in meters"),
    ],
)
def test_capture_rejects_incompatible_metadata_before_reading_pixels(
    peers, capture_command, frame_header, change, message
):
    client, renderer = peers
    renderer.sendall(encode_header({**frame_header, **change}))

    with pytest.raises(ValueError, match=message):
        client.capture(capture_command)


@pytest.mark.parametrize("width", [0, -1, 4097, 2.0, True])
def test_invalid_camera_dimensions_do_not_send_a_request(peers, capture_command, width):
    client, renderer = peers
    capture_command["camera"]["width"] = width

    with pytest.raises(ValueError, match="camera dimensions"):
        client.capture(capture_command)

    renderer.setblocking(False)
    with pytest.raises(BlockingIOError):
        renderer.recv(1)


@pytest.mark.parametrize("distance", [float("nan"), -0.1, float("-inf")])
def test_capture_rejects_invalid_depth_distances(peers, capture_command, frame_header, distance):
    client, renderer = peers
    renderer.sendall(encode_header(frame_header) + bytes(8) + struct.pack("<ff", 0, distance))

    with pytest.raises(ValueError, match="NaN or negative"):
        client.capture(capture_command)


@pytest.mark.parametrize("part", ["prefix", "header", "rgba", "depth"])
def test_capture_reports_truncated_responses(peers, capture_command, frame_header, part):
    client, renderer = peers
    header = encode_header(frame_header)
    truncated = {
        "prefix": header[:2],
        "header": header[:-1],
        "rgba": header + bytes(7),
        "depth": header + bytes(8) + bytes(7),
    }[part]
    renderer.sendall(truncated)
    renderer.shutdown(socket.SHUT_WR)

    with pytest.raises(ConnectionError, match="before completing"):
        client.capture(capture_command)


@pytest.mark.parametrize("size", [0, MAX_HEADER_BYTES + 1])
def test_rejects_unbounded_response_header_lengths(peers, size):
    client, renderer = peers
    renderer.sendall(struct.pack("<I", size))

    with pytest.raises(ValueError, match="header length"):
        client.receive_header()


@pytest.mark.parametrize(
    ("header", "error", "message"),
    [
        ([], ValueError, "JSON object"),
        ({"type": "error", "message": "bad mesh"}, RuntimeError, "bad mesh"),
    ],
)
def test_reports_renderer_protocol_errors(peers, header, error, message):
    client, renderer = peers
    renderer.sendall(encode_header(header))

    with pytest.raises(error, match=message):
        client.receive_header()


def test_stalled_capture_times_out_and_preserves_configured_timeout(peers, capture_command):
    client, _ = peers
    client.connection.settimeout(0.02)
    started = time.monotonic()

    with pytest.raises(TimeoutError):
        client.capture(capture_command)

    assert time.monotonic() - started < 1
    assert client.connection.gettimeout() == 0.02


def test_close_interrupts_a_command_blocked_by_native_backpressure(workers, peers, capture_command):
    client, native = peers
    client.connection.settimeout(15)
    client.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)
    capture_command["poses"] = [0] * 100000
    capture = workers.submit(client.capture, capture_command)
    assert struct.unpack("<I", receive_exact(native, 4))[0] > 100000

    client.close()

    assert wait([capture], timeout=2).done == {capture}
    with pytest.raises(OSError):
        capture.result()


def test_load_resolves_manifest_and_reset_waits_for_matching_episode(workers, peers, tmp_path):
    client, native = peers
    manifest = tmp_path / "scene.json"

    def respond():
        load = receive_command(native)
        native.sendall(encode_header({"type": "loaded"}))
        reset = receive_command(native)
        native.sendall(encode_header({"type": "reset", "episode": 4}))
        return load, reset

    response = workers.submit(respond)
    client.load(manifest)
    client.reset(4)

    assert response.result(timeout=2) == (
        {"type": "load", "manifest": str(manifest.resolve())},
        {"type": "reset", "episode": 4},
    )


def test_load_restores_capture_timeout_after_success(peers, tmp_path):
    client, native = peers
    client.connection.settimeout(15)
    native.sendall(encode_header({"type": "loaded"}))

    client.load(tmp_path / "scene.json", timeout=180)

    assert client.connection.gettimeout() == 15
    assert receive_command(native) == {
        "type": "load",
        "manifest": str(tmp_path / "scene.json"),
    }


def test_load_restores_capture_timeout_after_native_failure(peers, tmp_path):
    client, native = peers
    client.connection.settimeout(15)
    native.sendall(encode_header({"type": "error", "message": "unsupported material"}))

    with pytest.raises(RuntimeError, match="unsupported material"):
        client.load(tmp_path / "scene.json", timeout=180)

    assert client.connection.gettimeout() == 15


def test_load_uses_its_deadline_without_changing_capture_timeout(peers, tmp_path):
    client, _ = peers
    client.connection.settimeout(15)
    started = time.monotonic()

    with pytest.raises(TimeoutError, match="deadline"):
        client.load(tmp_path / "scene.json", timeout=0.02)

    assert time.monotonic() - started < 1
    assert client.connection.gettimeout() == 15


def test_close_interrupts_cold_asset_load(workers, peers, tmp_path):
    client, native = peers
    loading = workers.submit(client.load, tmp_path / "scene.json", timeout=180)
    assert receive_command(native)["type"] == "load"

    client.close()

    assert wait([loading], timeout=2).done == {loading}
    with pytest.raises(OSError):
        loading.result()


@pytest.mark.parametrize("timeout", [0, -1, float("inf"), float("nan")])
def test_load_rejects_unbounded_or_nonpositive_timeout_before_sending(peers, tmp_path, timeout):
    client, native = peers

    with pytest.raises(ValueError, match="positive and finite"):
        client.load(tmp_path / "scene.json", timeout=timeout)

    native.setblocking(False)
    with pytest.raises(BlockingIOError):
        native.recv(1)


@pytest.mark.parametrize("episode", [2, True])
def test_reset_rejects_wrong_or_boolean_episode_acknowledgment(peers, episode):
    client, native = peers
    native.sendall(encode_header({"type": "reset", "episode": episode}))

    with pytest.raises(ValueError, match="wrong episode"):
        client.reset(1)


@pytest.fixture
def native_process(mocker):
    process = mocker.Mock(spec=subprocess.Popen)
    process.poll.return_value = None
    process.wait.return_value = 0
    sockets = []
    ready = {"type": "ready", "version": 1}
    launched = threading.Event()

    def launch(args, **kwargs):
        port = int(next(arg.split("=", 1)[1] for arg in args if arg.startswith("-DimosPort=")))
        connection = socket.create_connection(("127.0.0.1", port), timeout=1)
        sockets.append(connection)
        if ready:
            connection.sendall(encode_header(ready))
        launched.set()
        return process

    popen = mocker.patch("dimos.sim2.unreal.client.subprocess.Popen", side_effect=launch)
    try:
        yield popen, process, sockets, ready, launched
    finally:
        for connection in sockets:
            with suppress(OSError):
                connection.shutdown(socket.SHUT_RDWR)
            connection.close()


@pytest.fixture
def renderer(tmp_path, native_process):
    executable = tmp_path / "UnrealRenderer"
    executable.touch()
    owner = RendererProcess(executable, tmp_path / "logs" / "renderer.log", startup_timeout=1)
    try:
        yield owner
    finally:
        owner.cancel()
        owner.close()


def test_owned_process_shutdown_is_acknowledged_and_reaped(workers, renderer, native_process):
    popen, process, sockets, _, _ = native_process
    client = renderer.start()

    def acknowledge_shutdown():
        command = receive_command(sockets[0])
        sockets[0].sendall(encode_header({"type": "shutdown"}))
        return command

    response = workers.submit(acknowledge_shutdown)
    renderer.close()

    assert response.result(timeout=2) == {"type": "shutdown"}
    process.wait.assert_called_once_with(timeout=5.0)
    process.terminate.assert_not_called()
    assert popen.call_args.kwargs["stdout"].closed
    assert client.connection.fileno() == -1
    assert renderer.process is None
    assert renderer.client is None


@pytest.mark.parametrize("version", [2, True, "1"])
def test_bad_native_handshake_terminates_owned_process_and_closes_log(
    renderer, native_process, version
):
    popen, process, _, ready, _ = native_process
    ready["version"] = version

    with pytest.raises(ValueError, match="unsupported Unreal renderer handshake"):
        renderer.start()

    process.terminate.assert_called_once_with()
    process.wait.assert_called_once_with(timeout=5.0)
    assert popen.call_args.kwargs["stdout"].closed


def test_cancel_before_start_does_not_launch_a_process(renderer, native_process):
    popen, _, _, _, _ = native_process
    renderer.cancel()

    with pytest.raises(RuntimeError, match="startup cancelled"):
        renderer.start()

    popen.assert_not_called()


def test_cancel_interrupts_startup_with_incomplete_handshake(workers, renderer, native_process):
    popen, process, _, ready, launched = native_process
    ready.clear()
    startup = workers.submit(renderer.start)
    assert launched.wait(timeout=1)

    renderer.cancel()

    assert wait([startup], timeout=2).done == {startup}
    with pytest.raises((RuntimeError, OSError)):
        startup.result()
    process.terminate.assert_called()
    assert popen.call_args.kwargs["stdout"].closed


def test_close_interrupts_startup_and_owns_final_cleanup(workers, renderer, native_process):
    popen, process, _, ready, launched = native_process
    ready.clear()
    startup = workers.submit(renderer.start)
    assert launched.wait(timeout=1)

    renderer.close()

    assert wait([startup], timeout=2).done == {startup}
    with pytest.raises((RuntimeError, OSError)):
        startup.result()
    process.wait.assert_called_once_with(timeout=5.0)
    assert popen.call_args.kwargs["stdout"].closed


def test_cancellation_during_process_creation_still_terminates_late_child(
    workers, renderer, native_process
):
    popen, process, _, _, _ = native_process
    entered, finish_launch = threading.Event(), threading.Event()
    launch = popen.side_effect

    def delayed_launch(*args, **kwargs):
        entered.set()
        assert finish_launch.wait(timeout=2)
        return launch(*args, **kwargs)

    popen.side_effect = delayed_launch
    startup = workers.submit(renderer.start)
    try:
        assert entered.wait(timeout=1)
        renderer.cancel()
    finally:
        finish_launch.set()

    assert wait([startup], timeout=2).done == {startup}
    with pytest.raises(RuntimeError, match="startup cancelled"):
        startup.result()
    process.terminate.assert_called_once_with()
    process.wait.assert_called_once_with(timeout=5.0)
    assert popen.call_args.kwargs["stdout"].closed


def test_startup_deadline_bounds_wait_for_native_connection(renderer, native_process):
    popen, process, _, _, _ = native_process
    popen.side_effect = None
    popen.return_value = process
    renderer.startup_timeout = 0.02
    started = time.monotonic()

    with pytest.raises(TimeoutError, match="did not connect"):
        renderer.start()

    assert time.monotonic() - started < 1
    process.terminate.assert_called_once_with()
    assert popen.call_args.kwargs["stdout"].closed


def test_cancel_interrupts_capture_then_reaps_owned_process(
    workers, renderer, native_process, capture_command
):
    popen, process, sockets, _, _ = native_process
    client = renderer.start()
    capturing = workers.submit(client.capture, capture_command)
    assert receive_command(sockets[0])["type"] == "capture"

    renderer.cancel()
    assert wait([capturing], timeout=2).done == {capturing}
    with pytest.raises(OSError):
        capturing.result()
    renderer.close()

    process.terminate.assert_called()
    process.wait.assert_called_once_with(timeout=5.0)
    assert popen.call_args.kwargs["stdout"].closed


def test_cancel_kills_only_owned_process_if_termination_times_out(renderer, native_process):
    _, process, _, _, _ = native_process
    renderer.start()
    process.wait.side_effect = [subprocess.TimeoutExpired("UnrealRenderer", 5), 0]

    renderer.cancel()
    renderer.close()

    process.kill.assert_called_once_with()
    assert process.wait.call_count == 2


@pytest.mark.parametrize("timeout", [0, -1, float("inf"), float("nan")])
def test_rejects_unbounded_or_nonpositive_timeouts(tmp_path, timeout):
    with pytest.raises(ValueError, match="positive and finite"):
        RendererProcess(Path("unreal"), tmp_path / "log", startup_timeout=timeout)
