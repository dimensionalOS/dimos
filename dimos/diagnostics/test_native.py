# Copyright 2025-2026 Dimensional Inc.
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

"""Python → native async queue/handler → Python, over isolated real Zenoh sessions."""

from contextlib import ExitStack
import os
from pathlib import Path
from queue import Queue
import selectors
import socket
import subprocess
import threading

import pytest
import zenoh

from dimos.diagnostics.analysis import load_bundle
from dimos.diagnostics.model import TraceContext


@pytest.fixture
def native_peer(tmp_path):
    executable = Path(__file__).resolve().parents[2] / "target/debug/examples/diagnostic_echo"
    if not executable.is_file():
        pytest.skip(
            "build native/rust/dimos-module example diagnostic_echo for Python↔Rust qualification"
        )
    with ExitStack() as cleanup:
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            endpoint = f"tcp/127.0.0.1:{reservation.getsockname()[1]}"
        config = zenoh.Config()
        config.insert_json5("scouting/multicast/enabled", "false")
        config.insert_json5("scouting/gossip/enabled", "false")
        config.insert_json5("listen/endpoints", f'["{endpoint}"]')
        session = zenoh.open(config)
        cleanup.callback(session.close)
        messages = Queue()
        subscriber = session.declare_subscriber("native-conformance/output", messages.put)
        cleanup.callback(subscriber.undeclare)
        publisher = session.declare_publisher("native-conformance/input")
        cleanup.callback(publisher.undeclare)
        matched = threading.Event()
        listener = publisher.declare_matching_listener(
            lambda status: matched.set() if status.matching else None
        )
        cleanup.callback(listener.undeclare)
        env = {
            **os.environ,
            "DIMOS_DIAGNOSTICS": "trace",
            "DIMOS_DIAGNOSTICS_RUN": "native-test",
            "DIMOS_DIAGNOSTICS_DIR": str(tmp_path),
            "DIMOS_DIAGNOSTICS_TEST_ENDPOINT": endpoint,
            "DIMOS_DIAGNOSTICS_TEST_KEY": "native-conformance",
        }
        process = subprocess.Popen(
            [str(executable)], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )

        def close():
            if process.poll() is None:
                process.terminate()
            process.communicate(timeout=5)

        cleanup.callback(close)
        selector = cleanup.enter_context(selectors.DefaultSelector())
        selector.register(process.stdout, selectors.EVENT_READ)
        assert selector.select(timeout=10), "native peer did not start"
        assert process.stdout.readline() == b"ready\n"
        assert matched.wait(10), "native input subscription did not propagate"
        yield publisher, messages, process, tmp_path


def test_python_rust_queue_parent_identity_and_shutdown_flush(native_peer):
    publisher, subscriber, process, directory = native_peer
    parent = TraceContext("native-test", "python-\u03b1", "hand/左", 1, 1234, "test/monotonic")
    payload = b"identical\x00\xffpayload"
    publisher.put(payload, attachment=b"DIMOS_TRACE_V1\0" + parent.encode())
    sample = subscriber.get(timeout=10)
    assert bytes(sample.payload) == payload
    context = TraceContext.decode(bytes(sample.attachment).removeprefix(b"DIMOS_TRACE_V1\0"))
    assert context.parents == (parent.sample_id,)
    assert context.run_id == parent.run_id
    assert context.publisher != parent.publisher
    assert process.wait(timeout=10) == 0
    manifests, events = load_bundle(directory)
    assert manifests
    handler = [event for event in events if event["event"] == "native.handler.start"]
    assert handler and handler[0]["context"]["publisher"] == parent.publisher
    assert any(event["event"] == "native.transport_publish.end" for event in events)
