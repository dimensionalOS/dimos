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

"""Real local ZMQ contract tests; the peer simulates protocol state, not physics."""

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import threading
import time

import msgpack
import numpy as np
import pytest
import zmq

from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_session import SonicModelProfile
from dimos.utils.testing.waiting import wait_until


class ScriptedPeer:
    def __init__(self):
        self.ready = threading.Event()
        self.connected = threading.Event()
        self.stopping = threading.Event()
        self.pause_feedback = threading.Event()
        self.drop_pose_subscription = threading.Event()
        self.lock = threading.Lock()
        self.packets = []
        self.error = None
        self.feedback_endpoint = None
        self.reference_endpoint = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        try:
            with zmq.Context() as ctx, ctx.socket(zmq.PUB) as pub, ctx.socket(zmq.SUB) as sub:
                pub.setsockopt(zmq.LINGER, 0)
                sub.setsockopt(zmq.LINGER, 0)
                sub.setsockopt(zmq.SUBSCRIBE, b"command")
                sub.setsockopt(zmq.SUBSCRIBE, b"pose")
                port = pub.bind_to_random_port("tcp://127.0.0.1")
                self.feedback_endpoint = f"tcp://127.0.0.1:{port}"
                self.ready.set()
                if not self.connected.wait(5) or self.stopping.is_set():
                    return
                sub.connect(self.reference_endpoint)
                planner, epoch, accepted, index, next_feedback = True, 1, -1, 0, 0.0
                while not self.stopping.is_set():
                    if self.drop_pose_subscription.is_set():
                        sub.setsockopt(zmq.UNSUBSCRIBE, b"pose")
                        self.drop_pose_subscription.clear()
                    if sub.poll(3):
                        packet = sub.recv()
                        with self.lock:
                            self.packets.append(packet)
                        if packet.startswith(b"command"):
                            selected = bool(packet[-1])
                            if planner and not selected:
                                epoch += 1
                            planner = selected
                        elif not planner:
                            accepted = int(np.frombuffer(packet[1540:1548], dtype="<i8")[0])
                    now = time.monotonic()
                    if now >= next_feedback and not self.pause_feedback.is_set():
                        index += 1
                        pub.send(
                            b"g1_debug"
                            + msgpack.packb(
                                {
                                    "index": index,
                                    "ros_timestamp": 0.0,
                                    "body_q": [0.0] * 29,
                                    "body_dq": [0.0] * 29,
                                    "left_hand_q": [0.1] * 7,
                                    "left_hand_dq": [0.0] * 7,
                                    "right_hand_q": [0.2] * 7,
                                    "right_hand_dq": [0.0] * 7,
                                    "token_state": [0.125] * 64,
                                    "reference_guard": {
                                        "armed": not planner,
                                        "expired": False,
                                        "planner_selected": planner,
                                        "planner_confirmed": planner,
                                        "rearm_requires_planner": False,
                                        "accepted_index": accepted,
                                        "applied_index": accepted,
                                        "rejected": 0,
                                        "epoch": epoch,
                                        "age_ms": 0.0,
                                    },
                                }
                            )
                        )
                        next_feedback = now + 0.01
        except Exception as exc:
            self.error = exc
            self.ready.set()

    def snapshot(self):
        with self.lock:
            return list(self.packets)


@pytest.fixture
def peer():
    peer = ScriptedPeer()
    peer.thread.start()
    try:
        assert peer.ready.wait(5)
        assert peer.error is None
        yield peer
    finally:
        peer.stopping.set()
        peer.connected.set()
        peer.thread.join(timeout=3)
        assert not peer.thread.is_alive()
        assert peer.error is None


@pytest.fixture
def execution_config(peer, tmp_path):
    return dict(
        reference_bind="tcp://127.0.0.1:*",
        feedback_endpoint=peer.feedback_endpoint,
        journal_dir=tmp_path,
        profile=SonicModelProfile(
            name="scripted-peer",
            source_fps=50,
            right_hand_order="native",
            handback_policy="native_planner",
            seed_seconds=0.06,
            blend_seconds=0.04,
            first_chunk_timeout=1.0,
            handback_timeout=0.4,
        ),
    )


@pytest.fixture
def execution(peer, execution_config):
    module = SonicExecutionModule(**execution_config)
    try:
        module.start()
        peer.reference_endpoint = module.get_status()["reference_endpoint"]
        peer.connected.set()
        wait_until(lambda: module.get_status()["ready"], timeout=3)
        yield module, peer
    finally:
        module.stop()


@pytest.fixture
def worker_execution(peer, execution_config, dimos_cluster):
    module = dimos_cluster.deploy(SonicExecutionModule, **execution_config)
    try:
        module.start()
        peer.reference_endpoint = module.get_status()["reference_endpoint"]
        peer.connected.set()
        wait_until(lambda: module.get_status()["ready"], timeout=3)
        yield module
    finally:
        module.stop()


def begin(module):
    status = module.start_task()
    wait_until(lambda: module.get_status()["task"]["phase"] == "waiting_for_chunk", timeout=3)
    return status["task_id"]


def submit(module, task_id, sequence=0, observed_at=None):
    observed_at = module.get_feedback().received_monotonic if observed_at is None else observed_at
    module.submit_chunk(task_id, sequence, observed_at, np.full((50, 78), 0.2))


def test_start_is_passive_and_reference_binding_is_exclusive(execution, tmp_path):
    module, peer = execution
    assert peer.snapshot() == []
    other = SonicExecutionModule(
        reference_bind=peer.reference_endpoint,
        feedback_endpoint=peer.feedback_endpoint,
        journal_dir=tmp_path / "other",
        profile=module.config.profile,
    )
    try:
        with pytest.raises(zmq.ZMQError, match="Address already in use"):
            other.start()
        assert peer.snapshot() == []
    finally:
        other.stop()


def test_finish_streams_actions_and_confirms_handback_without_task_success(execution):
    module, peer = execution
    task_id = begin(module)
    submit(module, task_id)
    wait_until(lambda: module.get_status()["task"]["last_frame_index"] > 8, timeout=2)
    module.finish_task(task_id)
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=2)
    status = module.get_status()
    assert status["task"]["outcome"] == "finished"
    assert status["task"]["handback_confirmed"] is True
    assert status["task_success"] is None
    packets = peer.snapshot()
    poses = [p for p in packets if p.startswith(b"pose")]
    indices = [int(np.frombuffer(p[1540:1548], dtype="<i8")[0]) for p in poses]
    assert indices == sorted(set(indices))
    np.testing.assert_allclose(np.frombuffer(poses[0][1548:], dtype="<f4")[:7], 0.1)
    assert all(p[-3:-1] == b"\0\0" for p in packets if p.startswith(b"command"))
    last_command = max(i for i, packet in enumerate(packets) if packet.startswith(b"command"))
    assert not any(packet.startswith(b"pose") for packet in packets[last_command + 1 :])
    events = [json.loads(line) for line in Path(status["journal_path"]).read_text().splitlines()]
    assert events[-1]["event"] == "terminal"


def test_overlap_old_task_ids_and_late_chunks_are_rejected(execution):
    module, _ = execution
    first = begin(module)
    with pytest.raises(RuntimeError, match="active"):
        module.start_task()
    module.cancel_task(first)
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=2)
    assert module.get_status()["task"]["outcome"] == "cancelled"
    second = begin(module)
    assert second != first
    with pytest.raises(ValueError, match="task"):
        submit(module, first)
    with pytest.raises(ValueError, match="task"):
        module.cancel_task(first)
    submit(module, second)
    module.finish_task(second)


def test_observation_time_is_preserved_and_stale_chunks_cannot_refresh_session(execution):
    module, _ = execution
    task_id = begin(module)
    submit(module, task_id)
    with pytest.raises(ValueError, match="observation"):
        submit(module, task_id, sequence=1, observed_at=time.monotonic() - 5)
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=3)
    status = module.get_status()["task"]
    assert status["outcome"] == "failed"
    assert status["reason"] == "observation stale"
    assert status["handback_confirmed"] is True


def test_missing_feedback_fails_without_claiming_handback(execution):
    module, peer = execution
    begin(module)
    peer.pause_feedback.set()
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=3)
    status = module.get_status()["task"]
    assert status["outcome"] == "failed"
    assert "feedback stale" in status["reason"]
    assert status["handback_confirmed"] is False
    assert module.get_feedback() is None


def test_action_source_failure_is_distinct_from_finish(execution):
    module, _ = execution
    task_id = begin(module)
    module.fail_task(task_id, "policy worker failed")
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=2)
    status = module.get_status()["task"]
    assert status["outcome"] == "failed"
    assert status["reason"] == "policy worker failed"
    assert status["handback_confirmed"] is True


def test_module_stop_waits_for_planner_and_keeps_terminal_status(execution):
    module, _ = execution
    begin(module)
    module.stop()
    status = module.get_status()
    assert status["state"] == "stopped"
    assert status["task"]["outcome"] == "cancelled"
    assert status["task"]["handback_confirmed"] is True
    assert module.get_feedback() is None


def test_subscription_loss_fails_and_stops_policy_output(execution):
    module, peer = execution
    task_id = begin(module)
    submit(module, task_id)
    peer.drop_pose_subscription.set()
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=2)
    status = module.get_status()
    assert status["state"] == "fault"
    assert status["task"]["outcome"] == "failed"
    assert "subscriptions lost" in status["task"]["reason"]
    assert status["task"]["handback_confirmed"] is True
    with pytest.raises(RuntimeError, match="subscriptions lost"):
        module.start_task()


def test_concurrent_stops_do_not_close_loop_before_bounded_handback(execution, mocker):
    module, peer = execution
    begin(module)
    peer.pause_feedback.set()
    # Intercept final loop closure so the regression reports its premature call
    # instead of stranding the first stop on an event loop closed by the second.
    close_module = module._close_module
    phases_at_close = []
    mocker.patch.object(
        module,
        "_close_module",
        side_effect=lambda: phases_at_close.append(module.get_status()["task"]["phase"]),
    )
    started = time.monotonic()
    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            first = pool.submit(module.stop)
            wait_until(lambda: module.get_status()["task"]["phase"] == "handback", timeout=1)
            second = pool.submit(module.stop)
            first.result(timeout=2)
            second.result(timeout=2)
        assert phases_at_close == ["done", "done"]
        assert time.monotonic() - started < 2
        status = module.get_status()["task"]
        assert status["outcome"] == "failed"
        assert status["handback_confirmed"] is False
    finally:
        close_module()


def test_worker_rpc_preserves_chunks_and_terminal_results(worker_execution):
    module = worker_execution
    task_id = begin(module)
    submit(module, task_id)
    assert module.get_status()["task"]["last_chunk_sequence"] == 0
    module.finish_task(task_id)
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=2)
    status = module.get_status()["task"]
    assert status["outcome"] == "finished"
    assert status["handback_confirmed"] is True


@pytest.mark.parametrize("socket_name", ["_publisher", "_subscriber"])
def test_arrival_after_empty_poll_is_not_reported_as_receive_overload(
    execution_config, mocker, socket_name
):
    module = SonicExecutionModule(**execution_config)
    try:
        publisher = mocker.patch.object(module, "_publisher")
        subscriber = mocker.patch.object(module, "_subscriber")
        publisher.poll.return_value = subscriber.poll.return_value = 0
        target = getattr(module, socket_name)
        # Queue is empty at the receive boundary; a new packet arrives immediately
        # afterward. A second poll must not mistake it for an exhausted budget.
        target.poll.side_effect = [0, zmq.POLLIN]
        module._receive()
        assert target.recv.call_count == target.recv_multipart.call_count == 0
    finally:
        module.stop()


@pytest.mark.parametrize("socket_name", ["_publisher", "_subscriber"])
@pytest.mark.parametrize("extra_packet", [False, True])
def test_receive_budget_boundary(execution_config, mocker, socket_name, extra_packet):
    module = SonicExecutionModule(**execution_config)
    try:
        publisher = mocker.patch.object(module, "_publisher")
        subscriber = mocker.patch.object(module, "_subscriber")
        publisher.poll.return_value = subscriber.poll.return_value = 0
        target = getattr(module, socket_name)
        target.poll.side_effect = [zmq.POLLIN] * 64 + [zmq.POLLIN if extra_packet else 0]
        target.recv.return_value = b"\x01pose"
        target.recv_multipart.return_value = [b"g1_debuginvalid"]
        if extra_packet:
            with pytest.raises(RuntimeError, match="backlog exceeded receive budget"):
                module._receive()
        else:
            module._receive()
        assert target.recv.call_count + target.recv_multipart.call_count == 64
    finally:
        module.stop()
