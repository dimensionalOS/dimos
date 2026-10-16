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
from dataclasses import replace
import json
from pathlib import Path
import threading
import time
from uuid import uuid4

import msgpack
import numpy as np
import pytest
import zmq

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.rpc_client import RPCClient
from dimos.core.transport_factory import make_transport
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_observation import SonicDeliveryStamp, SonicObservation
from dimos.robot.unitree.g1.sonic_policy_client import (
    SonicPolicyChunk,
    SonicPolicyClient,
    SonicPolicyInfo,
)
from dimos.robot.unitree.g1.sonic_service_module import SonicServiceModule
from dimos.robot.unitree.g1.test_sonic_execution_module import (
    execution as execution,
    execution_config as execution_config,
    peer as peer,
)
from dimos.utils.testing.waiting import wait_until

KEY = "observation.images.egocentric"


@pytest.fixture
def policy(mocker):
    info = mocker.patch.object(
        SonicPolicyClient, "get_info", return_value=SonicPolicyInfo(1, {KEY: (3, 480, 640)})
    )

    def predict(**kwargs):
        return SonicPolicyChunk(
            kwargs["task_id"],
            kwargs["sequence"],
            kwargs["observed_at"],
            np.full((50, 78), 0.2, np.float32),
            0.01,
            0.01,
        )

    infer = mocker.patch.object(SonicPolicyClient, "infer", side_effect=predict)
    return info, infer, predict


@pytest.fixture
def service(execution, tmp_path, policy, mocker, request):
    module = SonicServiceModule(
        policies={"pi05": "ipc:///unused-policy"},
        execution_profile_name="scripted-peer",
        journal_dir=tmp_path / "service",
        replan_interval_seconds=0.04,
        policy_timeout_seconds=0.2,
        execution_rpc_timeout_seconds=0.2,
        recovery_timeout_seconds=0.8,
        **getattr(request, "param", {}),
    )
    mocker.patch.object(module, "_execution", execution[0], create=True)
    transport = make_transport(f"/service-test/{uuid4().hex}", SonicObservation)
    module.set_transport("observation", transport)
    try:
        transport.start()
        module.start()
        yield module
    finally:
        module.stop()


@pytest.fixture
def camera(service):
    stopped = threading.Event()
    errors = []

    def feed():
        sequence = 0
        try:
            while not stopped.is_set():
                received_at = time.monotonic()
                stamps = (
                    tuple(
                        SonicDeliveryStamp(group, sequence, received_at)
                        for group in ("camera", "body", "left_hand", "right_hand")
                    )
                    if service.config.observation_time_basis == "delivery"
                    else None
                )
                service.handle_observation(
                    SonicObservation(
                        sequence,
                        received_at,
                        np.full(28, sequence / 1000, np.float32),
                        np.full((480, 640, 3), sequence % 255, np.uint8),
                        time_basis=service.config.observation_time_basis,
                        delivery_stamps=stamps,
                    )
                )
                sequence += 1
                stopped.wait(0.02)
        except Exception as exc:
            errors.append(exc)

    thread = threading.Thread(target=feed)
    thread.start()
    try:
        wait_until(lambda: service.get_service_status()["observation_age_s"] is not None, timeout=2)
        yield stopped
    finally:
        stopped.set()
        thread.join(timeout=2)
        assert not thread.is_alive()
        assert errors == []


def request(service, duration=0.12):
    return service.request_service("pi05", "pick drink bottle and handover", duration)


def terminal(service):
    wait_until(lambda: service.get_service_status()["service"]["phase"] == "done", timeout=4)
    return service.get_service_status()["service"]


def test_finish_preserves_observation_time_and_requires_outcome_evidence(
    service, camera, execution
):
    accepted = request(service)
    done = terminal(service)
    assert done["service_id"] == accepted["service_id"]
    assert done["outcome"] == "finished" and done["handback_confirmed"]
    assert done["accepted_predictions"] >= 2 and done["task_success"] is None
    assert done["progress"] == 1.0
    events = [json.loads(row) for row in Path(done["journal_path"]).read_text().splitlines()]
    native = [
        json.loads(row)
        for row in Path(execution[0].get_status()["journal_path"]).read_text().splitlines()
    ]
    expected_times = [e["observed_at"] for e in events if e["event"] == "prediction_accepted"]
    assert expected_times == [e["observed_at"] for e in native if e["event"] == "chunk_accepted"]
    assert all(
        e["monotonic"] > e["observed_at"] for e in events if e["event"] == "prediction_accepted"
    )
    verified = service.report_outcome(done["service_id"], True, "Operator observed object in tray")
    assert verified["task_success"] is True
    with pytest.raises(ValueError, match="already"):
        service.report_outcome(done["service_id"], False, "conflicting report")


def test_passive_start_and_missing_observation_do_not_start_controller(service, execution, policy):
    with pytest.raises(ValueError, match="observation"):
        request(service)
    assert execution[1].snapshot() == []
    policy[0].assert_not_called()


@pytest.mark.parametrize(
    "policy_name,instruction,duration",
    [
        ("unknown", "task", 1),
        ("pi05", " ", 1),
        ("pi05", "task", 0),
        ("pi05", "task", float("nan")),
        ("pi05", "task", True),
    ],
)
def test_invalid_request_is_rejected_before_admission(
    service, camera, execution, policy_name, instruction, duration
):
    with pytest.raises(ValueError):
        service.request_service(policy_name, instruction, duration)
    assert execution[0].get_status()["task"] is None


def test_cancel_does_not_wait_for_inference_and_discards_late_result(
    service, camera, execution, policy
):
    entered, release = threading.Event(), threading.Event()

    def blocked(**kwargs):
        entered.set()
        assert release.wait(3)
        return policy[2](**kwargs)

    policy[1].side_effect = blocked
    try:
        accepted = request(service, 1)
        assert entered.wait(2)
        with pytest.raises(RuntimeError, match="active"):
            request(service)
        with pytest.raises(ValueError, match="another"):
            service.cancel_service("wrong-id")
        service.cancel_service(accepted["service_id"])
        wait_until(lambda: execution[0].get_status()["task"]["phase"] == "done", timeout=1)
        assert execution[0].get_status()["task"]["handback_confirmed"]
        assert not release.is_set()
    finally:
        release.set()
    done = terminal(service)
    assert done["outcome"] == "cancelled" and done["accepted_predictions"] == 0
    assert done["discarded_predictions"] == 1
    with pytest.raises(ValueError, match="finished"):
        service.report_outcome(done["service_id"], True, "unjustified success")


def test_frozen_camera_expires_without_refreshing_timestamp(service, camera):
    request(service, 2)
    wait_until(
        lambda: service.get_service_status()["service"]["accepted_predictions"] >= 2, timeout=2
    )
    camera.set()
    done = terminal(service)
    assert done["outcome"] == "failed" and done["handback_confirmed"]
    assert done["reason"] == "observation stale"
    assert done["task_success"] is None


def test_incompatible_policy_fails_before_controller_admission(service, camera, execution, policy):
    policy[0].return_value = SonicPolicyInfo(2, {KEY: (3, 480, 640)})
    request(service)
    done = terminal(service)
    assert done["outcome"] == "failed" and "one-frame" in done["reason"]
    assert execution[0].get_status()["task"] is None


def test_inference_error_requests_recovery(service, camera, policy):
    policy[1].side_effect = TimeoutError("policy timeout")
    request(service)
    done = terminal(service)
    assert done["outcome"] == "failed" and done["handback_confirmed"]
    assert "policy timeout" in done["reason"]
    assert service.get_service_status()["error"] is None
    reported = service.report_outcome(done["service_id"], False, "Policy failed before grasp")
    assert reported["task_success"] is False
    policy[1].side_effect = policy[2]
    following = request(service)
    assert following["service_id"] != done["service_id"]
    with pytest.raises(ValueError, match="another"):
        service.cancel_service(done["service_id"])
    with pytest.raises(ValueError, match="terminal"):
        service.report_outcome(following["service_id"], True, "premature")
    assert terminal(service)["outcome"] == "finished"


@pytest.mark.parametrize("bad_sample", ["duplicate", "old", "future", "reused_timestamp"])
def test_rejected_observation_keeps_previous_source_snapshot(service, policy, bad_sample):
    observed_at = time.monotonic()
    state, rgb = np.zeros(28), np.zeros((480, 640, 3), np.uint8)
    service.handle_observation(SonicObservation(3, observed_at, state, rgb))
    sequence, timestamp = {
        "duplicate": (3, time.monotonic()),
        "old": (4, observed_at - 5),
        "future": (4, observed_at + 5),
        "reused_timestamp": (4, observed_at),
    }[bad_sample]
    with pytest.raises(ValueError, match="observation"):
        service.handle_observation(SonicObservation(sequence, timestamp, state, rgb))
    request(service, 0.03)
    assert terminal(service)["outcome"] == "finished"
    assert policy[1].call_args.kwargs["observed_at"] == observed_at


def test_cancel_during_start_binds_returned_task_then_hands_back(
    service, camera, execution, mocker
):
    entered, release = threading.Event(), threading.Event()
    original = execution[0].start_task

    def delayed():
        entered.set()
        assert release.wait(2)
        return original()

    mocker.patch.object(execution[0], "start_task", side_effect=delayed)
    try:
        accepted = request(service)
        assert entered.wait(2)
        service.cancel_service(accepted["service_id"])
    finally:
        release.set()
    done = terminal(service)
    assert done["outcome"] == "cancelled" and done["handback_confirmed"]
    assert done["accepted_predictions"] == 0


def test_lost_start_reply_latches_uncertainty_and_never_retries(service, camera, execution, mocker):
    original = execution[0].start_task

    def lost_reply():
        original()
        raise TimeoutError("lost start reply")

    call = mocker.patch.object(execution[0], "start_task", side_effect=lost_reply)
    request(service)
    done = terminal(service)
    assert done["outcome"] == "failed" and not done["handback_confirmed"]
    assert service.get_service_status()["error"] is not None
    with pytest.raises(RuntimeError, match="fault"):
        request(service)
    call.assert_called_once_with()


def test_shutdown_requests_handback_before_draining_inference(service, camera, execution, policy):
    entered, release = threading.Event(), threading.Event()

    def blocked(**kwargs):
        entered.set()
        assert release.wait(3)
        return policy[2](**kwargs)

    policy[1].side_effect = blocked
    request(service, 1)
    assert entered.wait(2)
    camera.set()
    with ThreadPoolExecutor(max_workers=1) as pool:
        stopped = pool.submit(service.stop)
        try:
            wait_until(lambda: execution[0].get_status()["task"]["phase"] == "done", timeout=1)
            assert execution[0].get_status()["task"]["handback_confirmed"]
        finally:
            release.set()
        stopped.result(timeout=2)
    assert service.get_service_status()["service"]["outcome"] == "cancelled"


def test_unconfirmed_handback_latches_fault_and_freezes_progress(service, camera, execution):
    accepted = request(service, 2)
    wait_until(
        lambda: service.get_service_status()["service"]["accepted_predictions"] >= 2, timeout=2
    )
    execution[1].pause_feedback.set()
    service.cancel_service(accepted["service_id"])
    wait_until(lambda: service.get_service_status()["service"]["phase"] == "recovering", timeout=1)
    stopped_progress = service.get_service_status()["service"]["progress"]
    done = terminal(service)
    assert done["progress"] == stopped_progress
    assert done["outcome"] == "failed" and not done["handback_confirmed"]
    assert service.get_service_status()["state"] == "fault"
    assert not service.get_service_status()["accepting_requests"]
    with pytest.raises(RuntimeError, match="fault"):
        request(service)


def test_preflight_cancel_takes_priority_over_expired_camera(service, camera, execution, policy):
    entered, release = threading.Event(), threading.Event()
    info = policy[0].return_value

    def blocked():
        entered.set()
        assert release.wait(2)
        return info

    policy[0].side_effect = blocked
    try:
        accepted = request(service)
        assert entered.wait(2)
        camera.set()
        service.cancel_service(accepted["service_id"])
        wait_until(lambda: service.get_service_status()["observation_age_s"] > 0.2, timeout=1)
    finally:
        release.set()
    done = terminal(service)
    assert done["outcome"] == "cancelled"
    assert execution[0].get_status()["task"] is None


@pytest.mark.parametrize(
    "override,reason",
    [({"name": "another-checkpoint"}, "configured profile"), ({"max_duration": 1}, "budget")],
)
def test_execution_profile_preflight_rejects_without_start(
    service, camera, execution, mocker, override, reason
):
    status = execution[0].get_status()
    status["profile"].update(override)
    mocker.patch.object(execution[0], "get_status", return_value=status)
    request(service)
    done = terminal(service)
    assert done["outcome"] == "failed" and reason in done["reason"]
    assert execution[1].snapshot() == []


@pytest.fixture
def policy_wire(tmp_path):
    endpoint = f"ipc://{tmp_path}/policy.sock"
    ready, stopped = threading.Event(), threading.Event()
    requests = []

    def serve():
        with zmq.Context() as ctx, ctx.socket(zmq.REP) as sock:
            sock.setsockopt(zmq.LINGER, 0)
            sock.bind(endpoint)
            ready.set()
            while not stopped.is_set():
                if not sock.poll(20):
                    continue
                request = msgpack.unpackb(sock.recv(), raw=False)
                requests.append(request)
                if request["op"] == "info":
                    reply = dict(image_keys=[KEY], n_obs=1, shapes={KEY: [3, 480, 640]})
                else:
                    reply = dict(
                        chunk=np.full((50, 78), 0.2, dtype="<f4").tobytes(),
                        shape=[50, 78],
                        latency_s=0.01,
                    )
                sock.send(msgpack.packb(reply, use_bin_type=True))

    with ThreadPoolExecutor(max_workers=1) as pool:
        future = pool.submit(serve)
        try:
            assert ready.wait(2)
            yield endpoint, requests
        finally:
            stopped.set()
            future.result(timeout=2)


@pytest.fixture
def worker_service(peer, execution_config, policy_wire, tmp_path):
    original_config = global_config.model_dump()
    publisher = make_transport(f"/service-worker-test/{uuid4().hex}", SonicObservation)
    blueprint = (
        autoconnect(
            SonicExecutionModule.blueprint(**execution_config),
            SonicServiceModule.blueprint(
                policies={"pi05": policy_wire[0]},
                execution_profile_name="scripted-peer",
                journal_dir=tmp_path / "service",
                replan_interval_seconds=0.04,
                execution_rpc_timeout_seconds=0.3,
                policy_timeout_seconds=0.5,
                recovery_timeout_seconds=0.8,
            ),
        )
        .transports({("observation", SonicObservation): publisher})
        .global_config(n_workers=2, viewer="none")
    )
    coordinator = None
    stopped = threading.Event()

    def feed():
        sequence = 0
        while not stopped.is_set():
            publisher.publish(
                SonicObservation(
                    sequence, time.monotonic(), np.zeros(28), np.zeros((480, 640, 3), np.uint8)
                )
            )
            sequence += 1
            stopped.wait(0.02)

    try:
        coordinator = ModuleCoordinator.build(blueprint)
        module = coordinator.get_instance(SonicServiceModule)
        native = coordinator.get_instance(SonicExecutionModule)
        peer.reference_endpoint = native.get_status()["reference_endpoint"]
        peer.connected.set()
        wait_until(lambda: native.get_status()["ready"], timeout=3)
        publisher.start()
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(feed)
            try:
                wait_until(
                    lambda: module.get_service_status()["observation_age_s"] is not None,
                    timeout=3,
                )
                yield module, native
            finally:
                stopped.set()
                future.result(timeout=2)
    finally:
        publisher.stop()
        if coordinator is not None:
            coordinator.stop()
        global_config.update(**original_config)


def test_blueprint_worker_stream_rpc_and_real_policy_wire(worker_service, policy_wire, peer):
    module, native = worker_service
    accepted = request(module)
    done = terminal(module)
    assert done["service_id"] == accepted["service_id"]
    assert done["execution_task_id"] == native.get_status()["task"]["task_id"]
    assert done["outcome"] == "finished" and done["handback_confirmed"]
    assert done["task_success"] is None
    chunks = [req for req in policy_wire[1] if req["op"] == "chunk"]
    assert len(chunks) >= done["accepted_predictions"] >= 2
    assert all(req["task"] == accepted["instruction"] for req in chunks)
    assert all(req["states"] == [[0.0] * 28] for req in chunks)
    packets = peer.snapshot()
    last_command = max(i for i, packet in enumerate(packets) if packet.startswith(b"command"))
    assert not any(packet.startswith(b"pose") for packet in packets[last_command + 1 :])


def test_worker_injected_rpc_uses_bounded_timeout(dimos_cluster, tmp_path):
    module = dimos_cluster.deploy(
        SonicServiceModule,
        policies={"pi05": "ipc:///unused-policy"},
        execution_profile_name="scripted-peer",
        journal_dir=tmp_path,
        execution_rpc_timeout_seconds=0.1,
    )
    missing = RPCClient.remote(SonicExecutionModule, remote_name=f"missing-{uuid4().hex}")
    transport = make_transport(f"/service-missing-test/{uuid4().hex}", SonicObservation)
    try:
        module.set_module_ref("_execution", missing)
        module.set_transport("observation", transport)
        module.start()
        module.handle_observation(
            SonicObservation(0, time.monotonic(), np.zeros(28), np.zeros((480, 640, 3), np.uint8))
        )
        started = time.monotonic()
        request(module)
        done = terminal(module)
        assert time.monotonic() - started < 2
        assert done["outcome"] == "failed" and done["execution_task_id"] is None
        assert "RPC uncertain" in module.get_service_status()["error"]
        assert not module.get_service_status()["accepting_requests"]
    finally:
        module.stop()
        missing.stop_rpc_client()
        transport.stop()


@pytest.fixture
def delivery_profile(execution_config):
    execution_config["profile"] = replace(
        execution_config["profile"], observation_time_basis="delivery", seed_seconds=0.3
    )


def test_acquisition_service_rejects_delivery_snapshot_before_inference(service, policy):
    now = time.monotonic()
    stamps = tuple(SonicDeliveryStamp(str(i), 1, now) for i in range(4))
    with pytest.raises(ValueError, match="basis"):
        service.handle_observation(
            SonicObservation(
                0,
                now,
                np.zeros(28),
                np.zeros((480, 640, 3), np.uint8),
                time_basis="delivery",
                delivery_stamps=stamps,
            )
        )
    assert service.get_service_status()["observation_age_s"] is None
    policy[1].assert_not_called()


@pytest.mark.parametrize("service", [{"observation_time_basis": "delivery"}], indirect=True)
def test_service_rejects_execution_timing_profile_mismatch(service, camera, execution, policy):
    request(service)
    done = terminal(service)
    assert done["outcome"] == "failed" and "basis" in done["reason"]
    assert execution[0].get_status()["task"] is None
    policy[0].assert_not_called()


@pytest.mark.usefixtures("delivery_profile")
@pytest.mark.parametrize("service", [{"observation_time_basis": "delivery"}], indirect=True)
def test_delivery_basis_and_receipts_survive_service_copy_and_journal(service, camera):
    request(service)
    done = terminal(service)
    status = service.get_service_status()
    assert done["outcome"] == "finished" and done["handback_confirmed"]
    assert status["observation_time_basis"] == "delivery"
    assert status["acquisition_age_s"] is None and status["acquisition_skew_s"] is None
    assert status["delivery_age_s"] >= 0
    assert status["input_liveness_max_age_s"] == 0.2
    assert status["prediction_max_age_s"] == 0.5
    events = [json.loads(row) for row in Path(done["journal_path"]).read_text().splitlines()]
    predictions = [e for e in events if e["event"] == "inference_requested"]
    assert predictions
    assert all(e["observation_time_basis"] == "delivery" for e in predictions)
    assert all(len(e["delivery_stamps"]) == 4 for e in predictions)
    assert all(
        e["observed_at"] == min(s["received_at"] for s in e["delivery_stamps"]) for e in predictions
    )


@pytest.mark.usefixtures("delivery_profile")
@pytest.mark.parametrize("service", [{"observation_time_basis": "delivery"}], indirect=True)
def test_delivery_expiry_recovers_while_first_inference_is_still_blocked(
    service, camera, execution, policy
):
    entered, release = threading.Event(), threading.Event()

    def blocked(**kwargs):
        entered.set()
        assert release.wait(3)
        return policy[2](**kwargs)

    policy[1].side_effect = blocked
    try:
        request(service, 2)
        assert entered.wait(2)
        camera.set()
        wait_until(lambda: execution[0].get_status()["task"]["phase"] == "done", timeout=1)
        task = execution[0].get_status()["task"]
        assert task["handback_confirmed"] and "delivery" in task["reason"]
        assert not release.is_set()
    finally:
        release.set()
    done = terminal(service)
    assert done["outcome"] == "failed" and done["accepted_predictions"] == 0


@pytest.mark.usefixtures("delivery_profile")
@pytest.mark.parametrize("service", [{"observation_time_basis": "delivery"}], indirect=True)
def test_delivery_expiry_during_seeding_requests_recovery(
    service, camera, execution, policy, mocker
):
    original = execution[0].start_task

    def freeze_after_start():
        result = original()
        camera.set()
        return result

    mocker.patch.object(execution[0], "start_task", side_effect=freeze_after_start)
    request(service, 2)
    done = terminal(service)
    assert done["outcome"] == "failed" and done["handback_confirmed"]
    assert "delivery" in done["reason"]
    policy[1].assert_not_called()


def delivery_observation(sequence, source="receiver"):
    now = time.monotonic()
    return SonicObservation(
        sequence,
        now,
        np.zeros(28),
        np.zeros((480, 640, 3), np.uint8),
        time_basis="delivery",
        delivery_stamps=tuple(SonicDeliveryStamp(f"{source}/{i}", sequence, now) for i in range(4)),
    )


@pytest.mark.parametrize("service", [{"observation_time_basis": "delivery"}], indirect=True)
def test_delivery_receiver_restart_latches_fault_even_when_sequence_restarts(service):
    service.handle_observation(delivery_observation(5))
    with pytest.raises(ValueError, match="source changed"):
        service.handle_observation(delivery_observation(0, "replacement"))
    assert service.get_service_status()["state"] == "fault"
    with pytest.raises(RuntimeError, match="fault"):
        request(service)


@pytest.mark.usefixtures("delivery_profile")
@pytest.mark.parametrize("service", [{"observation_time_basis": "delivery"}], indirect=True)
def test_delivery_source_fault_during_policy_preflight_never_starts_execution(
    service, execution, policy
):
    entered, release = threading.Event(), threading.Event()
    info = policy[0].return_value

    def blocked_info():
        entered.set()
        assert release.wait(3)
        return info

    policy[0].side_effect = blocked_info
    service.handle_observation(delivery_observation(0))
    try:
        request(service)
        assert entered.wait(2)
        with pytest.raises(ValueError, match="source changed"):
            service.handle_observation(delivery_observation(1, "replacement"))
    finally:
        release.set()
    done = terminal(service)
    assert "source changed" in done["reason"]
    assert execution[0].get_status()["task"] is None
