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

"""Supervise one LeRobot action source through the guarded SONIC execution module.

Configured policies must share the declared execution profile. Server metadata
does not establish checkpoint/control compatibility. This module neither starts
native control nor determines object success from successful streaming.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Callable
from concurrent.futures import ThreadPoolExecutor
import copy
from dataclasses import asdict
from functools import partial
import math
from pathlib import Path
import threading
import time
from typing import Any, Protocol, TypeVar
from uuid import uuid4

from numpy.typing import NDArray
from pydantic import Field, field_validator

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In, Out
from dimos.robot.unitree.g1.sonic_observation import ObservationTimeBasis, SonicObservation
from dimos.robot.unitree.g1.sonic_policy_client import SonicPolicyChunk, SonicPolicyClient
from dimos.robot.unitree.g1.sonic_session import JsonlSessionJournal
from dimos.spec.utils import Spec

T = TypeVar("T")
_IMAGE_KEY = "observation.images.egocentric"
_EXECUTION_METHODS = (
    "get_status",
    "start_task",
    "submit_chunk",
    "finish_task",
    "cancel_task",
    "fail_task",
)


class SonicExecutionSpec(Spec, Protocol):
    def get_status(self) -> dict[str, Any]: ...
    def start_task(self) -> dict[str, Any]: ...
    def submit_chunk(
        self, task_id: str, sequence: int, observed_at: float, actions: NDArray[Any]
    ) -> dict[str, Any]: ...
    def finish_task(self, task_id: str) -> dict[str, Any]: ...
    def cancel_task(self, task_id: str) -> dict[str, Any]: ...
    def fail_task(self, task_id: str, reason: str) -> dict[str, Any]: ...


class SonicServiceConfig(ModuleConfig):
    policies: dict[str, str]
    execution_profile_name: str = Field(min_length=1)
    journal_dir: Path
    observation_time_basis: ObservationTimeBasis = "acquisition"
    replan_interval_seconds: float = Field(default=0.35, gt=0, le=1, allow_inf_nan=False)
    observation_max_age_seconds: float = Field(default=0.2, gt=0, le=0.2, allow_inf_nan=False)
    policy_timeout_seconds: float = Field(default=2.5, gt=0, le=5, allow_inf_nan=False)
    execution_rpc_timeout_seconds: float = Field(default=0.5, gt=0, le=1, allow_inf_nan=False)
    recovery_timeout_seconds: float = Field(default=6, gt=0, le=10, allow_inf_nan=False)
    max_duration_seconds: float = Field(default=20, gt=0, le=60, allow_inf_nan=False)

    @field_validator("policies")
    @classmethod
    def explicit_endpoints(cls, value: dict[str, str]) -> dict[str, str]:
        if not value or any(not key.strip() for key in value):
            raise ValueError("at least one named policy is required")
        for endpoint in value.values():
            SonicPolicyClient(endpoint)
        return value


class SonicServiceModule(Module):
    """Request, supervise and cancel policy execution; report outcomes separately."""

    config: SonicServiceConfig
    observation: In[SonicObservation]
    service_status: Out[dict[str, Any]]
    _execution: SonicExecutionSpec

    def __init__(self, **kwargs: Any) -> None:
        self._lifecycle_lock = threading.Lock()
        self._snapshot_lock = threading.Lock()
        self._snapshot: dict[str, Any] = {}
        self._observation: SonicObservation | None = None
        self._service: dict[str, Any] | None = None
        self._journal: JsonlSessionJournal | None = None
        self._runner: asyncio.Task[None] | None = None
        self._inference: asyncio.Task[SonicPolicyChunk] | None = None
        self._workers: ThreadPoolExecutor | None = None
        self._running = self._closing = self._busy = self._cancel_requested = False
        self._error: str | None = None
        super().__init__(**kwargs)
        self._publish()

    @rpc
    def set_module_ref(self, name: str, module_ref: RPCClient) -> None:
        if name == "_execution":
            # A worker receives its own unpickled proxy. Bound its transport's
            # actual waits; cancelling to_thread alone does not stop a call.
            for method in _EXECUTION_METHODS:
                module_ref.rpc.rpc_timeouts[f"{module_ref.remote_name}/{method}"] = (
                    self.config.execution_rpc_timeout_seconds
                )
        super().set_module_ref(name, module_ref)

    @rpc
    def start(self) -> None:
        with self._lifecycle_lock:
            if not self._running:
                super().start()

    @rpc
    def stop(self) -> None:
        with self._lifecycle_lock:
            super().stop()

    async def main(self) -> AsyncIterator[None]:
        self.config.journal_dir.mkdir(parents=True, exist_ok=True)
        self._workers = ThreadPoolExecutor(max_workers=2, thread_name_prefix="sonic-service")
        self._running, self._closing = True, False
        self._publish()
        try:
            yield
        finally:
            self._closing = self._cancel_requested = True
            if self._runner is not None and not self._runner.done():
                budget = (
                    4 * self.config.execution_rpc_timeout_seconds
                    + self.config.recovery_timeout_seconds
                    + 2 * self.config.policy_timeout_seconds
                    + 1
                )
                done, _ = await asyncio.wait({self._runner}, timeout=budget)
                if not done:
                    self._error = "service shutdown deadline exceeded; controller state unresolved"
                    self._runner.cancel()
                    await asyncio.gather(self._runner, return_exceptions=True)
            self._running = False
            self._publish()
            self._workers.shutdown(wait=False, cancel_futures=True)
            if isinstance(self._execution, RPCClient):
                self._execution.stop_rpc_client()

    def _publish(self) -> None:
        snapshot = {
            "state": "fault" if self._error else "running" if self._running else "stopped",
            "accepting_requests": self._running
            and not self._closing
            and not self._busy
            and not self._error,
            "policies": sorted(self.config.policies),
            "observation_time_basis": self.config.observation_time_basis,
            "input_liveness_max_age_s": self.config.observation_max_age_seconds,
            "prediction_max_age_s": None
            if self._service is None
            else self._service.get("prediction_max_age_seconds"),
            "acquisition_skew_s": None,
            "observation_time": None
            if self._observation is None
            else self._observation.observed_at,
            "service": copy.deepcopy(self._service),
            "error": self._error,
        }
        with self._snapshot_lock:
            self._snapshot = snapshot
        if self._running:
            self.service_status.publish(copy.deepcopy(snapshot))

    @rpc
    def get_service_status(self) -> dict[str, Any]:
        """Return lifecycle and streaming progress; object outcome stays unverified."""
        with self._snapshot_lock:
            snapshot = copy.deepcopy(self._snapshot)
        observed_at = snapshot.pop("observation_time")
        snapshot["observation_age_s"] = (
            None if observed_at is None else time.monotonic() - observed_at
        )
        delivery = snapshot["observation_time_basis"] == "delivery"
        snapshot["delivery_age_s"] = snapshot["observation_age_s"] if delivery else None
        snapshot["acquisition_age_s"] = None if delivery else snapshot["observation_age_s"]
        return snapshot

    @rpc
    async def handle_observation(self, observation: SonicObservation) -> None:
        """Accept the configured timing basis, preserving its original provenance."""
        if observation.time_basis != self.config.observation_time_basis:
            raise ValueError("observation time basis differs from service profile")
        # Own and revalidate metadata/arrays after transport deserialization.
        observation = SonicObservation(
            observation.sequence,
            observation.observed_at,
            observation.state,
            observation.rgb,
            time_basis=observation.time_basis,
            delivery_stamps=observation.delivery_stamps,
        )
        previous = self._observation
        now = time.monotonic()
        observation.require_fresh(
            now=now,
            max_age=self.config.observation_max_age_seconds,
        )
        if previous is not None and previous.delivery_stamps is not None:
            assert observation.delivery_stamps is not None
            for old, new in zip(previous.delivery_stamps, observation.delivery_stamps, strict=True):
                if old.source_id != new.source_id or new.source_marker < old.source_marker:
                    self._error = (
                        "delivery source changed; replace receiver/adapter/service session"
                    )
                    self._publish()
                    raise ValueError(self._error)
                if new.source_marker == old.source_marker or new.received_at <= old.received_at:
                    raise ValueError("all component deliveries must advance")
        observation.require_fresh(
            now=now,
            after_sequence=-1 if previous is None else previous.sequence,
            after_observed_at=-math.inf if previous is None else previous.observed_at,
            max_age=self.config.observation_max_age_seconds,
        )
        self._observation = observation
        self._publish()

    def _fresh_observation(self, after_sequence: int = -1) -> SonicObservation:
        if self._error:
            raise ValueError(self._error)
        if self._observation is None:
            raise ValueError("a fresh paired observation is required")
        self._observation.require_fresh(
            now=time.monotonic(),
            after_sequence=after_sequence,
            max_age=self.config.observation_max_age_seconds,
        )
        return self._observation

    def _event(self, event: str, **fields: Any) -> None:
        assert self._journal is not None
        self._journal({"event": event, "monotonic": time.monotonic(), **fields})

    @rpc
    async def request_service(
        self, policy_name: str, instruction: str, duration_seconds: float
    ) -> dict[str, Any]:
        """Reserve one request; preflight and execution continue in the background."""
        if self._error:
            raise RuntimeError(f"service fault: {self._error}")
        if not self._running or self._closing:
            raise RuntimeError("service module is stopped")
        if self._busy:
            raise RuntimeError("a service is already active")
        if (
            policy_name not in self.config.policies
            or not instruction.strip()
            or len(instruction) > 4096
        ):
            raise ValueError("configured policy and nonempty instruction required")
        if (
            type(duration_seconds) not in (float, int)
            or not math.isfinite(duration_seconds)
            or not 0 < duration_seconds <= self.config.max_duration_seconds
        ):
            raise ValueError("duration exceeds the configured streaming budget")
        self._fresh_observation()
        service_id = uuid4().hex
        path = self.config.journal_dir / f"{service_id}.jsonl"
        journal = JsonlSessionJournal(path)
        service = {
            "service_id": service_id,
            "policy_name": policy_name,
            "observation_time_basis": self.config.observation_time_basis,
            "instruction": instruction,
            "duration_seconds": duration_seconds,
            "phase": "preparing",
            "outcome": None,
            "reason": "",
            "execution_task_id": None,
            "execution": None,
            "handback_confirmed": False,
            "task_success": None,
            "outcome_evidence": None,
            "accepted_predictions": 0,
            "discarded_predictions": 0,
            "progress": 0.0,
            "running_started": None,
            "journal_path": str(path),
        }
        journal({"event": "request", "monotonic": time.monotonic(), "service": service})
        self._journal, self._service = journal, service
        self._cancel_requested, self._busy = False, True
        self._runner = asyncio.create_task(self._run())
        self._publish()
        return copy.deepcopy(service)

    def _require_service(self, service_id: str) -> dict[str, Any]:
        if self._service is None or self._service["service_id"] != service_id:
            raise ValueError("request belongs to another service")
        return self._service

    @rpc
    async def cancel_service(self, service_id: str) -> dict[str, Any]:
        """Request cancellation promptly; poll until done for handback evidence."""
        service = self._require_service(service_id)
        if service["phase"] != "done":
            self._cancel_requested = True
            self._stop_progress()
            self._event("cancel_requested")
            self._publish()
        return copy.deepcopy(service)

    @rpc
    async def report_outcome(self, service_id: str, success: bool, evidence: str) -> dict[str, Any]:
        """Record an external object-outcome report, not an automatic verification."""
        service = self._require_service(service_id)
        if (
            service["phase"] != "done"
            or type(success) is not bool
            or not evidence.strip()
            or len(evidence) > 4096
        ):
            raise ValueError("terminal service and nonempty outcome evidence required")
        if service["task_success"] is not None:
            raise ValueError("outcome already reported")
        if success and (service["outcome"] != "finished" or not service["handback_confirmed"]):
            raise ValueError("success requires finished execution with confirmed handback")
        self._event("outcome_reported", success=success, evidence=evidence)
        service.update(task_success=success, outcome_evidence=evidence)
        self._publish()
        return copy.deepcopy(service)

    async def _call(self, method: Callable[..., T], *args: Any) -> T:
        try:
            return await self._offload(method, *args)
        except (TimeoutError, ConnectionError) as exc:
            self._error = f"execution RPC uncertain: {type(exc).__name__}: {exc}"
            raise

    async def _offload(self, method: Callable[..., T], *args: Any, **kwargs: Any) -> T:
        assert self._workers is not None
        return await asyncio.get_running_loop().run_in_executor(
            self._workers, partial(method, *args, **kwargs)
        )

    def _update_progress(self) -> None:
        assert self._service is not None
        service = self._service
        if service["phase"] == "executing" and service["running_started"] is not None:
            service["progress"] = min(
                1.0,
                (time.monotonic() - service["running_started"]) / service["duration_seconds"],
            )

    def _stop_progress(self) -> None:
        assert self._service is not None
        self._update_progress()
        self._service["phase"] = "recovering"

    def _preflight_cancelled(self) -> bool:
        assert self._service is not None
        if self._cancel_requested:
            self._service.update(outcome="cancelled", reason="cancelled before execution")
            return True
        return False

    def _execution_status(self, task: dict[str, Any]) -> None:
        assert self._service is not None
        if task["task_id"] != self._service["execution_task_id"]:
            raise RuntimeError("execution ownership changed")
        self._service["execution"] = task
        if task["phase"] == "done":
            self._stop_progress()
            self._service.update(
                outcome=task["outcome"],
                reason=task["reason"],
                handback_confirmed=task["handback_confirmed"],
            )
            if not task["handback_confirmed"]:
                self._error = "execution handback unconfirmed; controller ownership unresolved"
        elif task["phase"] == "handback":
            self._stop_progress()

    async def _run(self) -> None:
        assert self._service is not None
        service = self._service
        start_attempted = False
        try:
            status = await self._call(self._execution.get_status)
            if self._preflight_cancelled():
                return
            profile = status["profile"]
            if (
                profile.get("observation_time_basis", "acquisition")
                != self.config.observation_time_basis
            ):
                raise ValueError("execution and service observation time basis must match")
            if profile["name"] != self.config.execution_profile_name or not status["ready"]:
                raise ValueError("ready execution with the configured profile required")
            service["prediction_max_age_seconds"] = profile["max_observation_age"]
            self._event(
                "timing_profile",
                observation_time_basis=self.config.observation_time_basis,
                input_liveness_max_age_s=self.config.observation_max_age_seconds,
                prediction_max_age_s=profile["max_observation_age"],
            )
            startup = (
                profile["seed_seconds"] + profile["arm_timeout"] + profile["first_chunk_timeout"]
            )
            if (
                service["duration_seconds"]
                + startup
                + 2 * self.config.execution_rpc_timeout_seconds
                >= profile["max_duration"]
            ):
                raise ValueError("streaming duration leaves no execution startup/finish budget")
            if self.config.recovery_timeout_seconds <= profile["handback_timeout"]:
                raise ValueError("service recovery timeout must exceed native handback timeout")
            client = SonicPolicyClient(
                self.config.policies[service["policy_name"]],
                timeout_seconds=self.config.policy_timeout_seconds,
            )
            info = await self._offload(client.get_info)
            if self._preflight_cancelled():
                return
            if info.n_obs != 1 or info.image_shapes != {_IMAGE_KEY: (3, 480, 640)}:
                raise ValueError(
                    "this service requires the one-frame egocentric RGB policy profile"
                )
            self._fresh_observation()
            start_attempted = True
            task = await self._call(self._execution.start_task)
            service.update(execution_task_id=task["task_id"], phase="executing")
            self._execution_status(task)
            self._event("execution_bound", execution_task_id=task["task_id"])
            next_inference = 0.0
            last_observation = -1
            sequence = 0
            recovery_started: float | None = None
            while True:
                status = await self._call(self._execution.get_status)
                task = status["task"]
                self._execution_status(task)
                now = time.monotonic()
                if (
                    self.config.observation_time_basis == "delivery"
                    and task["phase"] not in ("done", "handback")
                    and not self._cancel_requested
                ):
                    if self._error:
                        raise ValueError(self._error)
                    try:
                        self._fresh_observation()
                    except ValueError as exc:
                        raise ValueError(f"delivery inputs expired: {exc}") from exc
                if task["phase"] == "done":
                    break
                if task["phase"] == "handback":
                    recovery_started = recovery_started or now
                    if now - recovery_started >= self.config.recovery_timeout_seconds:
                        raise TimeoutError("execution handback was not confirmed before deadline")
                elif self._cancel_requested:
                    self._stop_progress()
                    self._execution_status(
                        await self._call(self._execution.cancel_task, task["task_id"])
                    )
                elif (
                    service["running_started"] is not None
                    and now - service["running_started"] >= service["duration_seconds"]
                ):
                    self._stop_progress()
                    self._execution_status(
                        await self._call(self._execution.finish_task, task["task_id"])
                    )
                elif task["phase"] in ("waiting_for_chunk", "running"):
                    if self._inference is not None and self._inference.done():
                        pending, self._inference = self._inference, None
                        prediction = pending.result()
                        self._execution_status(
                            await self._call(
                                self._execution.submit_chunk,
                                prediction.task_id,
                                prediction.sequence,
                                prediction.observed_at,
                                prediction.actions,
                            )
                        )
                        service["accepted_predictions"] += 1
                        if service["running_started"] is None:
                            service["running_started"] = time.monotonic()
                        self._event(
                            "prediction_accepted",
                            sequence=prediction.sequence,
                            observed_at=prediction.observed_at,
                        )
                    if (
                        not self._cancel_requested
                        and self._inference is None
                        and now >= next_inference
                    ):
                        try:
                            observation = self._fresh_observation(last_observation)
                        except ValueError:
                            observation = None
                        if observation is not None:
                            last_observation = observation.sequence
                            next_inference = time.monotonic() + self.config.replan_interval_seconds
                            self._event(
                                "inference_requested",
                                sequence=sequence,
                                observed_at=observation.observed_at,
                                observation_time_basis=observation.time_basis,
                                delivery_stamps=None
                                if observation.delivery_stamps is None
                                else [asdict(s) for s in observation.delivery_stamps],
                            )
                            self._inference = asyncio.create_task(
                                self._offload(
                                    client.infer,
                                    info=info,
                                    task_id=task["task_id"],
                                    sequence=sequence,
                                    observed_at=observation.observed_at,
                                    states=observation.state[None],
                                    images=[{_IMAGE_KEY: observation.rgb}],
                                    task=service["instruction"],
                                )
                            )
                            sequence += 1
                self._update_progress()
                self._publish()
                await asyncio.sleep(0.01)
        except Exception as exc:
            service.update(outcome="failed", reason=f"{type(exc).__name__}: {exc}")
            if start_attempted and service["execution_task_id"] is None:
                # A lost start reply may have admitted a task. Never retry it or
                # cancel an unrelated task discovered through a later status.
                self._error = service["reason"]
            if service["execution_task_id"] is not None:
                await self._recover(service["reason"])
        finally:
            if self._inference is not None:
                done, _ = await asyncio.wait(
                    {self._inference}, timeout=self.config.policy_timeout_seconds + 0.1
                )
                if done:
                    # Retrieve exceptions as well as results; neither can restart control.
                    if not self._inference.cancelled() and self._inference.exception() is None:
                        service["discarded_predictions"] += 1
                    self._inference = None
                else:
                    self._error = "policy worker did not drain before deadline"
            service["phase"] = "done"
            try:
                self._event("terminal", service=service)
            except Exception as exc:
                self._error = f"service journal failed: {exc}"
                service.update(outcome="failed", reason=self._error)
            self._busy = self._inference is not None
            self._publish()

    async def _recover(self, reason: str) -> None:
        assert self._service is not None
        service = self._service
        self._stop_progress()
        try:
            self._execution_status(
                await self._call(self._execution.fail_task, service["execution_task_id"], reason)
            )
            deadline = time.monotonic() + self.config.recovery_timeout_seconds
            while service["execution"]["phase"] != "done":
                if time.monotonic() >= deadline:
                    raise TimeoutError("recovery unconfirmed")
                await asyncio.sleep(0.01)
                status = await self._call(self._execution.get_status)
                self._execution_status(status["task"])
        except Exception as exc:
            self._error = f"execution recovery uncertain: {type(exc).__name__}: {exc}"
            service.update(outcome="failed", reason=self._error, handback_confirmed=False)
