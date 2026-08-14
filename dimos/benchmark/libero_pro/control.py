"""Evaluator-only client for the privileged LIBERO-PRO control listener."""

from __future__ import annotations

from typing import cast

import grpc  # type: ignore[import-untyped]

from dimos.benchmark.libero_pro.models import LiberoTaskManifest
from dimos.benchmark.libero_pro.proto import libero_pro_pb2 as pb2, libero_pro_pb2_grpc as pb2_grpc


class EvaluationControlClient:
    def __init__(self, endpoint: str, token: str) -> None:
        self._channel = grpc.insecure_channel(endpoint)
        self._stub = pb2_grpc.EvaluationControlStub(self._channel)
        self._metadata = (("authorization", f"Bearer {token}"),)

    def wait_ready(self, timeout_s: float = 30.0) -> None:
        grpc.channel_ready_future(self._channel).result(timeout=timeout_s)
        self._stub.GetHealth(pb2.Empty(), metadata=self._metadata, timeout=timeout_s)

    def initialize(
        self,
        manifest: LiberoTaskManifest,
        init_state_index: int,
        *,
        timeout_s: float = 120.0,
    ) -> None:
        ready = self._stub.InitializeTrial(
            pb2.InitializeTrialRequest(
                suite=manifest.task.suite,
                task_order_index=manifest.task.task_order_index,
                task_index=manifest.task.task_index,
                init_state_index=init_state_index,
                horizon_ticks=manifest.contract.horizon_ticks,
                control_frequency_hz=manifest.contract.control_frequency_hz,
                settling_ticks=manifest.contract.settling_ticks,
            ),
            metadata=self._metadata,
            timeout=timeout_s,
        )
        if ready.task_name != manifest.task.task_name:
            raise RuntimeError("container task name does not match task manifest")
        if ready.instruction != manifest.task.instruction:
            raise RuntimeError("container instruction does not match task manifest")

    def start(self) -> None:
        self._stub.StartTrial(pb2.Empty(), metadata=self._metadata, timeout=5)

    def wait_terminal(self, timeout_s: float) -> pb2.TerminalResult:
        return cast(
            "pb2.TerminalResult",
            self._stub.WaitForTerminal(pb2.Empty(), metadata=self._metadata, timeout=timeout_s),
        )

    def cancel(self) -> pb2.TerminalResult:
        return cast(
            "pb2.TerminalResult",
            self._stub.CancelTrial(pb2.Empty(), metadata=self._metadata, timeout=5),
        )

    def close(self) -> None:
        self._channel.close()
