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
# ruff: noqa: UP006, UP035, UP045

"""State and sequence validation shared by public gateway endpoints."""

from collections import deque
import math
from threading import Lock
from typing import Callable, Deque, Iterable, Optional

import grpc  # type: ignore[import-untyped]

from . import vlnce_public_v1_pb2 as pb
from .vlnce_public_v1_pb2_grpc import (
    VlncePublicGatewayServicer,
)


class ProtocolViolationError(RuntimeError):
    """Public traffic violated the negotiated session contract."""


class BoundedObservationQueue:
    """Keep only complete observation epochs, preferring the newest under load."""

    def __init__(self, expected: pb.Handshake, capacity: int = 2) -> None:
        if capacity < 1:
            raise ValueError("observation queue capacity must be positive")
        self._expected = expected
        self._capacity = capacity
        self._items: Deque[pb.Observation] = deque()
        self._last_sequence = 0
        self._dropped = 0
        self._lock = Lock()

    @property
    def dropped(self) -> int:
        return self._dropped

    def publish(self, observation: pb.Observation) -> None:
        self._validate_complete(observation)
        with self._lock:
            if observation.sequence <= self._last_sequence:
                raise ProtocolViolationError("observation sequence must increase monotonically")
            self._last_sequence = observation.sequence
            if len(self._items) == self._capacity:
                self._items.popleft()
                self._dropped += 1
            published = pb.Observation()
            published.CopyFrom(observation)
            published.dropped_observations = self._dropped
            self._items.append(published)

    def take_latest(self) -> Optional[pb.Observation]:
        with self._lock:
            if not self._items:
                return None
            latest = self._items.pop()
            self._dropped += len(self._items)
            self._items.clear()
            latest.dropped_observations = self._dropped
            return latest

    def _validate_complete(self, observation: pb.Observation) -> None:
        expected = self._expected
        if not observation.rgb or not observation.depth:
            raise ProtocolViolationError("observation epoch is missing RGB or depth")
        if (
            observation.world_frame != expected.world_frame
            or observation.base_frame != expected.base_frame
            or observation.camera_frame != expected.camera_frame
            or observation.rgb_encoding != expected.rgb_encoding
            or observation.depth_encoding != expected.depth_encoding
        ):
            raise ProtocolViolationError("observation epoch does not match negotiated frames")
        calibrations = (observation.rgb_calibration, observation.depth_calibration)
        if any(
            not calibration.width
            or not calibration.height
            or calibration.fx <= 0
            or calibration.fy <= 0
            for calibration in calibrations
        ):
            raise ProtocolViolationError("observation epoch has incomplete calibration")
        pose = observation.world_from_base
        quaternion = (pose.qx, pose.qy, pose.qz, pose.qw)
        if not all(math.isfinite(value) for value in quaternion) or math.isclose(
            sum(value * value for value in quaternion), 0.0
        ):
            raise ProtocolViolationError("observation epoch has an invalid pose")


class BoundedCommandQueue:
    """Reject command loss instead of silently dropping accepted robot actions."""

    def __init__(self, capacity: int = 1) -> None:
        if capacity < 1:
            raise ValueError("command queue capacity must be positive")
        self._capacity = capacity
        self._items: Deque[pb.ClientMessage] = deque()

    def put(self, command: pb.ClientMessage) -> None:
        if len(self._items) == self._capacity:
            raise ProtocolViolationError("accepted command queue is full")
        self._items.append(command)

    def take(self) -> Optional[pb.ClientMessage]:
        return self._items.popleft() if self._items else None


class PublicSession:
    """Strict public command state machine with no benchmark-private fields."""

    def __init__(self, expected: pb.Handshake, command_capacity: int = 1) -> None:
        self.expected = expected
        self.phase = "awaiting_handshake"
        self.last_command_sequence = 0
        self.last_observation_sequence = 0
        self.commands = BoundedCommandQueue(command_capacity)

    def record_observation(self, sequence: int) -> None:
        if sequence <= self.last_observation_sequence:
            raise ProtocolViolationError("observation sequence must increase monotonically")
        self.last_observation_sequence = sequence

    def accept(self, message: pb.ClientMessage) -> pb.ServerMessage:
        payload = message.WhichOneof("payload")
        if self.phase == "awaiting_handshake":
            if payload != "handshake" or message.handshake != self.expected:
                raise ProtocolViolationError("incompatible or missing initial handshake")
            self.phase = "ready"
            return pb.ServerMessage(ready=pb.Ready(negotiated=self.expected))
        if payload == "handshake":
            raise ProtocolViolationError("handshake may be sent only once")
        if payload == "lifecycle":
            return self._accept_lifecycle(message.lifecycle)
        if payload == "control":
            return self._accept_control(message.control)
        if payload == "submit_route":
            return self._accept_submission(message.submit_route)
        raise ProtocolViolationError("client message has no payload")

    def _accept_lifecycle(self, command: pb.LifecycleCommand) -> pb.ServerMessage:
        if command.kind == pb.LifecycleCommand.BEGIN and self.phase == "ready":
            self.phase = "running"
            return pb.ServerMessage(
                acknowledgement=pb.Acknowledgement(kind=pb.Acknowledgement.CONTROL_ACCEPTED)
            )
        if command.kind == pb.LifecycleCommand.CANCEL and self.phase in {"ready", "running"}:
            self.phase = "cancelled"
            return pb.ServerMessage(
                acknowledgement=pb.Acknowledgement(kind=pb.Acknowledgement.CANCELLATION_ACCEPTED)
            )
        raise ProtocolViolationError("lifecycle command is invalid in the current state")

    def _accept_control(self, command: pb.PlanarControl) -> pb.ServerMessage:
        if self.phase != "running":
            raise ProtocolViolationError("motion control requires a running episode")
        self._validate_correlation(command.command_sequence, command.observation_sequence)
        values = (command.linear_x, command.linear_y, command.angular_z)
        if not all(math.isfinite(value) for value in values):
            raise ProtocolViolationError("motion control values must be finite")
        if (
            abs(command.linear_x) > self.expected.max_linear_x
            or abs(command.linear_y) > self.expected.max_linear_y
            or abs(command.angular_z) > self.expected.max_angular_z
        ):
            raise ProtocolViolationError("motion control exceeds negotiated limits")
        self.commands.put(pb.ClientMessage(control=command))
        self.last_command_sequence = command.command_sequence
        return pb.ServerMessage(
            acknowledgement=pb.Acknowledgement(
                command_sequence=command.command_sequence,
                kind=pb.Acknowledgement.CONTROL_ACCEPTED,
            )
        )

    def _accept_submission(self, command: pb.SubmitRoute) -> pb.ServerMessage:
        if self.phase != "running":
            raise ProtocolViolationError("route submission requires a running episode")
        self._validate_correlation(command.command_sequence, command.observation_sequence)
        self.commands.put(pb.ClientMessage(submit_route=command))
        self.last_command_sequence = command.command_sequence
        self.phase = "submitted"
        return pb.ServerMessage(
            acknowledgement=pb.Acknowledgement(
                command_sequence=command.command_sequence,
                kind=pb.Acknowledgement.ROUTE_SUBMITTED,
            )
        )

    def _validate_correlation(self, command_sequence: int, observation_sequence: int) -> None:
        if command_sequence != self.last_command_sequence + 1:
            raise ProtocolViolationError("command sequence must increase by one")
        if observation_sequence != self.last_observation_sequence:
            raise ProtocolViolationError("command does not reference the latest observation")


class ConformanceGateway(VlncePublicGatewayServicer):
    """Small endpoint used by conformance tests and the fake runtime."""

    def __init__(
        self,
        expected: pb.Handshake,
        on_command: Optional[Callable[[pb.ClientMessage], None]] = None,
    ) -> None:
        self.session = PublicSession(expected)
        self._on_command = on_command

    def Stream(
        self,
        request_iterator: Iterable[pb.ClientMessage],
        context: grpc.ServicerContext,
    ) -> Iterable[pb.ServerMessage]:
        try:
            for request in request_iterator:
                reply = self.session.accept(request)
                accepted = self.session.commands.take()
                if accepted is not None and self._on_command is not None:
                    self._on_command(accepted)
                yield reply
        except ProtocolViolationError as error:
            context.abort(grpc.StatusCode.FAILED_PRECONDITION, str(error))
