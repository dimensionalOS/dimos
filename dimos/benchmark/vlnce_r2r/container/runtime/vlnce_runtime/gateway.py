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

"""Single-episode public UDS gateway for the private VLN-CE runtime."""

from concurrent import futures
import math
import os
from queue import Empty, Full, Queue
from threading import Event, Lock
import time

import grpc
import numpy as np

from .protocol import vlnce_public_v1_pb2 as pb
from .protocol.contract import (
    BASE_FRAME,
    CAMERA_FRAME,
    DEPTH_ENCODING,
    RGB_ENCODING,
    WORLD_FRAME,
    expected_handshake,
)
from .protocol.session import ProtocolViolationError, PublicSession
from .protocol.vlnce_public_v1_pb2_grpc import (
    VlncePublicGatewayServicer,
    add_VlncePublicGatewayServicer_to_server,
)


class GatewayRuntimeError(RuntimeError):
    """The public gateway could not complete a healthy episode."""


class _EnvironmentAction:
    """One bounded request executed only by Habitat's owning main thread."""

    def __init__(self, kind, payload=None):
        self.kind = kind
        self.payload = payload
        self.done = Event()
        self.result = None
        self.error = None


class EpisodeGateway(VlncePublicGatewayServicer):
    """Own one public stream and translate accepted actions into Habitat steps."""

    def __init__(self, private_case, environment, renderer=None):
        self.environment = environment
        self.renderer = renderer
        self.session = PublicSession(expected_handshake(private_case))
        self.finished = Event()
        self.begun = Event()
        self.terminal_reason = None
        self.native_metrics = None
        self.failure = None
        self._stream_claimed = False
        self._stream_lock = Lock()
        self._next_observation_sequence = 1
        self._map_published = False
        self._environment_actions = Queue(maxsize=1)

    def Stream(self, request_iterator, context):
        with self._stream_lock:
            if self._stream_claimed:
                context.abort(grpc.StatusCode.ALREADY_EXISTS, "episode stream already claimed")
            self._stream_claimed = True
        try:
            for request in request_iterator:
                reply = self.session.accept(request)
                payload = request.WhichOneof("payload")
                if payload == "control":
                    accepted = self.session.commands.take()
                    if accepted is None:
                        raise GatewayRuntimeError("accepted control was not queued")
                    observation = self._perform("control", accepted.control)
                    yield reply
                    yield pb.ServerMessage(observation=observation)
                elif payload == "submit_route":
                    accepted = self.session.commands.take()
                    if accepted is None:
                        raise GatewayRuntimeError("accepted submission was not queued")
                    self.native_metrics = self._perform("submit")
                    self.terminal_reason = "submitted"
                    yield reply
                    self.finished.set()
                    return
                elif payload == "lifecycle" and request.lifecycle.kind == pb.LifecycleCommand.BEGIN:
                    observation = self._perform("observe")
                    yield reply
                    yield pb.ServerMessage(observation=observation)
                    self.begun.set()
                elif (
                    payload == "lifecycle" and request.lifecycle.kind == pb.LifecycleCommand.CANCEL
                ):
                    self.terminal_reason = "cancelled"
                    yield reply
                    self.finished.set()
                    return
                else:
                    yield reply
        except ProtocolViolationError as error:
            self.failure = error
            self.finished.set()
            context.abort(grpc.StatusCode.FAILED_PRECONDITION, str(error))
        except Exception as error:
            self.failure = error
            self.finished.set()
            context.abort(grpc.StatusCode.INTERNAL, "benchmark runtime failed")

    def finish_timeout(self):
        """Finalize a healthy elapsed timeout through official STOP scoring."""

        if self.finished.is_set():
            return
        self.native_metrics = self.environment.submit_route()
        if self.renderer is not None:
            self.renderer.capture_terminal("timeout")
        self.terminal_reason = "timeout"
        self.finished.set()

    def process_pending(self, timeout=0.0):
        """Execute at most one queued action from Habitat's owning thread."""

        try:
            action = self._environment_actions.get(timeout=timeout)
        except Empty:
            return False
        try:
            if action.kind == "observe":
                action.result = self._observation()
                if self.renderer is not None:
                    self.renderer.capture_initial()
            elif action.kind == "control":
                control = action.payload
                self.environment.apply_planar(
                    control.linear_x,
                    control.linear_y,
                    control.angular_z,
                    self.session.expected.control_period_seconds,
                )
                action.result = self._observation()
                if self.renderer is not None:
                    self.renderer.capture_control()
            elif action.kind == "submit":
                action.result = self.environment.submit_route()
                if self.renderer is not None:
                    self.renderer.capture_terminal("submitted")
            else:
                raise GatewayRuntimeError("unknown environment action")
        except Exception as error:
            action.error = error
            self.failure = error
            self.finished.set()
        finally:
            action.done.set()
            self._environment_actions.task_done()
        return True

    def _perform(self, kind, payload=None):
        action = _EnvironmentAction(kind, payload)
        try:
            self._environment_actions.put_nowait(action)
        except Full as error:
            raise ProtocolViolationError("environment action queue is full") from error
        if not action.done.wait(timeout=30.0):
            raise GatewayRuntimeError("environment action timed out")
        if action.error is not None:
            raise action.error
        return action.result

    def _observation(self):
        observations = self.environment.observations
        rgb = np.ascontiguousarray(observations["rgb"], dtype=np.uint8)
        depth = np.ascontiguousarray(observations["depth"], dtype="<f4")
        state = self.environment.pose
        rotation = state.rotation
        message = pb.Observation(
            sequence=self._next_observation_sequence,
            monotonic_time_ns=int(time.monotonic() * 1000000000),
            world_frame=WORLD_FRAME,
            base_frame=BASE_FRAME,
            camera_frame=CAMERA_FRAME,
            world_from_base=pb.Pose(
                x=float(state.position[0]),
                y=float(state.position[1]),
                z=float(state.position[2]),
                qx=float(rotation.x),
                qy=float(rotation.y),
                qz=float(rotation.z),
                qw=float(rotation.w),
            ),
            # Habitat's sensor looks down -Z with +Y up. A pi rotation about X
            # expresses the conventional optical frame (+Z forward, +Y down).
            base_from_camera=pb.Pose(y=1.25, qx=1.0),
            rgb_calibration=_calibration(rgb.shape[1], rgb.shape[0]),
            rgb=rgb.tobytes(order="C"),
            rgb_encoding=RGB_ENCODING,
            depth_calibration=_calibration(depth.shape[1], depth.shape[0]),
            depth=depth.tobytes(order="C"),
            depth_encoding=DEPTH_ENCODING,
        )
        if not self._map_published:
            message.static_map.CopyFrom(_occupancy_message(self.environment.static_occupancy()))
            self._map_published = True
        self.session.record_observation(message.sequence)
        self._next_observation_sequence += 1
        return message


class GatewayServer:
    """Bounded gRPC server lifecycle for one benchmark UDS endpoint."""

    def __init__(self, socket_path, gateway):
        self.socket_path = str(socket_path)
        self.gateway = gateway
        self._executor = futures.ThreadPoolExecutor(max_workers=2)
        self._server = grpc.server(self._executor)
        add_VlncePublicGatewayServicer_to_server(gateway, self._server)

    def start(self):
        directory = os.path.dirname(self.socket_path)
        if not directory or not os.path.isdir(directory):
            raise GatewayRuntimeError("public socket directory does not exist")
        if os.path.exists(self.socket_path):
            raise GatewayRuntimeError("public socket path already exists")
        address = f"unix://{self.socket_path}"
        if self._server.add_insecure_port(address) != 1:
            raise GatewayRuntimeError("could not bind public UDS endpoint")
        self._server.start()

    def stop(self, grace_seconds=2.0):
        stopped = self._server.stop(grace_seconds)
        stopped.wait(timeout=grace_seconds + 1.0)
        self._executor.shutdown(wait=True)
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)


def _calibration(width, height):
    focal = float(width) / (2.0 * math.tan(math.radians(90.0) / 2.0))
    return pb.Calibration(
        width=width,
        height=height,
        fx=focal,
        fy=focal,
        cx=(float(width) - 1.0) / 2.0,
        cy=(float(height) - 1.0) / 2.0,
    )


def _occupancy_message(occupancy):
    traversability = np.ascontiguousarray(occupancy["traversability"], dtype=np.uint8)
    expected_shape = (occupancy["height"], occupancy["width"])
    if traversability.shape != expected_shape:
        raise GatewayRuntimeError("static occupancy dimensions are inconsistent")
    return pb.OccupancyMap(
        frame_id=WORLD_FRAME,
        resolution=occupancy["resolution"],
        width=occupancy["width"],
        height=occupancy["height"],
        origin=pb.Pose(x=occupancy["origin_x"], z=occupancy["origin_z"], qw=1.0),
        traversability=traversability.tobytes(order="C"),
        encoding="uint8_traversable",
    )
