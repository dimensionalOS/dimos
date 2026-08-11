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

# mypy: ignore-errors
# ruff: noqa

from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Handshake(_message.Message):
    __slots__ = (
        "attempt_id",
        "case_id",
        "episode_id",
        "protocol_revision",
        "world_frame",
        "base_frame",
        "camera_frame",
        "rgb_encoding",
        "depth_encoding",
        "max_linear_x",
        "max_linear_y",
        "max_angular_z",
        "control_period_seconds",
        "capabilities",
    )
    ATTEMPT_ID_FIELD_NUMBER: _ClassVar[int]
    CASE_ID_FIELD_NUMBER: _ClassVar[int]
    EPISODE_ID_FIELD_NUMBER: _ClassVar[int]
    PROTOCOL_REVISION_FIELD_NUMBER: _ClassVar[int]
    WORLD_FRAME_FIELD_NUMBER: _ClassVar[int]
    BASE_FRAME_FIELD_NUMBER: _ClassVar[int]
    CAMERA_FRAME_FIELD_NUMBER: _ClassVar[int]
    RGB_ENCODING_FIELD_NUMBER: _ClassVar[int]
    DEPTH_ENCODING_FIELD_NUMBER: _ClassVar[int]
    MAX_LINEAR_X_FIELD_NUMBER: _ClassVar[int]
    MAX_LINEAR_Y_FIELD_NUMBER: _ClassVar[int]
    MAX_ANGULAR_Z_FIELD_NUMBER: _ClassVar[int]
    CONTROL_PERIOD_SECONDS_FIELD_NUMBER: _ClassVar[int]
    CAPABILITIES_FIELD_NUMBER: _ClassVar[int]
    attempt_id: str
    case_id: str
    episode_id: str
    protocol_revision: str
    world_frame: str
    base_frame: str
    camera_frame: str
    rgb_encoding: str
    depth_encoding: str
    max_linear_x: float
    max_linear_y: float
    max_angular_z: float
    control_period_seconds: float
    capabilities: _containers.RepeatedScalarFieldContainer[str]
    def __init__(
        self,
        attempt_id: _Optional[str] = ...,
        case_id: _Optional[str] = ...,
        episode_id: _Optional[str] = ...,
        protocol_revision: _Optional[str] = ...,
        world_frame: _Optional[str] = ...,
        base_frame: _Optional[str] = ...,
        camera_frame: _Optional[str] = ...,
        rgb_encoding: _Optional[str] = ...,
        depth_encoding: _Optional[str] = ...,
        max_linear_x: _Optional[float] = ...,
        max_linear_y: _Optional[float] = ...,
        max_angular_z: _Optional[float] = ...,
        control_period_seconds: _Optional[float] = ...,
        capabilities: _Optional[_Iterable[str]] = ...,
    ) -> None: ...

class LifecycleCommand(_message.Message):
    __slots__ = ("kind",)
    class Kind(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        KIND_UNSPECIFIED: _ClassVar[LifecycleCommand.Kind]
        BEGIN: _ClassVar[LifecycleCommand.Kind]
        CANCEL: _ClassVar[LifecycleCommand.Kind]

    KIND_UNSPECIFIED: LifecycleCommand.Kind
    BEGIN: LifecycleCommand.Kind
    CANCEL: LifecycleCommand.Kind
    KIND_FIELD_NUMBER: _ClassVar[int]
    kind: LifecycleCommand.Kind
    def __init__(self, kind: _Optional[_Union[LifecycleCommand.Kind, str]] = ...) -> None: ...

class PlanarControl(_message.Message):
    __slots__ = ("command_sequence", "observation_sequence", "linear_x", "linear_y", "angular_z")
    COMMAND_SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    OBSERVATION_SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    LINEAR_X_FIELD_NUMBER: _ClassVar[int]
    LINEAR_Y_FIELD_NUMBER: _ClassVar[int]
    ANGULAR_Z_FIELD_NUMBER: _ClassVar[int]
    command_sequence: int
    observation_sequence: int
    linear_x: float
    linear_y: float
    angular_z: float
    def __init__(
        self,
        command_sequence: _Optional[int] = ...,
        observation_sequence: _Optional[int] = ...,
        linear_x: _Optional[float] = ...,
        linear_y: _Optional[float] = ...,
        angular_z: _Optional[float] = ...,
    ) -> None: ...

class SubmitRoute(_message.Message):
    __slots__ = ("command_sequence", "observation_sequence")
    COMMAND_SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    OBSERVATION_SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    command_sequence: int
    observation_sequence: int
    def __init__(
        self, command_sequence: _Optional[int] = ..., observation_sequence: _Optional[int] = ...
    ) -> None: ...

class ClientMessage(_message.Message):
    __slots__ = ("handshake", "lifecycle", "control", "submit_route")
    HANDSHAKE_FIELD_NUMBER: _ClassVar[int]
    LIFECYCLE_FIELD_NUMBER: _ClassVar[int]
    CONTROL_FIELD_NUMBER: _ClassVar[int]
    SUBMIT_ROUTE_FIELD_NUMBER: _ClassVar[int]
    handshake: Handshake
    lifecycle: LifecycleCommand
    control: PlanarControl
    submit_route: SubmitRoute
    def __init__(
        self,
        handshake: _Optional[_Union[Handshake, _Mapping]] = ...,
        lifecycle: _Optional[_Union[LifecycleCommand, _Mapping]] = ...,
        control: _Optional[_Union[PlanarControl, _Mapping]] = ...,
        submit_route: _Optional[_Union[SubmitRoute, _Mapping]] = ...,
    ) -> None: ...

class Calibration(_message.Message):
    __slots__ = ("width", "height", "fx", "fy", "cx", "cy")
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    FX_FIELD_NUMBER: _ClassVar[int]
    FY_FIELD_NUMBER: _ClassVar[int]
    CX_FIELD_NUMBER: _ClassVar[int]
    CY_FIELD_NUMBER: _ClassVar[int]
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    def __init__(
        self,
        width: _Optional[int] = ...,
        height: _Optional[int] = ...,
        fx: _Optional[float] = ...,
        fy: _Optional[float] = ...,
        cx: _Optional[float] = ...,
        cy: _Optional[float] = ...,
    ) -> None: ...

class Pose(_message.Message):
    __slots__ = ("x", "y", "z", "qx", "qy", "qz", "qw")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    QX_FIELD_NUMBER: _ClassVar[int]
    QY_FIELD_NUMBER: _ClassVar[int]
    QZ_FIELD_NUMBER: _ClassVar[int]
    QW_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    def __init__(
        self,
        x: _Optional[float] = ...,
        y: _Optional[float] = ...,
        z: _Optional[float] = ...,
        qx: _Optional[float] = ...,
        qy: _Optional[float] = ...,
        qz: _Optional[float] = ...,
        qw: _Optional[float] = ...,
    ) -> None: ...

class OccupancyMap(_message.Message):
    __slots__ = (
        "frame_id",
        "resolution",
        "width",
        "height",
        "origin",
        "traversability",
        "encoding",
    )
    FRAME_ID_FIELD_NUMBER: _ClassVar[int]
    RESOLUTION_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    ORIGIN_FIELD_NUMBER: _ClassVar[int]
    TRAVERSABILITY_FIELD_NUMBER: _ClassVar[int]
    ENCODING_FIELD_NUMBER: _ClassVar[int]
    frame_id: str
    resolution: float
    width: int
    height: int
    origin: Pose
    traversability: bytes
    encoding: str
    def __init__(
        self,
        frame_id: _Optional[str] = ...,
        resolution: _Optional[float] = ...,
        width: _Optional[int] = ...,
        height: _Optional[int] = ...,
        origin: _Optional[_Union[Pose, _Mapping]] = ...,
        traversability: _Optional[bytes] = ...,
        encoding: _Optional[str] = ...,
    ) -> None: ...

class Observation(_message.Message):
    __slots__ = (
        "sequence",
        "monotonic_time_ns",
        "world_frame",
        "base_frame",
        "camera_frame",
        "world_from_base",
        "base_from_camera",
        "calibration",
        "rgb",
        "rgb_encoding",
        "depth",
        "depth_encoding",
        "static_map",
        "dropped_observations",
    )
    SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    MONOTONIC_TIME_NS_FIELD_NUMBER: _ClassVar[int]
    WORLD_FRAME_FIELD_NUMBER: _ClassVar[int]
    BASE_FRAME_FIELD_NUMBER: _ClassVar[int]
    CAMERA_FRAME_FIELD_NUMBER: _ClassVar[int]
    WORLD_FROM_BASE_FIELD_NUMBER: _ClassVar[int]
    BASE_FROM_CAMERA_FIELD_NUMBER: _ClassVar[int]
    CALIBRATION_FIELD_NUMBER: _ClassVar[int]
    RGB_FIELD_NUMBER: _ClassVar[int]
    RGB_ENCODING_FIELD_NUMBER: _ClassVar[int]
    DEPTH_FIELD_NUMBER: _ClassVar[int]
    DEPTH_ENCODING_FIELD_NUMBER: _ClassVar[int]
    STATIC_MAP_FIELD_NUMBER: _ClassVar[int]
    DROPPED_OBSERVATIONS_FIELD_NUMBER: _ClassVar[int]
    sequence: int
    monotonic_time_ns: int
    world_frame: str
    base_frame: str
    camera_frame: str
    world_from_base: Pose
    base_from_camera: Pose
    calibration: Calibration
    rgb: bytes
    rgb_encoding: str
    depth: bytes
    depth_encoding: str
    static_map: OccupancyMap
    dropped_observations: int
    def __init__(
        self,
        sequence: _Optional[int] = ...,
        monotonic_time_ns: _Optional[int] = ...,
        world_frame: _Optional[str] = ...,
        base_frame: _Optional[str] = ...,
        camera_frame: _Optional[str] = ...,
        world_from_base: _Optional[_Union[Pose, _Mapping]] = ...,
        base_from_camera: _Optional[_Union[Pose, _Mapping]] = ...,
        calibration: _Optional[_Union[Calibration, _Mapping]] = ...,
        rgb: _Optional[bytes] = ...,
        rgb_encoding: _Optional[str] = ...,
        depth: _Optional[bytes] = ...,
        depth_encoding: _Optional[str] = ...,
        static_map: _Optional[_Union[OccupancyMap, _Mapping]] = ...,
        dropped_observations: _Optional[int] = ...,
    ) -> None: ...

class Ready(_message.Message):
    __slots__ = ("negotiated",)
    NEGOTIATED_FIELD_NUMBER: _ClassVar[int]
    negotiated: Handshake
    def __init__(self, negotiated: _Optional[_Union[Handshake, _Mapping]] = ...) -> None: ...

class Acknowledgement(_message.Message):
    __slots__ = ("command_sequence", "kind")
    class Kind(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        KIND_UNSPECIFIED: _ClassVar[Acknowledgement.Kind]
        CONTROL_ACCEPTED: _ClassVar[Acknowledgement.Kind]
        ROUTE_SUBMITTED: _ClassVar[Acknowledgement.Kind]
        CANCELLATION_ACCEPTED: _ClassVar[Acknowledgement.Kind]

    KIND_UNSPECIFIED: Acknowledgement.Kind
    CONTROL_ACCEPTED: Acknowledgement.Kind
    ROUTE_SUBMITTED: Acknowledgement.Kind
    CANCELLATION_ACCEPTED: Acknowledgement.Kind
    COMMAND_SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    KIND_FIELD_NUMBER: _ClassVar[int]
    command_sequence: int
    kind: Acknowledgement.Kind
    def __init__(
        self,
        command_sequence: _Optional[int] = ...,
        kind: _Optional[_Union[Acknowledgement.Kind, str]] = ...,
    ) -> None: ...

class PublicError(_message.Message):
    __slots__ = ("code", "message", "terminal")
    CODE_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    TERMINAL_FIELD_NUMBER: _ClassVar[int]
    code: str
    message: str
    terminal: bool
    def __init__(
        self,
        code: _Optional[str] = ...,
        message: _Optional[str] = ...,
        terminal: _Optional[bool] = ...,
    ) -> None: ...

class ServerMessage(_message.Message):
    __slots__ = ("ready", "observation", "acknowledgement", "error")
    READY_FIELD_NUMBER: _ClassVar[int]
    OBSERVATION_FIELD_NUMBER: _ClassVar[int]
    ACKNOWLEDGEMENT_FIELD_NUMBER: _ClassVar[int]
    ERROR_FIELD_NUMBER: _ClassVar[int]
    ready: Ready
    observation: Observation
    acknowledgement: Acknowledgement
    error: PublicError
    def __init__(
        self,
        ready: _Optional[_Union[Ready, _Mapping]] = ...,
        observation: _Optional[_Union[Observation, _Mapping]] = ...,
        acknowledgement: _Optional[_Union[Acknowledgement, _Mapping]] = ...,
        error: _Optional[_Union[PublicError, _Mapping]] = ...,
    ) -> None: ...
