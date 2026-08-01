"""DimOS benchmark runtime protocol package."""

from dimos_runtime_protocol.compat import CompatibilityResult, check_compatible
from dimos_runtime_protocol.models import (
    ArtifactOutput,
    CommandMode,
    EpisodeResetRequest,
    EpisodeResetResponse,
    ErrorResponse,
    HealthResponse,
    MotorActionFrame,
    MotorDescription,
    MotorStateFrame,
    ObservationFrame,
    ObservationKind,
    ProtocolVersion,
    RobotMotorSurface,
    RuntimeDescription,
    ScoreOutput,
    StepRequest,
    StepResponse,
)
from dimos_runtime_protocol.version import PROTOCOL_VERSION

__all__ = [
    "PROTOCOL_VERSION",
    "ArtifactOutput",
    "CommandMode",
    "CompatibilityResult",
    "EpisodeResetRequest",
    "EpisodeResetResponse",
    "ErrorResponse",
    "HealthResponse",
    "MotorActionFrame",
    "MotorDescription",
    "MotorStateFrame",
    "ObservationFrame",
    "ObservationKind",
    "ProtocolVersion",
    "RobotMotorSurface",
    "RuntimeDescription",
    "ScoreOutput",
    "StepRequest",
    "StepResponse",
    "check_compatible",
]
