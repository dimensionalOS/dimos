"""Data models used by the DimOS Studio local API."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field


class StudioSettings(BaseModel):
    """Persisted settings for one local DimOS Studio installation."""

    robot_ip: str = "30.201.217.128"
    robot_name: str = "Go2_49060"
    signal_port: int = Field(default=9991, ge=1, le=65535)
    blueprint: str = "dimos-go2-studio.go2"
    agent_model: str = "gpt-5.6-luna"
    detection_model: Literal["qwen", "moondream"] = "moondream"
    viewer: Literal["rerun", "none"] = "rerun"
    system_prompt: str = (
        "You control a Unitree Go2 through DimOS. Prefer observation before action. "
        "Never move when safety state is unclear, and stop when a human takes over."
    )
    obstacle_avoidance: bool = True
    movement_locked: bool = True
    navigation_speed_scale: float = Field(default=0.18, ge=0.1, le=1.0)


class RobotCheckRequest(BaseModel):
    robot_ip: str
    signal_port: int = Field(default=9991, ge=1, le=65535)


class RuntimeStartRequest(BaseModel):
    mode: Literal[
        "simulation",
        "hardware_readonly",
        "hardware",
        "hosted_teleop",
    ] = "simulation"
    confirmation: str = ""


class SkillSourceRequest(BaseModel):
    source: str


class AgentMessageRequest(BaseModel):
    message: str = Field(min_length=1, max_length=4000)


class TeleopKeyRequest(BaseModel):
    api_key: str = Field(min_length=8, max_length=500)


class _StrictMissionRequest(BaseModel):
    """Strict HTTP ingress DTO; domain contracts live in the Go2 extension."""

    model_config = ConfigDict(extra="forbid", str_strip_whitespace=True)


class MissionCreateRequest(_StrictMissionRequest):
    objective: str = Field(min_length=1, max_length=2000)


class MissionStartRequest(_StrictMissionRequest):
    confirmation: str = ""


class MissionActionRequest(_StrictMissionRequest):
    reason: str = Field(default="", max_length=500)


class StageTwoNavigateRequest(_StrictMissionRequest):
    instruction_id: str = Field(
        min_length=8,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9_.:-]*$",
    )
    destination: str = Field(min_length=1, max_length=200)


class StageTwoCancelRequest(_StrictMissionRequest):
    task_id: str = Field(
        min_length=8,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9_.:-]*$",
    )


class StageTwoConfirmPlaceRequest(_StrictMissionRequest):
    name: str = Field(min_length=1, max_length=200)
    aliases: list[str] = Field(default_factory=list, max_length=20)


class StageTwoReplyRequest(_StrictMissionRequest):
    event: Literal["agent.reply.completed"]
    reply_id: str = Field(
        min_length=8,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9_.:-]*$",
    )
    instruction_id: str = Field(
        min_length=8,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9_.:-]*$",
    )
    text: str = Field(min_length=1, max_length=20_000)
    completed_at: str = Field(min_length=10, max_length=64)
