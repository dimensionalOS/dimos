"""Data models used by the DimOS Studio local API."""

from typing import Literal

from pydantic import BaseModel, Field


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
    navigation_speed_scale: float = Field(default=0.35, ge=0.1, le=1.0)


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


class MissionCreateRequest(BaseModel):
    objective: str = Field(min_length=1, max_length=2000)


class MissionStartRequest(BaseModel):
    confirmation: str = ""


class MissionActionRequest(BaseModel):
    reason: str = Field(default="", max_length=500)
