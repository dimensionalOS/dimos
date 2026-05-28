from __future__ import annotations

import os
from pathlib import Path

import yaml
from dotenv import load_dotenv
from pydantic import BaseModel


class RobotConfig(BaseModel):
    ip: str = ""
    obstacle_avoidance: bool = True
    camera_resize: tuple[int, int] = (640, 480)
    connect_timeout_sec: float = 15.0


class WaypointConfig(BaseModel):
    id: str
    pos: tuple[float, float, float]
    yaw: float


class PatrolConfig(BaseModel):
    loop_forever: bool = True
    scan_turns: int = 1
    scan_pause_sec: float = 1.0
    motion_settle_sec: float = 0.05
    forward_steps_per_cycle: int = 5
    forward_speed_mps: float = 0.60
    forward_step_duration_sec: float = 1.25
    sweep_yaw_radps: float = 0.24
    sweep_turn_duration_sec: float = 0.28


class DetectionConfig(BaseModel):
    enabled: bool = True
    model_name: str = "yolo11n.pt"
    interval_sec: float = 0.5
    conf_threshold: float = 0.25
    cooldown_sec: int = 300
    detection_classes: list[str]


class AlertConfig(BaseModel):
    audio_file: str
    clip_duration_sec: int = 5
    buffer_seconds: int = 10
    capture_fps: int = 15


class WebConfig(BaseModel):
    host: str = "0.0.0.0"
    port: int = 8080
    stream_fps: int = 10


class AisleConfig(BaseModel):
    enabled: bool = True
    x_min_m: float = 0.4
    x_max_m: float = 2.0
    half_width_m: float = 0.45
    min_points_in_zone: int = 12
    min_z_m: float = -0.35
    max_z_m: float = 1.20
    cell_size_m: float = 0.12
    min_occupied_cells: int = 2
    alert_repeat_sec: float = 3.0
    closeup_stop_distance_m: float = 0.8
    approach_step_m: float = 0.25
    max_approach_steps: int = 3
    turn_step_deg: float = 12.0
    reclear_consecutive_frames: int = 2


class Settings(BaseModel):
    robot: RobotConfig
    waypoints: list[WaypointConfig]
    patrol: PatrolConfig
    aisle: AisleConfig
    detection: DetectionConfig
    alert: AlertConfig
    web: WebConfig
    telegram_bot_token: str
    telegram_owner_chat_id: int


def load_config(yaml_path: Path = Path("config.yaml")) -> Settings:
    load_dotenv()
    data = yaml.safe_load(yaml_path.read_text())
    data["telegram_bot_token"] = os.environ["TELEGRAM_BOT_TOKEN"].strip()
    data["telegram_owner_chat_id"] = int(os.environ["TELEGRAM_OWNER_CHAT_ID"])

    robot_ip = os.environ.get("ROBOT_IP", "").strip()
    if robot_ip:
        data["robot"]["ip"] = robot_ip

    return Settings(**data)
