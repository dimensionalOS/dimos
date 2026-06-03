# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from collections.abc import Callable
import difflib
from typing import Any

ARM_API_ID = 7106
MODE_API_ID = 7101
ARM_TOPIC = "rt/api/arm/request"
MODE_TOPIC = "rt/api/sport/request"

# G1 Arm Actions — all use ``ARM_API_ID`` on ``ARM_TOPIC``.
G1_ARM_CONTROLS: list[tuple[str, int, str]] = [
    ("Handshake", 27, "Perform a handshake gesture with the right hand."),
    ("HighFive", 18, "Give a high five with the right hand."),
    ("Hug", 19, "Perform a hugging gesture with both arms."),
    ("HighWave", 26, "Wave with the hand raised high."),
    ("Clap", 17, "Clap hands together."),
    ("FaceWave", 25, "Wave near the face level."),
    ("LeftKiss", 12, "Blow a kiss with the left hand."),
    ("ArmHeart", 20, "Make a heart shape with both arms overhead."),
    ("RightHeart", 21, "Make a heart gesture with the right hand."),
    ("HandsUp", 15, "Raise both hands up in the air."),
    ("XRay", 24, "Hold arms in an X-ray pose position."),
    ("RightHandUp", 23, "Raise only the right hand up."),
    ("Reject", 22, "Make a rejection or 'no' gesture."),
    ("CancelAction", 99, "Cancel any current arm action and return hands to neutral position."),
]

# G1 Movement Modes — all use ``MODE_API_ID`` on ``MODE_TOPIC``.
G1_MODE_CONTROLS: list[tuple[str, int, str]] = [
    ("WalkMode", 500, "Switch to normal walking mode."),
    ("WalkControlWaist", 501, "Switch to walking mode with waist control."),
    ("RunMode", 801, "Switch to running mode."),
]

ARM_COMMANDS: dict[str, tuple[int, str]] = {
    name: (command_id, description) for name, command_id, description in G1_ARM_CONTROLS
}

MODE_COMMANDS: dict[str, tuple[int, str]] = {
    name: (command_id, description) for name, command_id, description in G1_MODE_CONTROLS
}

ARM_COMMANDS_DOC = "\n".join(
    f'- "{name}": {description}' for name, (_, description) in ARM_COMMANDS.items()
)
MODE_COMMANDS_DOC = "\n".join(
    f'- "{name}": {description}' for name, (_, description) in MODE_COMMANDS.items()
)


PublishRequest = Callable[[str, dict[str, Any]], dict[str, Any]]


def execute_g1_command(
    publish_request: PublishRequest,
    command_dict: dict[str, tuple[int, str]],
    api_id: int,
    topic: str,
    command_name: str,
    *,
    logger: Any | None = None,
) -> str:
    """Dispatch a named G1 arm/mode command via ``publish_request``.

    Returns a human-readable status string (mirrors the previous skill
    behaviour where the LLM consumed the return value).
    """
    if command_name not in command_dict:
        suggestions = difflib.get_close_matches(command_name, command_dict.keys(), n=3, cutoff=0.6)
        return f"There's no '{command_name}' command. Did you mean: {suggestions}"

    command_id, _ = command_dict[command_name]

    try:
        publish_request(topic, {"api_id": api_id, "parameter": {"data": command_id}})
        return f"'{command_name}' command executed successfully."
    except Exception as exc:
        if logger is not None:
            logger.error(f"Failed to execute {command_name}: {exc}")
        return "Failed to execute the command."


__all__ = [
    "ARM_API_ID",
    "ARM_COMMANDS",
    "ARM_COMMANDS_DOC",
    "ARM_TOPIC",
    "G1_ARM_CONTROLS",
    "G1_MODE_CONTROLS",
    "MODE_API_ID",
    "MODE_COMMANDS",
    "MODE_COMMANDS_DOC",
    "MODE_TOPIC",
    "PublishRequest",
    "execute_g1_command",
]
