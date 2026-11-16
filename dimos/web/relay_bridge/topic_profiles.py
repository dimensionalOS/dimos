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

"""Fixed topic profiles and their updatable settings. Importing this table
never loads a media dependency.

A profile is the ceiling and the default of every setting a viewer may change:
sizes, quality and rates come back clamped to it, a point stride never falls
below it. ``normalize_settings`` is mirrored by ``normalizeTopicSettings`` in
web/shared/topics.ts, which the relay runs against ``PROFILE_OPTIONS`` as
advertised in the provider's capabilities.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import math
from typing import Any


@dataclass(frozen=True)
class MediaProfile:
    kind: str
    max_hz: float
    width: int = 0
    height: int = 0
    quality: int = 75
    keep_every: int = 1


MEDIA_PROFILES = {
    "image.jpeg.low.v1": MediaProfile("jpeg", 30, 640, 360, 60),
    "image.jpeg.balanced.v1": MediaProfile("jpeg", 30, 1280, 720, 75),
    "image.jpeg.high.v1": MediaProfile("jpeg", 30, 1920, 1080, 85),
    "depth.lossless.v1": MediaProfile("depth", 30),
    "cloud.q16.low.v1": MediaProfile("cloud", 5, keep_every=4),
    "cloud.q16.balanced.v1": MediaProfile("cloud", 20),
    "cloud.q16.full.v1": MediaProfile("cloud", 30),
    # Direct WebRTC H.264, advertised only by a provider with video enabled.
    "video.h264.v1": MediaProfile("h264", 30, 1920, 1080),
}
VIDEO_PROFILES = frozenset(
    name for name, profile in MEDIA_PROFILES.items() if profile.kind == "h264"
)
# Constrained-baseline H.264 target per output size class, in bits per second.
# docs/web/benchmarks/topic-video.md records the measurement that set them.
VIDEO_BITRATES = ((640 * 360, 1_000_000), (1280 * 720, 2_500_000), (1920 * 1080, 4_500_000))

SETTING_NAMES = ("maxHz", "maxResolution", "quality", "keepEvery", "fields", "maxBitrate")
CLOUD_FIELDS = ["intensity", "color"]
MAX_RATE = 1000


def _options(profile: MediaProfile | None) -> dict[str, Any]:
    options: dict[str, Any] = {
        "maxHz": {
            "kind": "number",
            "max": MAX_RATE,
            "default": profile.max_hz if profile else MAX_RATE,
        }
    }
    if profile is not None and profile.kind in ("jpeg", "h264"):
        options["maxResolution"] = {
            "kind": "size",
            "min": 16,
            "max": 4096,
            "default": [profile.width, profile.height],
        }
    if profile is not None and profile.kind == "jpeg":
        options["quality"] = {"kind": "int", "min": 1, "max": 100, "default": profile.quality}
    elif profile is not None and profile.kind == "h264":
        options["maxBitrate"] = {
            "kind": "int",
            "min": 100_000,
            "max": 4_500_000,
            "default": 4_500_000,
        }
    elif profile is not None and profile.kind == "cloud":
        options["keepEvery"] = {"kind": "int", "min": 1, "max": 64, "default": profile.keep_every}
        options["fields"] = {"kind": "enum", "values": CLOUD_FIELDS, "default": CLOUD_FIELDS}
    return options


# Advertised in TopicCapabilities.profileOptions; the relay validates against it.
PROFILE_OPTIONS: dict[str, dict[str, Any]] = {
    "raw": _options(None),
    **{name: _options(profile) for name, profile in MEDIA_PROFILES.items()},
}


def _number(value: Any, integer: bool) -> int | float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("option_out_of_range")
    number: int | float = value
    if integer:
        if int(number) != number:
            raise ValueError("option_out_of_range")
        return int(number)
    return number


def _clamp(name: str, option: Mapping[str, Any], value: Any) -> Any:
    default = option.get("default")
    kind = option["kind"]
    if kind in ("number", "int"):
        number = _number(value, kind == "int")
        if (
            (name == "maxHz" and number <= 0)
            or ("min" in option and number < option["min"])
            or ("max" in option and number > option["max"] and name != "maxBitrate")
        ):
            raise ValueError("option_out_of_range")
        if not isinstance(default, (int, float)):
            return number
        # The profile bounds the data: a coarser stride is a smaller output.
        return max(number, default) if name == "keepEvery" else min(number, default)
    if kind == "size":
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError("option_out_of_range")
        sides = [_number(side, True) for side in value]
        if any(side < option["min"] or side > option["max"] for side in sides):
            raise ValueError("option_out_of_range")
        limits: list[int] = option["default"]
        return [min(side, limit) for side, limit in zip(sides, limits, strict=True)]
    if kind == "enum":
        values = option["values"]
        if (
            not isinstance(value, list)
            or len(set(value)) != len(value)
            or any(item not in values for item in value)
        ):
            raise ValueError("option_out_of_range")
        return [item for item in values if item in value]
    if not isinstance(value, bool):
        raise ValueError("option_out_of_range")
    return value


def normalize_settings(profile_name: str, requested: Mapping[str, Any] | None) -> dict[str, Any]:
    """The settings in force for a request on one profile.

    Raises ``ValueError("unsupported_option")`` for a name the profile does not
    offer and ``ValueError("option_out_of_range")`` for a value outside its
    advertised range. ``None`` values count as absent (a reset).
    """
    options = PROFILE_OPTIONS[profile_name]
    requested = {name: value for name, value in (requested or {}).items() if value is not None}
    for name in requested:
        if name not in options:
            raise ValueError("unsupported_option")
    settings: dict[str, Any] = {}
    for name in SETTING_NAMES:
        option = options.get(name)
        if option is None:
            continue
        value = requested.get(name, option.get("default"))
        if value is None:
            continue
        settings[name] = _clamp(name, option, value)
    if "maxBitrate" in settings:
        settings["maxBitrate"] = min(
            settings["maxBitrate"], video_bitrate(*settings["maxResolution"])
        )
    return settings


def video_bitrate(width: int, height: int) -> int:
    """The fixed target for an encoder emitting ``width`` x ``height`` frames."""
    for pixels, bitrate in VIDEO_BITRATES:
        if width * height <= pixels:
            return bitrate
    return VIDEO_BITRATES[-1][1]
