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

"""Hardware profiles: the tested platform and accelerator combinations.

A profile is more specific than Python's environment markers: it decides
which backend extra provides ``onnxruntime`` (``cpu`` or ``cuda``) and it
records what a managed environment was prepared for. Hosts outside the
tested matrix (Jetson, Intel macOS) are detected and reported rather than
guessed.
"""

from __future__ import annotations

from dataclasses import dataclass
import platform
import sys
from typing import Literal

from dimos.utils.logging_config import setup_logger
from dimos.utils.nvidia_env import detect_cuda_major, detect_hardware

logger = setup_logger()

Accelerator = Literal["cpu", "cuda"]
JETSON_HARDWARE = ("thor", "orin", "xavier", "nano")
MIN_CUDA_MAJOR = 12


@dataclass(frozen=True)
class Profile:
    name: str
    system: str
    """``sys.platform`` value: ``linux`` or ``darwin``."""
    machine: str
    """``platform.machine()`` value: ``x86_64``, ``aarch64`` or ``arm64``."""
    accelerator: Accelerator
    marker: str
    """Environment marker selecting this platform for uv's ``environments``."""

    def matches_host(self, system: str, machine: str) -> bool:
        return self.system == system and self.machine == machine


def _profile(name: str, system: str, machine: str, accelerator: Accelerator) -> Profile:
    marker = f"sys_platform == '{system}' and platform_machine == '{machine}'"
    return Profile(name, system, machine, accelerator, marker)


PROFILES: dict[str, Profile] = {
    profile.name: profile
    for profile in (
        _profile("linux-x86_64-cpu", "linux", "x86_64", "cpu"),
        _profile("linux-x86_64-cuda", "linux", "x86_64", "cuda"),
        _profile("linux-aarch64-cpu", "linux", "aarch64", "cpu"),
        _profile("macos-arm64-cpu", "darwin", "arm64", "cpu"),
    )
}


@dataclass(frozen=True)
class HostFacts:
    system: str
    machine: str
    hardware: str
    cuda_major: int

    @classmethod
    def detect(cls) -> HostFacts:
        return cls(
            system=sys.platform,
            machine=platform.machine(),
            hardware=detect_hardware(),
            cuda_major=detect_cuda_major(),
        )


class UnsupportedProfileError(RuntimeError):
    """The host is outside the tested profile matrix."""

    def __init__(self, hardware: str, message: str) -> None:
        super().__init__(message)
        self.hardware = hardware


def detect_profile(facts: HostFacts | None = None) -> Profile:
    facts = facts or HostFacts.detect()
    if facts.system == "darwin":
        if facts.machine == "arm64":
            return PROFILES["macos-arm64-cpu"]
        raise UnsupportedProfileError(
            facts.hardware,
            "Intel macOS has no tested profile; use --environment current to run in the "
            "environment you manage yourself.",
        )
    if facts.system == "linux" and facts.machine == "aarch64":
        if facts.hardware in JETSON_HARDWARE:
            raise UnsupportedProfileError(
                facts.hardware,
                f"NVIDIA Jetson ({facts.hardware}) has no tested profile yet; pass "
                "--profile linux-aarch64-cpu to use CPU wheels, or --environment current "
                "to run in the environment you manage yourself.",
            )
        return PROFILES["linux-aarch64-cpu"]
    if facts.system == "linux" and facts.machine == "x86_64":
        if facts.cuda_major >= MIN_CUDA_MAJOR:
            return PROFILES["linux-x86_64-cuda"]
        if facts.cuda_major > 0:
            logger.warning(
                "The NVIDIA driver supports CUDA %d; the cuda profile needs %d+, using cpu",
                facts.cuda_major,
                MIN_CUDA_MAJOR,
            )
        return PROFILES["linux-x86_64-cpu"]
    raise UnsupportedProfileError(
        facts.hardware,
        f"{facts.system}/{facts.machine} has no tested profile; use --environment current "
        "to run in the environment you manage yourself.",
    )


def resolve_profile(override: str | None, facts: HostFacts | None = None) -> tuple[Profile, bool]:
    """The profile to plan with and whether it was chosen explicitly.

    An override must match the host's OS and architecture; a different
    accelerator is allowed (with a warning when the host cannot run it), which
    is what image builds and CPU fallbacks need.
    """
    if override is None:
        return detect_profile(facts), False
    profile = PROFILES.get(override)
    if profile is None:
        raise ValueError(
            f"unknown profile {override!r}; choose one of {', '.join(sorted(PROFILES))}"
        )
    facts = facts or HostFacts.detect()
    if not profile.matches_host(facts.system, facts.machine):
        raise ValueError(
            f"profile {override!r} targets {profile.system}/{profile.machine}, "
            f"but this host is {facts.system}/{facts.machine}"
        )
    if profile.accelerator == "cuda" and facts.cuda_major < MIN_CUDA_MAJOR:
        logger.warning(
            "Profile %s needs an NVIDIA driver with CUDA %d+; this host reports %d",
            profile.name,
            MIN_CUDA_MAJOR,
            facts.cuda_major,
        )
    return profile, True
