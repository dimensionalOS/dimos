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

import pytest

from dimos.deps.profiles import (
    PROFILES,
    HostFacts,
    UnsupportedProfileError,
    detect_profile,
    resolve_profile,
)


def facts(system: str, machine: str, hardware: str = "", cuda: int = 0) -> HostFacts:
    return HostFacts(system=system, machine=machine, hardware=hardware, cuda_major=cuda)


@pytest.mark.parametrize(
    ("host", "expected"),
    [
        (facts("linux", "x86_64", "linux-x86-nvidia", 12), "linux-x86_64-cuda"),
        (facts("linux", "x86_64", "linux-x86-nvidia", 13), "linux-x86_64-cuda"),
        (facts("linux", "x86_64", "linux-x86-nvidia", 11), "linux-x86_64-cpu"),
        (facts("linux", "x86_64", "linux-x86-no-nvidia"), "linux-x86_64-cpu"),
        (facts("linux", "aarch64", "linux-arm-no-nvidia"), "linux-aarch64-cpu"),
        (facts("darwin", "arm64", "darwin-apple-silicon"), "macos-arm64-cpu"),
    ],
)
def test_detect_profile(host: HostFacts, expected: str) -> None:
    assert detect_profile(host) is PROFILES[expected]


@pytest.mark.parametrize(
    ("host", "fragment"),
    [
        (facts("linux", "aarch64", "orin", 12), "NVIDIA Jetson"),
        (facts("darwin", "x86_64", "darwin-intel"), "Intel macOS"),
        (facts("win32", "AMD64", "unknown"), "win32/AMD64"),
    ],
)
def test_unsupported_hosts(host: HostFacts, fragment: str) -> None:
    with pytest.raises(UnsupportedProfileError, match=fragment) as info:
        detect_profile(host)
    assert info.value.hardware == host.hardware
    assert "--environment current" in str(info.value)


def test_resolve_profile_without_override_detects() -> None:
    profile, overridden = resolve_profile(None, facts("linux", "aarch64", "linux-arm-no-nvidia"))
    assert profile.name == "linux-aarch64-cpu" and not overridden


def test_resolve_profile_override_validation() -> None:
    host = facts("linux", "x86_64", "linux-x86-no-nvidia")
    profile, overridden = resolve_profile("linux-x86_64-cuda", host)
    assert profile.accelerator == "cuda" and overridden
    with pytest.raises(ValueError, match="unknown profile 'jetson'"):
        resolve_profile("jetson", host)
    with pytest.raises(ValueError, match="targets darwin/arm64, but this host is linux/x86_64"):
        resolve_profile("macos-arm64-cpu", host)
    jetson = facts("linux", "aarch64", "orin", 12)
    profile, overridden = resolve_profile("linux-aarch64-cpu", jetson)
    assert profile.name == "linux-aarch64-cpu" and overridden


def test_profile_markers() -> None:
    assert PROFILES["linux-x86_64-cuda"].marker == (
        "sys_platform == 'linux' and platform_machine == 'x86_64'"
    )
    assert PROFILES["macos-arm64-cpu"].matches_host("darwin", "arm64")
    assert not PROFILES["macos-arm64-cpu"].matches_host("linux", "arm64")
