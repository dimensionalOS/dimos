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

"""Inference backend choice and the explicit platform support table for ``dimos prepare``."""

from __future__ import annotations

from collections.abc import Iterable
import os
from pathlib import Path
import platform
import sys
from typing import Literal

from dimos.utils.nvidia_env import detect_cuda_major

Backend = Literal["cpu", "cuda"]
BACKEND_CHOICES = ("auto", "cpu", "cuda")
MIN_CUDA_MAJOR = 12
SUPPORTED_HOSTS = frozenset({("linux", "x86_64"), ("linux", "aarch64"), ("darwin", "arm64")})
CUDA_HOST = ("linux", "x86_64")
# Bundles whose feature extras carry platform-limited packages. Checked against
# pyproject.toml by dimos/deps/test_backend.py.
PLANNING_BUNDLES = frozenset({"runtime-manipulation", "runtime-unitree-dds"})
DDS_BUNDLES = frozenset({"runtime-unitree-dds"})
DDS_DOC = "docs/usage/transports/dds.md"
DEPENDENCIES_DOC = "docs/usage/dependencies.md"


class UnsupportedError(Exception):
    """The requested bundles or backend cannot be installed on this host."""


def host() -> tuple[str, str]:
    """``(sys.platform, machine)``, for example ``("linux", "x86_64")``."""
    return sys.platform, platform.machine()


def python_version() -> str:
    return f"{sys.version_info[0]}.{sys.version_info[1]}"


def resolve_backend(choice: str) -> tuple[Backend, str]:
    """Pick the inference-provider extra for ``choice``. Returns ``(backend, reason)``."""
    if choice not in BACKEND_CHOICES:
        raise ValueError(f"backend must be one of {', '.join(BACKEND_CHOICES)}, not {choice!r}")
    cuda_host = host() == CUDA_HOST
    cuda_major = detect_cuda_major() if cuda_host else 0
    if choice == "cpu":
        return "cpu", "requested"
    if choice == "cuda":
        if not cuda_host:
            raise UnsupportedError(
                "--backend cuda is supported on Linux x86_64 only; use --backend cpu"
            )
        if cuda_major:
            return "cuda", f"requested; the NVIDIA driver supports CUDA {cuda_major}.x"
        return (
            "cuda",
            "requested; no NVIDIA driver detected, so the GPU is unusable until one exists",
        )
    if cuda_major >= MIN_CUDA_MAJOR:
        return "cuda", f"auto: the NVIDIA driver supports CUDA {cuda_major}.x"
    if cuda_major:
        return (
            "cpu",
            f"auto: the NVIDIA driver supports CUDA {cuda_major}.x; cuda needs {MIN_CUDA_MAJOR}+",
        )
    if cuda_host:
        return "cpu", "auto: no NVIDIA driver detected"
    return "cpu", "auto: the cuda backend is Linux x86_64 only"


def cyclonedds_wheel_available() -> bool:
    """cyclonedds 0.10.5 ships wheels only for CPython 3.10 on Linux x86_64 and macOS arm64."""
    return python_version() == "3.10" and host() in {("linux", "x86_64"), ("darwin", "arm64")}


def cyclonedds_home_configured() -> bool:
    """The source build of cyclonedds finds the C library through CYCLONEDDS_HOME."""
    home = os.environ.get("CYCLONEDDS_HOME", "")
    return bool(home) and Path(home).is_dir()


def check_supported(bundles: Iterable[str], backend: Backend) -> None:
    """Fail before installing anything for a known-unsupported bundle/host combination."""
    selected = set(bundles)
    if host() not in SUPPORTED_HOSTS:
        raise UnsupportedError(
            f"{sys.platform}/{platform.machine()} is not a supported host; "
            "dimos prepare supports Linux x86_64, Linux aarch64 and macOS arm64"
        )
    if backend == "cuda" and host() != CUDA_HOST:
        raise UnsupportedError(
            "--backend cuda is supported on Linux x86_64 only; use --backend cpu"
        )
    planning = sorted(selected & PLANNING_BUNDLES)
    if planning and host() == ("darwin", "arm64") and python_version() != "3.12":
        raise UnsupportedError(
            f"{', '.join(planning)} need Python 3.12 on Apple silicon: Drake 1.45.0 ships only a "
            f"cp312 macOS 14 wheel and this interpreter is Python {python_version()}"
        )
    dds = sorted(selected & DDS_BUNDLES)
    if dds and not cyclonedds_wheel_available() and not cyclonedds_home_configured():
        raise UnsupportedError(
            f"{', '.join(dds)} need the CycloneDDS C library: cyclonedds has no wheel for "
            f"Python {python_version()} on this host and CYCLONEDDS_HOME does not point at an "
            f"installation. Set it up first: {DDS_DOC}"
        )


def limitations(bundles: Iterable[str], backend: Backend) -> list[str]:
    """Known setup limits for these bundles on this host. Informational, never failures."""
    selected = set(bundles)
    notes: list[str] = []
    if host() == CUDA_HOST:
        notes.append("Linux x86_64 installs CUDA-capable PyTorch wheels even with --backend cpu")
    else:
        notes.append("the cuda backend is Linux x86_64 only; Jetson CUDA is not supported")
    if python_version() == "3.10" and host() != ("darwin", "arm64"):
        notes.append("PGO relocalization (gtsam-extended) has no Python 3.10 wheel here")
    if selected & PLANNING_BUNDLES:
        if host() == ("linux", "aarch64"):
            notes.append("Drake has no Linux aarch64 wheel; roboplan (the default planner) works")
        if host() == CUDA_HOST and python_version() != "3.12":
            notes.append("the A750 arm adapter needs Python 3.12 (the only a750-control wheel)")
        notes.append(
            "Galaxea A1Z needs the vendor a1z/gs_usb/pyusb packages "
            "(docs/capabilities/manipulation/a1z.md); GraspGenX and LeRobot policies run in "
            "isolated uv environments (uv at runtime, GraspGenX on Linux x86_64 only)"
        )
    if selected & DDS_BUNDLES:
        notes.append(
            "cyclonedds builds against the CycloneDDS C library except on Python 3.10 "
            f"Linux x86_64 / macOS arm64 ({DDS_DOC})"
        )
    if "runtime-drone" in selected:
        notes.append("DJI video streaming needs gst-launch-1.0 (GStreamer)")
    notes.append(
        "native modules (lidar drivers, SLAM, planners, RealSense) are built separately "
        "with nix/cargo: docs/usage/native_modules.md"
    )
    notes.append(
        "ROS 2, the ZED SDK, GStreamer/PyGObject, Deno (downloaded on demand), ffmpeg, "
        f"PortAudio and API keys are separate prerequisites: {DEPENDENCIES_DOC}"
    )
    return notes
