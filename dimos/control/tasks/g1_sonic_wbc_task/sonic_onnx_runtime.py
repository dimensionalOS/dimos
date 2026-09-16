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

"""Shared ONNX Runtime setup for SONIC inference and diagnostics."""

from __future__ import annotations

from pathlib import Path
import platform
import re
from typing import Any, cast

import onnxruntime as ort  # type: ignore[import-untyped]

CUDA_PROVIDER = "CUDAExecutionProvider"
CPU_PROVIDER = "CPUExecutionProvider"
L4T_RELEASE = Path("/etc/nv_tegra_release")
# The setup scripts pin the wheel as well as its runtime version.
JETSON_ORT_VERSIONS = {35: "1.18.1", 36: "1.24.0"}
_DISABLE_CPU_FALLBACK = "session.disable_cpu_ep_fallback"


def jetson_l4t_major() -> int | None:
    """Detect Jetson Linux; an ARM server is not necessarily a Jetson."""
    if platform.machine().lower() not in {"aarch64", "arm64"} or not L4T_RELEASE.exists():
        return None
    release = L4T_RELEASE.read_text(encoding="utf-8")
    match = re.match(r"# R(\d+)\b", release)
    major = int(match.group(1)) if match else None
    if major not in JETSON_ORT_VERSIONS:
        raise RuntimeError(f"SONIC supports Jetson Linux R35/R36; found {release.strip()}")
    return major


def validate_jetson_ort_version() -> None:
    """Reject runtimes that have not passed the matching JetPack preflight."""
    major = jetson_l4t_major()
    if major is None:
        return
    expected = JETSON_ORT_VERSIONS[major]
    actual = str(getattr(ort, "__version__", "unknown"))
    if actual != expected:
        jetpack = major - 30
        raise RuntimeError(
            f"SONIC on JetPack {jetpack} requires validated ONNX Runtime {expected}; "
            f"found {actual}. Run bin/hardware/g1/setup-sonic."
        )


def prepare_sonic_onnx_runtime() -> None:
    """Validate CUDA availability and preload packaged libraries when supported."""
    validate_jetson_ort_version()

    available_providers = ort.get_available_providers()
    if CUDA_PROVIDER not in available_providers:
        raise RuntimeError(
            "SONIC requires CUDAExecutionProvider; ONNX Runtime only exposes "
            f"{available_providers}. Install the SONIC CUDA dependencies before "
            "starting the control task."
        )

    # Recent x86 wheels bundle CUDA/cuDNN libraries and expose preload_dlls().
    # Jetson wheels instead link against the matching system CUDA/cuDNN libraries.
    preload_dlls = getattr(ort, "preload_dlls", None)
    if preload_dlls is not None and jetson_l4t_major() is None:
        preload_dlls()


def create_sonic_session(
    model_name: str,
    model_path: str | Path,
    *,
    allow_cpu_shape_ops: bool,
    session_options: Any | None = None,
) -> ort.InferenceSession:
    """Create a CUDA-first SONIC session with an explicit CPU fallback policy."""
    options = session_options if session_options is not None else cast("Any", ort).SessionOptions()
    if allow_cpu_shape_ops:
        providers = [CUDA_PROVIDER, CPU_PROVIDER]
    else:
        options.add_session_config_entry(_DISABLE_CPU_FALLBACK, "1")
        providers = [CUDA_PROVIDER]

    # Orin / JetPack 6 TF32 exceeds the planner's reference accuracy tolerance.
    # Keep full float32 math on that validated runtime, including during control.
    provider_options: list[dict[str, str]] = [{} for _ in providers]
    if jetson_l4t_major() == 36:
        provider_options[0] = {"use_tf32": "0"}
    session = ort.InferenceSession(
        str(model_path),
        sess_options=options,
        providers=providers,
        provider_options=provider_options,
    )
    active_providers = session.get_providers()
    if not active_providers or active_providers[0] != CUDA_PROVIDER:
        raise RuntimeError(
            f"SONIC {model_name} did not activate CUDAExecutionProvider; "
            f"active providers: {active_providers}. Refusing unsafe CPU inference."
        )
    # A later execution-provider failure must reach the control fault handler,
    # rather than rebuilding the session on CPU inside InferenceSession.run().
    cast("Any", session).disable_fallback()
    return session
