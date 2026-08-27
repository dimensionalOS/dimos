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
from typing import Any, cast

import onnxruntime as ort  # type: ignore[import-untyped]

CUDA_PROVIDER = "CUDAExecutionProvider"
CPU_PROVIDER = "CPUExecutionProvider"
JETSON_ORT_VERSION = "1.20.1"
_DISABLE_CPU_FALLBACK = "session.disable_cpu_ep_fallback"


def prepare_sonic_onnx_runtime() -> None:
    """Validate CUDA availability and preload packaged libraries when supported."""
    ort_version = str(getattr(ort, "__version__", "unknown"))
    if platform.machine() == "aarch64" and ort_version != JETSON_ORT_VERSION:
        raise RuntimeError(
            f"SONIC on Jetson requires validated ONNX Runtime {JETSON_ORT_VERSION}; "
            f"found {ort_version}. Run bin/hardware/g1/setup-sonic-jp5."
        )

    available_providers = ort.get_available_providers()
    if CUDA_PROVIDER not in available_providers:
        raise RuntimeError(
            "SONIC requires CUDAExecutionProvider; ONNX Runtime only exposes "
            f"{available_providers}. Install the SONIC CUDA dependencies before "
            "starting the control task."
        )

    # Recent x86 wheels bundle CUDA/cuDNN libraries and expose preload_dlls().
    # Jetson's source-built ORT 1.20 wheel instead links against system CUDA.
    preload_dlls = getattr(ort, "preload_dlls", None)
    if preload_dlls is not None:
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

    session = ort.InferenceSession(
        str(model_path),
        sess_options=options,
        providers=providers,
    )
    active_providers = session.get_providers()
    if not active_providers or active_providers[0] != CUDA_PROVIDER:
        raise RuntimeError(
            f"SONIC {model_name} did not activate CUDAExecutionProvider; "
            f"active providers: {active_providers}. Refusing unsafe CPU inference."
        )
    return session
