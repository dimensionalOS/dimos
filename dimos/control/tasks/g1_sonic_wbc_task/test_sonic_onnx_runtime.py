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

from typing import Any

import onnxruntime as ort  # type: ignore[import-untyped]
import pytest

from dimos.control.tasks.g1_sonic_wbc_task import sonic_onnx_runtime


def test_system_linked_ort_does_not_require_preload_dlls(mocker: Any) -> None:
    mocker.patch.object(
        ort,
        "get_available_providers",
        return_value=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )
    mocker.patch.object(ort, "preload_dlls", new=None)

    sonic_onnx_runtime.prepare_sonic_onnx_runtime()


def test_prepare_rejects_cpu_only_runtime_before_loading_models(mocker: Any) -> None:
    mocker.patch.object(
        ort,
        "get_available_providers",
        return_value=["CPUExecutionProvider"],
    )

    with pytest.raises(RuntimeError, match="requires CUDAExecutionProvider"):
        sonic_onnx_runtime.prepare_sonic_onnx_runtime()


def test_prepare_rejects_unvalidated_ort_version_on_jetson(mocker: Any) -> None:
    mocker.patch.object(sonic_onnx_runtime.platform, "machine", return_value="aarch64")
    mocker.patch.object(ort, "__version__", "1.24.1")

    with pytest.raises(RuntimeError, match="requires validated ONNX Runtime 1.20.1"):
        sonic_onnx_runtime.prepare_sonic_onnx_runtime()


def test_policy_session_disables_cpu_fallback(mocker: Any) -> None:
    session = mocker.Mock()
    session.get_providers.return_value = [
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]
    inference_session = mocker.patch.object(ort, "InferenceSession", return_value=session)

    result = sonic_onnx_runtime.create_sonic_session(
        "encoder", "encoder.onnx", allow_cpu_shape_ops=False
    )

    assert result is session
    options = inference_session.call_args.kwargs["sess_options"]
    assert options.get_session_config_entry("session.disable_cpu_ep_fallback") == "1"
    assert inference_session.call_args.kwargs["providers"] == ["CUDAExecutionProvider"]


def test_planner_session_explicitly_allows_audited_cpu_partition(mocker: Any) -> None:
    session = mocker.Mock()
    session.get_providers.return_value = [
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]
    inference_session = mocker.patch.object(ort, "InferenceSession", return_value=session)

    result = sonic_onnx_runtime.create_sonic_session(
        "planner", "planner.onnx", allow_cpu_shape_ops=True
    )

    assert result is session
    assert inference_session.call_args.kwargs["providers"] == [
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]


def test_session_rejects_cuda_provider_that_failed_to_load(mocker: Any) -> None:
    session = mocker.Mock()
    session.get_providers.return_value = ["CPUExecutionProvider"]
    mocker.patch.object(ort, "InferenceSession", return_value=session)

    with pytest.raises(RuntimeError, match="did not activate CUDAExecutionProvider"):
        sonic_onnx_runtime.create_sonic_session(
            "encoder", "encoder.onnx", allow_cpu_shape_ops=False
        )
