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


@pytest.fixture
def jetson(tmp_path, monkeypatch):
    release = tmp_path / "nv_tegra_release"
    release.write_text("# R35 (release), REVISION: 3.1\n")
    monkeypatch.setattr(sonic_onnx_runtime, "L4T_RELEASE", release)
    monkeypatch.setattr(sonic_onnx_runtime.platform, "machine", lambda: "aarch64")
    return release


@pytest.mark.parametrize(("release", "version"), [(35, "1.18.1"), (36, "1.24.0")])
def test_jetson_uses_matching_runtime_and_system_libraries(jetson, mocker, release, version):
    jetson.write_text(f"# R{release} (release), REVISION: 4.3\n")
    mocker.patch.object(sonic_onnx_runtime.platform, "machine", return_value="aarch64")
    mocker.patch.object(ort, "__version__", version)
    mocker.patch.object(
        ort,
        "get_available_providers",
        return_value=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )
    preload = mocker.patch.object(ort, "preload_dlls")

    sonic_onnx_runtime.prepare_sonic_onnx_runtime()

    preload.assert_not_called()


def test_prepare_rejects_cpu_only_runtime_before_loading_models(mocker: Any) -> None:
    mocker.patch.object(
        ort,
        "get_available_providers",
        return_value=["CPUExecutionProvider"],
    )

    with pytest.raises(RuntimeError, match="requires CUDAExecutionProvider"):
        sonic_onnx_runtime.prepare_sonic_onnx_runtime()


@pytest.mark.parametrize(("release", "setup"), [(35, "jp5"), (36, "jp6")])
def test_prepare_rejects_unvalidated_ort_version_on_jetson(jetson, mocker, release, setup):
    jetson.write_text(f"# R{release} (release), REVISION: 4.3\n")
    mocker.patch.object(sonic_onnx_runtime.platform, "machine", return_value="aarch64")
    mocker.patch.object(ort, "__version__", "1.23.2")

    with pytest.raises(RuntimeError, match=f"setup-sonic-{setup}"):
        sonic_onnx_runtime.prepare_sonic_onnx_runtime()


def test_arm_server_is_not_assumed_to_be_jetpack5(jetson, mocker):
    jetson.unlink()
    mocker.patch.object(ort, "__version__", "1.23.2")
    mocker.patch.object(ort, "get_available_providers", return_value=["CUDAExecutionProvider"])
    preload = mocker.patch.object(ort, "preload_dlls")

    sonic_onnx_runtime.prepare_sonic_onnx_runtime()

    preload.assert_called_once_with()


def test_unknown_jetpack_fails_before_model_loading(jetson):
    jetson.write_text("# R38 (release), REVISION: 1.0\n")
    with pytest.raises(RuntimeError, match="supports Jetson Linux R35/R36"):
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
    session.disable_fallback.assert_called_once_with()


def test_jetpack6_disables_reduced_precision_without_relaxing_cpu_gate(jetson, mocker):
    jetson.write_text("# R36 (release), REVISION: 4.3\n")
    session = mocker.Mock()
    session.get_providers.return_value = ["CUDAExecutionProvider", "CPUExecutionProvider"]
    inference = mocker.patch.object(ort, "InferenceSession", return_value=session)

    sonic_onnx_runtime.create_sonic_session("encoder", "encoder.onnx", allow_cpu_shape_ops=False)

    assert inference.call_args.kwargs["provider_options"] == [{"use_tf32": "0"}]
    assert inference.call_args.kwargs["providers"] == ["CUDAExecutionProvider"]
    options = inference.call_args.kwargs["sess_options"]
    assert options.get_session_config_entry("session.disable_cpu_ep_fallback") == "1"


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
