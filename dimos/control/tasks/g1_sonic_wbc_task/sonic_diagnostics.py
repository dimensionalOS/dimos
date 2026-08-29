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

"""Offline compatibility, accuracy, and latency checks for SONIC on G1."""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass
from functools import partial
import hashlib
import json
import os
from pathlib import Path
import platform
import tempfile
import time
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.control.tasks.g1_sonic_wbc_task.sonic_hardware import (
    ensure_sonic_max_performance,
)
from dimos.control.tasks.g1_sonic_wbc_task.sonic_onnx_runtime import (
    CPU_PROVIDER,
    JETSON_ORT_VERSION,
    create_sonic_session,
    prepare_sonic_onnx_runtime,
)
from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import SONIC_MODEL_PROFILES
from dimos.utils.data import get_data_dir

CUDA_HOME = Path("/usr/local/cuda-11.8")
L4T_RELEASE = Path("/etc/nv_tegra_release")
CUDNN_LIBRARY = Path("/usr/lib/aarch64-linux-gnu/libcudnn.so.8")
REFERENCE_PATH = Path(__file__).with_name("sonic_doctor_reference.json")

EXPECTED_MODEL_SHA256 = (
    {f"{profile.name} encoder": profile.encoder_sha256 for profile in SONIC_MODEL_PROFILES.values()}
    | {
        f"{profile.name} decoder": profile.decoder_sha256
        for profile in SONIC_MODEL_PROFILES.values()
    }
    | {
        "planner": "39b553e197f62f077975ba38512bc04781a3fc37c2af7c6756e04629f760edea",
    }
)
ALLOWED_PLANNER_CPU_OPS = frozenset({"Atan", "Slice", "Concat", "ArgMax", "Tile", "ArgMin", "Clip"})
MAX_PLANNER_CPU_EVENTS = 56
MAX_POLICY_P99_MS = 15.0
MAX_PLANNER_P95_MS = 100.0

_Check = tuple[str, Callable[[], str]]
_Array = NDArray[Any]


@dataclass(frozen=True)
class SonicDiagnosticCheck:
    """One user-visible SONIC preflight result."""

    name: str
    passed: bool
    detail: str


@dataclass(frozen=True)
class SonicDiagnosticReport:
    """Complete SONIC preflight result."""

    checks: tuple[SonicDiagnosticCheck, ...]

    @property
    def passed(self) -> bool:
        return all(check.passed for check in self.checks)


@dataclass(frozen=True)
class SonicModelPaths:
    """Both released SONIC policy bundles and their shared planner."""

    profiles: dict[str, tuple[Path, Path]]
    planner: Path


def resolve_sonic_model_paths() -> SonicModelPaths:
    """Resolve the same model overrides used by the G1 SONIC blueprint."""
    model_dir_env = os.environ.get("SONIC_MODEL_DIR")
    model_dir = Path(model_dir_env) if model_dir_env else get_data_dir("sonic")
    planner_env = os.environ.get("SONIC_PLANNER_PATH")
    planner = Path(planner_env) if planner_env else model_dir / "planner_sonic.onnx"
    return SonicModelPaths(
        profiles={
            profile.name: (
                Path(model_dir / profile.model_subdir / "model_encoder.onnx"),
                Path(model_dir / profile.model_subdir / "model_decoder.onnx"),
            )
            for profile in SONIC_MODEL_PROFILES.values()
        },
        planner=Path(planner),
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _host_checks() -> tuple[_Check, ...]:
    compat_library = CUDA_HOME / "compat/libcuda.so"
    cuda_runtime = CUDA_HOME / "lib64/libcudart.so.11.0"
    ort_version = str(getattr(ort, "__version__", "unknown"))
    return (
        (
            "architecture",
            lambda: (
                "aarch64"
                if platform.machine() == "aarch64"
                else (_raise(f"expected aarch64, found {platform.machine()}"))
            ),
        ),
        ("Jetson Linux", _verify_l4t),
        ("CUDA 11.8 runtime", lambda: _require_path(cuda_runtime)),
        ("CUDA 11.8 compatibility driver", lambda: _require_path(compat_library)),
        ("cuDNN 8", lambda: _require_path(CUDNN_LIBRARY)),
        (
            "ONNX Runtime",
            lambda: (
                ort_version
                if ort_version == JETSON_ORT_VERSION
                else _raise(
                    f"expected {JETSON_ORT_VERSION}, found {ort_version}; "
                    "run bin/hardware/g1/setup-sonic-jp5"
                )
            ),
        ),
        ("CUDA execution provider", _verify_cuda_provider),
        ("Jetson MAXN and locked clocks", _verify_max_performance),
    )


def _verify_max_performance() -> str:
    ensure_sonic_max_performance()
    return "MAXN; CPU/GPU clocks locked"


def _raise(message: str) -> str:
    raise RuntimeError(message)


def _require_path(path: Path) -> str:
    if not path.exists():
        raise RuntimeError(f"missing {path}")
    return str(path)


def _verify_l4t() -> str:
    try:
        release = L4T_RELEASE.read_text(encoding="utf-8").splitlines()[0]
    except OSError as exc:
        raise RuntimeError(f"cannot read {L4T_RELEASE}: {exc}") from exc
    if not release.startswith("# R35"):
        raise RuntimeError(f"expected L4T R35, found {release}")
    return release


def _verify_cuda_provider() -> str:
    providers = ort.get_available_providers()
    if "CUDAExecutionProvider" not in providers:
        raise RuntimeError(f"unavailable; ONNX Runtime exposes {providers}")
    return ", ".join(providers)


def _run_checks(checks: tuple[_Check, ...]) -> list[SonicDiagnosticCheck]:
    results: list[SonicDiagnosticCheck] = []
    for name, check in checks:
        try:
            results.append(SonicDiagnosticCheck(name, True, check()))
        except Exception as exc:
            results.append(SonicDiagnosticCheck(name, False, str(exc)))
    return results


def _model_checks(paths: SonicModelPaths) -> tuple[_Check, ...]:
    models = (
        [
            (f"{profile_name} encoder", encoder)
            for profile_name, (encoder, _decoder) in paths.profiles.items()
        ]
        + [
            (f"{profile_name} decoder", decoder)
            for profile_name, (_encoder, decoder) in paths.profiles.items()
        ]
        + [("planner", paths.planner)]
    )
    return tuple(
        (
            f"{name} model",
            lambda name=name, path=path: _verify_model(name, path),
        )
        for name, path in models
    )


def _verify_model(name: str, path: Path) -> str:
    if not path.is_file():
        raise RuntimeError(f"missing {path}")
    actual = _sha256(path)
    expected = EXPECTED_MODEL_SHA256[name]
    if actual != expected:
        raise RuntimeError(f"hash {actual} does not match validated hash {expected}")
    return f"{path} ({actual})"


def _planner_inputs() -> dict[str, _Array]:
    standing_qpos = np.array(
        [
            0.0,
            0.0,
            0.78874,
            1.0,
            0.0,
            0.0,
            0.0,
            -0.312,
            0.669,
            -0.312,
            0.669,
            0.0,
            0.0,
            0.0,
            -0.363,
            0.0,
            -0.363,
            0.0,
            0.6,
            0.0,
            0.0,
            0.0,
            0.0,
            0.2,
            0.0,
            0.0,
            -0.2,
            0.6,
            0.0,
            0.0,
            0.2,
            0.0,
            0.2,
            0.0,
            0.0,
            0.0,
        ],
        dtype=np.float32,
    )
    return {
        "context_mujoco_qpos": np.tile(standing_qpos, (1, 4, 1)),
        "target_vel": np.array([-1.0], dtype=np.float32),
        "mode": np.array([0], dtype=np.int64),
        "movement_direction": np.zeros((1, 3), dtype=np.float32),
        "facing_direction": np.array([[1.0, 0.0, 0.0]], dtype=np.float32),
        "random_seed": np.array([42], dtype=np.int64),
        "has_specific_target": np.zeros((1, 1), dtype=np.int64),
        "specific_target_positions": np.zeros((1, 4, 3), dtype=np.float32),
        "specific_target_headings": np.zeros((1, 4), dtype=np.float32),
        "allowed_pred_num_tokens": np.array([[1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0]], dtype=np.int64),
        "height": np.array([-1.0], dtype=np.float32),
    }


def _benchmark(operation: Callable[[], Any], samples: int, percentile: float) -> float:
    for _ in range(3):
        operation()
    durations_ms: list[float] = []
    for _ in range(samples):
        started = time.perf_counter()
        operation()
        durations_ms.append((time.perf_counter() - started) * 1000.0)
    return float(np.percentile(durations_ms, percentile))


def _run_policy_pair(
    encoder: Any,
    decoder: Any,
    encoder_input: _Array,
    decoder_input: _Array,
) -> None:
    encoder.run(None, {encoder.get_inputs()[0].name: encoder_input})
    decoder.run(None, {decoder.get_inputs()[0].name: decoder_input})


def _profile_cpu_ops(profile_path: Path) -> Counter[str]:
    events = json.loads(profile_path.read_text(encoding="utf-8"))
    return Counter(
        str(event.get("args", {}).get("op_name", "unknown"))
        for event in events
        if event.get("cat") == "Node"
        and str(event.get("name", "")).endswith("_kernel_time")
        and event.get("args", {}).get("provider") == CPU_PROVIDER
    )


def _reference_outputs() -> dict[str, _Array]:
    raw = json.loads(REFERENCE_PATH.read_text(encoding="utf-8"))
    return {
        "encoder": np.asarray(raw["encoder"], dtype=np.float32),
        "decoder": np.asarray(raw["decoder"], dtype=np.float32),
        "planner_qpos": np.asarray(raw["planner_qpos"], dtype=np.float32),
        "planner_frames": np.asarray(raw["planner_frames"], dtype=np.int32),
    }


def _accuracy_detail(name: str, actual: _Array, expected: _Array) -> str:
    difference = np.abs(np.asarray(actual, dtype=np.float64) - expected.astype(np.float64))
    if not np.all(np.isfinite(actual)):
        raise RuntimeError(f"{name} produced non-finite values")
    maximum = float(np.max(difference))
    mean = float(np.mean(difference))
    max_limit = 5e-3 if name == "planner" else 1e-4
    mean_limit = 1e-3 if name == "planner" else 1e-4
    if maximum > max_limit or mean > mean_limit:
        raise RuntimeError(
            f"max error {maximum:.6g}, mean error {mean:.6g}; limits are "
            f"{max_limit:.6g}/{mean_limit:.6g}"
        )
    return f"max error {maximum:.6g}, mean error {mean:.6g}"


def _inference_checks(paths: SonicModelPaths) -> list[SonicDiagnosticCheck]:
    prepare_sonic_onnx_runtime()
    sessions = {
        profile_name: (
            create_sonic_session(f"{profile_name} encoder", encoder, allow_cpu_shape_ops=False),
            create_sonic_session(f"{profile_name} decoder", decoder, allow_cpu_shape_ops=False),
        )
        for profile_name, (encoder, decoder) in paths.profiles.items()
    }
    encoder, decoder = sessions["sonic-v1.1"]

    encoder_input = np.zeros((1, 1751), dtype=np.float32)
    decoder_input = np.zeros((1, 994), dtype=np.float32)
    planner_inputs = _planner_inputs()
    encoder_output = encoder.run(None, {encoder.get_inputs()[0].name: encoder_input})[0]
    decoder_output = decoder.run(None, {decoder.get_inputs()[0].name: decoder_input})[0]

    with tempfile.TemporaryDirectory(prefix="dimos-sonic-doctor-") as profile_dir:
        profile_options = cast("Any", ort).SessionOptions()
        profile_options.enable_profiling = True
        profile_options.profile_file_prefix = str(Path(profile_dir) / "profile")
        planner = create_sonic_session(
            "planner",
            paths.planner,
            allow_cpu_shape_ops=True,
            session_options=profile_options,
        )
        planner_output = planner.run(None, planner_inputs)
        profile_path = Path(cast("Any", planner).end_profiling())
        cpu_ops = _profile_cpu_ops(profile_path)

    checks: list[SonicDiagnosticCheck] = []
    for profile_name, (profile_encoder, profile_decoder) in sessions.items():
        profile = SONIC_MODEL_PROFILES[cast("Any", profile_name)]
        profile_encoder_input = np.zeros((1, profile.encoder_obs_dim), dtype=np.float32)
        profile_decoder_input = np.zeros((1, 994), dtype=np.float32)
        outputs = (
            profile_encoder.run(
                None, {profile_encoder.get_inputs()[0].name: profile_encoder_input}
            )[0],
            profile_decoder.run(
                None, {profile_decoder.get_inputs()[0].name: profile_decoder_input}
            )[0],
        )
        finite = all(np.all(np.isfinite(output)) for output in outputs)
        checks.append(
            SonicDiagnosticCheck(
                f"{profile_name} inference",
                finite,
                f"encoder input={profile.encoder_obs_dim}, finite outputs={finite}",
            )
        )
    unexpected_ops = set(cpu_ops) - ALLOWED_PLANNER_CPU_OPS
    cpu_events = sum(cpu_ops.values())
    if unexpected_ops or cpu_events > MAX_PLANNER_CPU_EVENTS:
        checks.append(
            SonicDiagnosticCheck(
                "planner CUDA partition",
                False,
                f"CPU ops={dict(cpu_ops)}, allowed={sorted(ALLOWED_PLANNER_CPU_OPS)}, "
                f"maximum events={MAX_PLANNER_CPU_EVENTS}",
            )
        )
    else:
        checks.append(
            SonicDiagnosticCheck(
                "planner CUDA partition",
                True,
                f"CUDA-first with {cpu_events} audited CPU shape/index events: {dict(cpu_ops)}",
            )
        )

    reference = _reference_outputs()
    for name, actual, expected in (
        ("encoder", encoder_output, reference["encoder"]),
        ("decoder", decoder_output, reference["decoder"]),
        ("planner", planner_output[0], reference["planner_qpos"]),
    ):
        try:
            detail = _accuracy_detail(name, np.asarray(actual), expected)
            checks.append(SonicDiagnosticCheck(f"{name} accuracy", True, detail))
        except RuntimeError as exc:
            checks.append(SonicDiagnosticCheck(f"{name} accuracy", False, str(exc)))

    frames_match = np.array_equal(planner_output[1], reference["planner_frames"])
    checks.append(
        SonicDiagnosticCheck(
            "planner frame count",
            frames_match,
            f"actual={planner_output[1].tolist()}, expected={reference['planner_frames'].tolist()}",
        )
    )

    planner_p95 = _benchmark(lambda: planner.run(None, planner_inputs), samples=10, percentile=95.0)
    for profile_name, (profile_encoder, profile_decoder) in sessions.items():
        profile = SONIC_MODEL_PROFILES[cast("Any", profile_name)]
        profile_encoder_input = np.zeros((1, profile.encoder_obs_dim), dtype=np.float32)
        profile_decoder_input = np.zeros((1, 994), dtype=np.float32)
        policy_p99 = _benchmark(
            partial(
                _run_policy_pair,
                profile_encoder,
                profile_decoder,
                profile_encoder_input,
                profile_decoder_input,
            ),
            samples=20,
            percentile=99.0,
        )
        checks.append(
            SonicDiagnosticCheck(
                f"{profile_name} policy latency",
                policy_p99 <= MAX_POLICY_P99_MS,
                f"p99={policy_p99:.2f} ms, limit={MAX_POLICY_P99_MS:.2f} ms",
            )
        )
    checks.append(
        SonicDiagnosticCheck(
            "planner latency",
            planner_p95 <= MAX_PLANNER_P95_MS,
            f"p95={planner_p95:.2f} ms, limit={MAX_PLANNER_P95_MS:.2f} ms",
        )
    )
    return checks


def run_sonic_doctor(
    model_paths: SonicModelPaths | None = None,
) -> SonicDiagnosticReport:
    """Run all non-control SONIC deployment gates; never contacts the robot."""
    results = _run_checks(_host_checks())
    if not all(check.passed for check in results):
        return SonicDiagnosticReport(tuple(results))

    paths = model_paths if model_paths is not None else resolve_sonic_model_paths()
    model_results = _run_checks(_model_checks(paths))
    results.extend(model_results)
    if not all(check.passed for check in model_results):
        return SonicDiagnosticReport(tuple(results))

    try:
        results.extend(_inference_checks(paths))
    except Exception as exc:
        results.append(SonicDiagnosticCheck("SONIC inference", False, str(exc)))
    return SonicDiagnosticReport(tuple(results))
