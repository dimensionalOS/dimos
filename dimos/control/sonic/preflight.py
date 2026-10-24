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

"""No-motor SONIC startup and repeated planner inference check; not a hardware rollout."""

from pathlib import Path
from typing import Any, cast

import numpy as np
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.control.sonic.models import setup_models, sonic_model_directory
from dimos.control.sonic.sonic_onnx_runtime import create_sonic_session
from dimos.control.sonic.sonic_pipeline import SonicPipeline


def verify_planner(pipeline: SonicPipeline, planner_path: Path) -> float:
    """Compare 30 CUDA calls with an unoptimized CPU reference on three inputs.

    Repetition matters: the first call passed on the faulty ORT 1.18.1 setup.
    Both providers share ORT, so this detects regressions, not model correctness.
    The 1e-3 absolute tolerance is above the measured 4.7e-5 maximum discrepancy
    on the corrected JP5 installation and below the reproduced corruption.
    """
    options = cast("Any", ort).SessionOptions()
    options.graph_optimization_level = cast("Any", ort).GraphOptimizationLevel.ORT_DISABLE_ALL
    options.enable_mem_reuse = False
    reference = create_sonic_session(
        "planner",
        planner_path,
        allow_cpu_shape_ops=True,
        session_options=options,
        cpu_simulation=True,
    )
    cases = []
    for velocity in ((0.0, 0.0, 0.0), (0.3, 0.0, 0.0), (0.3, 0.1, 0.3)):
        pipeline.set_velocity(*velocity)
        inputs = pipeline._build_planner_inputs()
        expected, count = pipeline._validate_planner_result(reference.run(None, inputs))
        cases.append((inputs, expected[:count].copy(), count))

    max_error = 0.0
    for _ in range(10):
        for inputs, expected, expected_count in cases:
            actual, count = pipeline._validate_planner_result(pipeline._planner.run(None, inputs))
            if count != expected_count:
                raise RuntimeError(
                    f"SONIC planner frame count differs: {count} != {expected_count}"
                )
            error = float(np.max(np.abs(actual[:count] - expected)))
            if error > 1e-3:
                raise RuntimeError(f"SONIC planner CPU/CUDA discrepancy {error:.6g} exceeds 1e-3")
            max_error = max(max_error, error)
    return max_error


def main() -> None:
    models = sonic_model_directory()
    setup_models(models, check=True, motions=False)
    planner_path = models / "planner_sonic.onnx"
    pipeline = SonicPipeline(
        models / "sonic_v1_1/model_encoder.onnx",
        models / "sonic_v1_1/model_decoder.onnx",
        planner_path,
    )
    try:
        error = verify_planner(pipeline, planner_path)
    finally:
        pipeline.close()
    version = getattr(ort, "__version__", "unknown")
    print(f"PASS  ORT {version}: 30 planner calls; max CPU/CUDA error {error:.6g}")
    print("No robot connection or motor commands. This does not validate physical stability.")


if __name__ == "__main__":
    main()
