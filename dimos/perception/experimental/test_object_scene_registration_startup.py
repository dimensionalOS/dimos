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

import os
from pathlib import Path
import subprocess
import sys
import textwrap


def test_detector_dependencies_are_loaded_before_threaded_startup():
    # A fresh interpreter is essential: other tests may have already imported
    # Transformers. No model is constructed and no weights are downloaded.
    probe = textwrap.dedent("""
        from concurrent.futures import ThreadPoolExecutor
        import importlib
        import sys

        from dimos.perception.experimental.object_scene_registration import (
            ObjectSceneRegistrationModule,
        )

        assert "dimos.models.vl.moondream" in sys.modules, (
            "Moondream dependencies must load before threaded module startup"
        )
        assert "transformers.models.auto.modeling_auto" in sys.modules, (
            "AutoModelForCausalLM must be resolved before threaded module startup"
        )

        # These imports overlap during module startup. Once module loading has
        # resolved their dependencies, neither needs a cold import in an RPC thread.
        with ThreadPoolExecutor(max_workers=2) as pool:
            detector, metrics = list(pool.map(importlib.import_module, (
                "dimos.models.vl.moondream", "sklearn.metrics.pairwise",
            )))
        assert detector.MoondreamVlModel._model_class.__name__ == "AutoModelForCausalLM"
        assert callable(metrics.pairwise_distances)
    """)
    result = subprocess.run(
        [sys.executable, "-c", probe],
        cwd=Path(__file__).resolve().parents[3],
        env={**os.environ, "HF_HUB_OFFLINE": "1"},
        capture_output=True,
        text=True,
        timeout=60,
    )

    assert result.returncode == 0, result.stdout + result.stderr
