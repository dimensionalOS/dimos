# Copyright 2025-2026 Dimensional Inc.
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

"""Offline prerequisite checks. Never load a model, open a camera or contact a hub."""

from __future__ import annotations

import importlib
import importlib.util
import json
import os
from pathlib import Path

from dimos.utils.data import get_data_dir


def head_perception_dependencies() -> str | None:
    packages = ("open3d", "sam2", "transformers", "torch", "viser", "vpi")
    missing = [name for name in packages if importlib.util.find_spec(name) is None]
    errors = []
    if missing:
        errors.append("missing packages: " + ", ".join(missing))
    if "torch" not in missing:
        torch = importlib.import_module("torch")
        if not torch.cuda.is_available():
            errors.append(
                f"CUDA PyTorch required for EdgeTAM; installed torch={torch.__version__}, CUDA={torch.version.cuda}"
            )
    if errors:
        return (
            "Head perception: "
            + "; ".join(errors)
            + ". Run python -m dimos.robot.galaxea.r1pro.head_perception_preflight before starting cameras."
        )
    return None


def head_perception_assets() -> list[str]:
    """Inspect existing default-backend assets without initiating downloads."""
    missing = []
    if not (get_data_dir() / "models_edgetam/edgetam.pt").is_file():
        missing.append("DimOS models_edgetam/edgetam.pt")
    hf_home = Path(os.environ.get("HF_HOME", str(Path.home() / ".cache/huggingface")))
    cache = Path(os.environ.get("HF_HUB_CACHE", str(hf_home / "hub")))
    snapshots = cache / "models--vikhyatk--moondream2/snapshots"
    ready = snapshots.is_dir() and any(
        (p / "config.json").is_file()
        and any(
            f.is_file()
            for pattern in ("*.safetensors", "pytorch_model*.bin")
            for f in p.glob(pattern)
        )
        for p in snapshots.iterdir()
        if p.is_dir()
    )
    if not ready:
        missing.append("Hugging Face vikhyatk/moondream2 cached snapshot and weights")
    return missing


def head_perception_ready() -> str | None:
    """Check the default Moondream/EdgeTAM stack before any module is deployed."""
    errors = []
    dependencies = head_perception_dependencies()
    if dependencies:
        errors.append(dependencies)
    assets = head_perception_assets()
    if assets:
        errors.append("Missing cached model assets: " + ", ".join(assets))
    return "; ".join(errors) if errors else None


def main() -> int:
    dependencies = head_perception_dependencies()
    assets = head_perception_assets()
    print(
        json.dumps(
            {
                "dependencies": dependencies,
                "missing_assets": assets,
                "models_loaded": False,
                "hardware_opened": False,
            },
            indent=2,
        )
    )
    return int(bool(dependencies or assets))


if __name__ == "__main__":
    raise SystemExit(main())
