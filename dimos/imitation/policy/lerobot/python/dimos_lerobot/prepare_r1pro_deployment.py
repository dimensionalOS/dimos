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

"""Create an immutable local deployment copy with a tested ACT chunk setting."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil

from dimos_lerobot.runtime import _validate_features
from lerobot.configs.policies import PreTrainedConfig

from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_IO


def prepare(source: Path, output: Path, action_steps: int = 30) -> None:
    """Preserve trained weights and record the inference-only configuration change."""
    config = PreTrainedConfig.from_pretrained(source)
    _validate_features(config, R1PRO_PICK_PLACE_IO)
    raw = json.loads((source / "config.json").read_text())
    if raw.get("type") != "act" or not 1 <= action_steps <= raw["chunk_size"]:
        raise ValueError("Expected an ACT checkpoint and execution length within its chunk")
    if (source / "DIAGNOSTIC_ONLY.txt").exists():
        raise ValueError("A diagnostic checkpoint cannot become a task policy")
    with (source / "model.safetensors").open("rb") as weights:
        checksum = hashlib.file_digest(weights, "sha256").hexdigest()
    shutil.copytree(source, output)
    raw["n_action_steps"] = action_steps
    (output / "config.json").write_text(json.dumps(raw, indent=2) + "\n")
    (output / "deployment.json").write_text(
        json.dumps(
            {
                "source_checkpoint": str(source.resolve()),
                "weights_modified": False,
                "weights_sha256": checksum,
                "override": {"n_action_steps": action_steps},
                "profile": R1PRO_PICK_PLACE_IO.name,
            },
            indent=2,
        )
        + "\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--action-steps", type=int, default=30)
    args = parser.parse_args()
    prepare(args.source, args.output, args.action_steps)


if __name__ == "__main__":
    main()
