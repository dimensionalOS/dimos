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

"""Convert successful R1Pro NPZ demonstrations into a local LeRobot dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from lerobot.datasets.lerobot_dataset import LeRobotDataset
import numpy as np
from numpy.typing import NDArray

from dimos.robot.galaxea.r1pro.learning import (
    R1PRO_PICK_PLACE_FPS,
    R1PRO_PICK_PLACE_IMAGE_SIZE,
    R1PRO_PICK_PLACE_JOINTS,
    R1PRO_PICK_PLACE_TASK,
)


def stable_joint_statistics(
    states: NDArray[Any],
    actions: NDArray[Any],
) -> dict[str, dict[str, list[Any]]]:
    """Use float64 accumulation and unit scales for never-commanded joints.

    Float32 reduction can put a constant feature's mean outside its min/max,
    while returning zero standard deviation. Dividing that rounding error by
    epsilon creates large training targets. Passive joint vibration must also
    not become an artificial episode clock after division by a tiny variance.
    """
    state = np.asarray(states, dtype=np.float64)
    action = np.asarray(actions, dtype=np.float64)
    fixed = np.ptp(action, axis=0) < 1e-6
    result = {}
    for key, values in (("observation.state", state), ("action", action)):
        deviation = values.std(axis=0)
        deviation[fixed] = 1.0
        deviation = np.maximum(deviation, 1e-4)
        result[key] = {
            "mean": values.mean(axis=0).tolist(),
            "std": deviation.tolist(),
            "min": values.min(axis=0).tolist(),
            "max": values.max(axis=0).tolist(),
            "count": [len(values)],
        }
    return result


def update_joint_statistics(source: Path, output: Path) -> None:
    """Retain image statistics and replace the numerically unstable joint stats."""
    manifest = json.loads((source / "manifest.json").read_text())
    states, actions = [], []
    for episode in manifest["episodes"]:
        with np.load(source / episode["file"], allow_pickle=False) as data:
            states.append(data["observation.state"])
            actions.append(data["action"])
    stats_path = output / "meta" / "stats.json"
    stats = json.loads(stats_path.read_text())
    stats.update(stable_joint_statistics(np.concatenate(states), np.concatenate(actions)))
    stats_path.write_text(json.dumps(stats, indent=2) + "\n")
    (output / "normalization.json").write_text(
        json.dumps(
            {
                "method": "float64 mean/std; unit scale for joints with constant demonstration commands",
                "constant_command_joints": [
                    name
                    for name, lo, hi in zip(
                        R1PRO_PICK_PLACE_JOINTS,
                        stats["action"]["min"],
                        stats["action"]["max"],
                        strict=True,
                    )
                    if hi - lo < 1e-6
                ],
            },
            indent=2,
        )
        + "\n"
    )


def convert(source: Path, output: Path) -> None:
    manifest = json.loads((source / "manifest.json").read_text())
    if (
        manifest["joints"] != list(R1PRO_PICK_PLACE_JOINTS)
        or manifest["fps"] != R1PRO_PICK_PLACE_FPS
    ):
        raise ValueError("Demonstration contract does not match the R1Pro policy")
    width = len(R1PRO_PICK_PLACE_JOINTS)
    size = R1PRO_PICK_PLACE_IMAGE_SIZE
    features: dict[str, Any] = {
        key: {"dtype": "float32", "shape": (width,), "names": list(R1PRO_PICK_PLACE_JOINTS)}
        for key in ("observation.state", "action")
    }
    for camera in ("head", "right_wrist"):
        features[f"observation.images.{camera}"] = {
            "dtype": "image",
            "shape": (size, size, 3),
            "names": ["height", "width", "channels"],
        }
    dataset = LeRobotDataset.create(
        repo_id="local/r1pro-pick-place",
        root=output,
        fps=R1PRO_PICK_PLACE_FPS,
        robot_type="r1pro_sim_pick_place",
        features=features,
        use_videos=False,
        image_writer_threads=4,
    )
    try:
        for episode in manifest["episodes"]:
            if not episode["success"]:
                raise ValueError("Refusing a failed demonstration")
            with np.load(source / episode["file"], allow_pickle=False) as data:
                arrays = {key: data[key] for key in features}
                for index in range(episode["frames"]):
                    dataset.add_frame(
                        {
                            **{key: arrays[key][index] for key in features},
                            "task": R1PRO_PICK_PLACE_TASK,
                        }
                    )
                dataset.save_episode()
            print(f"Converted seed {episode['seed']}", flush=True)
    finally:
        dataset.finalize()  # type: ignore[no-untyped-call]
    update_joint_statistics(source, output)
    (output / "collection_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    convert(args.source, args.output)


if __name__ == "__main__":
    main()
