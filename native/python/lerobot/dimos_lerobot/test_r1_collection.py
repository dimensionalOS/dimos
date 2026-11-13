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

"""R1 MCAP actions and camera frames are consumable by real LeRobot policies."""

from dimos_lerobot.dataprep import write
from lerobot.configs.types import FeatureType
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import dataset_to_policy_features
import numpy as np

from dimos.imitation.dataprep.build import run_dataprep
from dimos.imitation.dataprep.schema import OutputConfig, RecordingSchema
from dimos.robot.galaxea.r1pro._collection_test_fakes import write_collection
from dimos.robot.galaxea.r1pro.collection import POSITION_JOINTS
from dimos.robot.galaxea.r1pro.config import R1PRO_BODY_AXES


def test_r1_capture_loads_as_policy_training_data(tmp_path):
    directory = write_collection(tmp_path / "recording")
    schema = RecordingSchema.read(directory)
    destination = tmp_path / "dataset"
    config = schema.dataprep_config(directory, OutputConfig(path=destination))
    run_dataprep(config, writer=write)
    dataset = LeRobotDataset(repo_id="local/r1pro-teleop", root=destination, video_backend="pyav")
    assert dataset.num_episodes == 1
    assert dataset.num_frames == 5
    assert dataset.fps == 5
    assert dataset.features["action"]["names"] == [*POSITION_JOINTS, *R1PRO_BODY_AXES]
    features = dataset_to_policy_features(dataset.features)
    assert features["action"].type == FeatureType.ACTION
    assert features["action"].shape == (23,)
    assert features["observation.state"].shape == (20,)
    frame = dataset[0]
    np.testing.assert_allclose(frame["action"].numpy(), [0.5] * 20 + [0.2, -0.1, 0.3])
    assert tuple(frame["observation.images.head"].shape) == (3, 384, 480)
    assert tuple(frame["observation.images.right_wrist"].shape) == (3, 720, 1280)
