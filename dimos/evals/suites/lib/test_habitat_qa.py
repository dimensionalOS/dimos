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

from dimos.evals.suites.lib.habitat_qa import HSSD_DATASET, environment


def test_shared_dataset_override_and_fresh_environment(monkeypatch, tmp_path):
    dataset = str(tmp_path / "hssd-hab.scene_dataset_config.json")
    monkeypatch.setenv("HSSD_DATASET_CONFIG", dataset)
    first = environment("test", "HSSD_DATASET_CONFIG", HSSD_DATASET)
    second = environment("test", "HSSD_DATASET_CONFIG", HSSD_DATASET)
    assert first is not second
    assert first.config.scene_dataset_config == dataset


def test_environment_resolves_dataset_and_asset_overrides(monkeypatch, tmp_path):
    dataset = str(tmp_path / "dataset.json")
    asset = str(tmp_path / "apartment_1.glb")
    monkeypatch.setenv("HABITAT_TEST_DATASET_CONFIG", dataset)
    monkeypatch.setenv("HABITAT_TEST_SCENE", asset)
    env = environment(
        "original.glb",
        "HABITAT_TEST_DATASET_CONFIG",
        "default",
        scene_env="HABITAT_TEST_SCENE",
    )
    assert env.config.scene_dataset_config == dataset
    assert env.config.scene_id == asset
