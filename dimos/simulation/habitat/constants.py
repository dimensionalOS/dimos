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

"""Public paths for the Habitat native build and downloaded scene datasets."""

from pathlib import Path
from typing import Final

from dimos.constants import DIMOS_PROJECT_ROOT

HABITAT_ROOT: Final[Path] = DIMOS_PROJECT_ROOT / "target" / "habitat"
HABITAT_DATA_DIR: Final[Path] = HABITAT_ROOT / "data"

# Installed by nix/install.sh with the Habitat downloader's hm3d_example UID.
HM3D_EXAMPLE_DATASET_CONFIG: Final[Path] = (
    HABITAT_DATA_DIR
    / "versioned_data/hm3d-0.2/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json"
)

# The Habitat downloader's hssd-hab UID adds scene_datasets/ below its --data-path.
HSSD_DATASET_CONFIG: Final[Path] = (
    HABITAT_DATA_DIR / "scene_datasets/hssd-hab/hssd-hab.scene_dataset_config.json"
)
