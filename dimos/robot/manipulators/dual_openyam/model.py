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

"""Authoritative arm-only Dual OpenYAM model assets."""

from pathlib import Path

from dimos.utils.data import LfsPath

# LFS extraction caches by directory name. Version revised artifacts so they cannot
# silently reuse an obsolete extracted model.
DUAL_OPENYAM_ARTIFACT = "dual_openyam_abc_box_v2"
DUAL_OPENYAM_PACKAGE = LfsPath(DUAL_OPENYAM_ARTIFACT)
DUAL_OPENYAM_MODEL_PATH = DUAL_OPENYAM_PACKAGE / "dual_openyam.urdf"
DUAL_OPENYAM_PACKAGE_PATHS: dict[str, Path] = {
    "dual_openyam_abc_box": DUAL_OPENYAM_PACKAGE,
}
DUAL_OPENYAM_SOURCE_PATH = DUAL_OPENYAM_PACKAGE / "SOURCE.md"
DUAL_OPENYAM_ABC_LICENSE_PATH = DUAL_OPENYAM_PACKAGE / "ABC_LICENSE"
DUAL_OPENYAM_I2RT_LICENSE_PATH = DUAL_OPENYAM_PACKAGE / "I2RT_YAM_LICENSE"
