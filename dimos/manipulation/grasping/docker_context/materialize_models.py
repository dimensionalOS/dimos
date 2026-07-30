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

"""Materialize pinned GraspGen artifacts during the container build."""

from pathlib import Path

from model_artifacts import (  # type: ignore[import-not-found]
    GRASPGEN_ARTIFACTS,
    materialize_model_artifacts,
)


def materialize_models(destination: Path) -> None:
    """Download, verify, and copy GraspGen model files into the image."""
    materialize_model_artifacts(destination, GRASPGEN_ARTIFACTS.values())


if __name__ == "__main__":
    materialize_models(Path("/app/GraspGen"))
