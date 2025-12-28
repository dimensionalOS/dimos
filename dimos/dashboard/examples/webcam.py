# Copyright 2025 Dimensional Inc.
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

from dimos.core.blueprints import autoconnect
from dimos.dashboard.module import Dashboard
from dimos.hardware.camera.module import CameraModule

if __name__ == "__main__":
    blueprint = (
        autoconnect(
            CameraModule.blueprint(),
            Dashboard.blueprint(
                open_rerun=True,
            ),
        )
        .global_config(n_dask_workers=3)
    )
    coordinator = blueprint.build()
    coordinator.loop()
