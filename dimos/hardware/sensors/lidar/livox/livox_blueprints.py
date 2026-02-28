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

from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.visualization.rerun.bridge import rerun_bridge

mid360 = autoconnect(
    Mid360.blueprint(),
    rerun_bridge(),
).global_config(n_dask_workers=2, robot_model="mid360")
