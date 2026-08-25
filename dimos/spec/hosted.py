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
"""Usage sketch for the hosted API.

The APIs and some names below do not exist yet. This file only shows how we
expect hosted placement to be used.
"""

# Keep the robot stack local and run marker detection on one GPU Host.
unitree_go2_markers = autoconnect(
    unitree_go2,
    MarkerDetectionStreamModule.blueprint(
        marker_length_m=0.1,
        camera_info=GO2Connection.camera_info_static,
    ).placement(tags={"gpu"}),
)

# Select one G1 Host and one GPU Host; navigation remains local.
multi_host_g1 = autoconnect(
    unitree_g1.blueprint().placement(tags={"g1"}),
    navigation.blueprint(),
    expensive.blueprint().placement(tags={"gpu"}),
)

# Pin the robot fragment to one exact Host name or ID.
pinned_g1 = autoconnect(
    unitree_g1.blueprint().placement(host="g1-01"),
    navigation.blueprint(),
    expensive.blueprint().placement(tags={"gpu"}),
)

# Select one matching embodiment; this does not create replicas.
available_g1 = unitree_g1.blueprint().placement(tags={"g1"})
