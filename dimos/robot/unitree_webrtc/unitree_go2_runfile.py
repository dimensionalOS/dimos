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

from dimos.protocol import pubsub

if __name__ == "__main__":
    # Imported here so dask doesn't import it multiple times.
    from dimos.robot.unitree_webrtc.unitree_go2_blueprints import standard as unitree_go2

    pubsub.lcm.autoconf()
    unitree_go2.build().loop()