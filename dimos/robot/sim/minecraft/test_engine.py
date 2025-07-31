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

import time

from dimos.robot.sim import minecraft


def test_full_engine():
    """Test the full Minecraft engine functionality."""
    engine = minecraft.Engine(frequency=1)  # 1 hz
    stream = engine.get_stream()

    def show_obs(output):
        print(minecraft.Output(output))

    stream.subscribe(show_obs)
    time.sleep(3)
    stream.dispose()
