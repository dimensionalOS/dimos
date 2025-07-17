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

from datetime import datetime

from dimos.types.timestamped import Timestamped


def test_timestamped_dt_method():
    ts = 1751075203.4120464
    timestamped = Timestamped(ts)
    dt = timestamped.dt()
    assert isinstance(dt, datetime)
    assert abs(dt.timestamp() - ts) < 1e-6
    assert dt.tzinfo is not None, "datetime should be timezone-aware"
