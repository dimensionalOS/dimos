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

import sys

from pytest_mock import MockerFixture

from dimos.core.o3dpickle import register_picklers


def test_register_picklers_is_a_noop_without_open3d(mocker: MockerFixture) -> None:
    mocker.patch.dict(sys.modules, {"open3d": None})

    register_picklers()
