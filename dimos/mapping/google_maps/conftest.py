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

import json
from pathlib import Path

import pytest

from dimos.mapping.google_maps.google_maps import GoogleMaps

_FIXTURE_DIR = Path(__file__).parent / "fixtures"


@pytest.fixture
def maps_client(mocker):
    ret = GoogleMaps()
    ret._client = mocker.MagicMock()
    return ret


@pytest.fixture
def maps_fixture():
    def open_file(relative: str) -> str:
        with open(_FIXTURE_DIR / relative) as f:
            return json.load(f)

    return open_file
