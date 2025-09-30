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

from typing import Protocol

from dimos.mapping.spec import Global3DMapSpec
from dimos.navigation.rosnav import ROSNav
from dimos.navigation.spec import NavSpec
from dimos.perception.spec import PointcloudPerception


class RosNavSpec(NavSpec, PointcloudPerception, Global3DMapSpec, Protocol):
    """Combined protocol for navigation components."""

    pass


def accepts_combined_protocol(nav: RosNavSpec) -> None:
    """Function that accepts all navigation protocols at once."""
    pass


def test_typing_prototypes():
    """Test that ROSNav correctly implements all required protocols."""
    rosnav = ROSNav()
    accepts_combined_protocol(rosnav)
