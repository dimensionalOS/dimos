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

from __future__ import annotations

import attrs


@attrs.frozen
class DamiaoRuntimeConfig:
    """Deployment values that may vary without changing robot topology."""

    bus_addresses: dict[str, str] = attrs.field(
        factory=dict,
        validator=attrs.validators.deep_mapping(
            key_validator=attrs.validators.and_(
                attrs.validators.instance_of(str),
                attrs.validators.min_len(1),
            ),
            value_validator=attrs.validators.and_(
                attrs.validators.instance_of(str),
                attrs.validators.min_len(1),
            ),
        ),
    )
    gravity_comp: bool = attrs.field(
        default=True,
        validator=attrs.validators.instance_of(bool),
    )
    tick_deadline_us: int = attrs.field(
        default=1_000,
        validator=attrs.validators.and_(
            attrs.validators.instance_of(int),
            attrs.validators.ge(1),
        ),
    )
