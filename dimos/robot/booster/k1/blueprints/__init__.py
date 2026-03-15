#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Cascaded Booster K1 blueprints split into focused modules."""

import lazy_loader as lazy

__getattr__, __dir__, __all__ = lazy.attach(
    __name__,
    submod_attrs={
        "agentic._common_agentic": ["_common_agentic"],
        "agentic.booster_k1_agentic": ["booster_k1_agentic"],
        "basic.booster_k1_basic": ["booster_k1_basic"],
        "smart.booster_k1_spatial": ["booster_k1_spatial"],
    },
)
