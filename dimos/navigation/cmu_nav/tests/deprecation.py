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

"""
Check if we should skip the cmu nav tests
"""

import os

import pytest

SKIP_CMU_NAV = pytest.mark.skipif(
    os.environ.get("CMU_NAV_TESTS") != "true",
    reason="cmu_nav tests disabled, set CMU_NAV_TESTS=true to run",
)
