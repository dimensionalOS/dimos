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

"""Verification suite for the perception/scene MCP skills (SKILLS_WISHLIST).

One test module per wishlist skill. Each drives the *real* skill code and
writes human-reviewable artifacts into ``test_outputs/`` alongside a generated
``REVIEW.md`` that says what a reviewer should see. See README.md.
"""
