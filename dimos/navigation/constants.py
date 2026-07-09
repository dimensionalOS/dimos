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

"""Shared navigation constants."""

# Max staleness in seconds for tf lookups against a live odometry stamp. Must
# comfortably exceed the coarsest static-transform publish period so leading-edge
# phase or modest clock skew does not silently drop frames.
TF_LOOKUP_TOLERANCE_S = 0.25
