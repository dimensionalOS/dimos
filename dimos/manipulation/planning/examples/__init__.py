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

"""
Manipulation Client - RPC client for ManipulationModule

Usage:
    # Start coordinator and planner first:
    dimos run coordinator-mock
    dimos run xarm7-planner-coordinator

    # Then run the interactive client:
    python -m dimos.manipulation.planning.examples.manipulation_client

    # IPython shell with client pre-loaded:
    c.joints()           # Get current joint positions
    c.plan([0.1, ...])   # Plan to joints
    c.preview()          # Preview in Meshcat
    c.execute()          # Execute via coordinator
"""

from dimos.manipulation.planning.examples.manipulation_client import ManipulationClient

__all__ = ["ManipulationClient"]
