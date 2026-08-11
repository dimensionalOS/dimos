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

"""Agent-facing task text for the public VLN-CE condition."""


def vlnce_task_prompt(instruction: str) -> str:
    """Explain the verified route instruction and irreversible terminal action."""

    if not instruction:
        raise ValueError("VLN-CE instruction must not be empty")
    return (
        "Follow this route instruction in the current environment:\n\n"
        f"{instruction}\n\n"
        "Use only the public camera, depth geometry, odometry, and traversability map to "
        "understand the scene and navigate. Point-to-point navigation completion does not end "
        "the route. When—and only when—you believe the full instruction is complete, call "
        "submit_route(). That call means VLN-CE STOP, is irreversible, and ends the evaluation. "
        "It acknowledges submission but does not reveal whether the route succeeded."
    )
