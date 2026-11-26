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

from dimos_lcm.control_msgs import DynamicJointState as LCMDynamicJointState


class DynamicJointState(LCMDynamicJointState):  # type: ignore[misc]
    """``control_msgs.DynamicJointState`` with its header stamp as ``ts``."""

    @property
    def ts(self) -> float:
        """The header stamp, under the name recorders read."""
        stamp = self.header.stamp
        return stamp.sec + stamp.nsec / 1_000_000_000  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> DynamicJointState:
        # The generated decoder resolves nested field types from the decoding class's
        # module: decode as the LCM type and copy its fields.
        decoded = LCMDynamicJointState.lcm_decode(data)
        return cls(**{name: getattr(decoded, name) for name in LCMDynamicJointState.__slots__})
