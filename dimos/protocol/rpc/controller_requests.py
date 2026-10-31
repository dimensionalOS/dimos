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

"""Read the pickled RPC requests dim-controller's Rust server still sends.

Integration branch only: main dropped pickle on purpose, and this goes when the
Controller speaks CBOR. A plain ``pickle.loads`` on a network payload runs whatever
the payload names, so this reader resolves nothing but the four message classes
the Controller's ``pickle.rs`` writes. Plain values need no class at all, so
anything else in a payload is refused before it can run.
"""

from __future__ import annotations

import io
import pickle
from typing import Any

# Every pickle the Controller writes starts with this protocol byte, which in CBOR is
# an empty array, never a call.
PICKLE_PROTOCOL_BYTE = b"\x80"

_CONTROLLER_MESSAGES = frozenset(
    {
        ("dimos.msgs.sensor_msgs.JointState", "JointState"),
        ("dimos.msgs.geometry_msgs.PoseStamped", "PoseStamped"),
        ("dimos.msgs.geometry_msgs.Vector3", "Vector3"),
        ("dimos.msgs.geometry_msgs.Quaternion", "Quaternion"),
    }
)


class _ControllerUnpickler(pickle.Unpickler):
    def find_class(self, module: str, name: str) -> Any:
        if (module, name) not in _CONTROLLER_MESSAGES:
            raise pickle.UnpicklingError(f"refused {module}.{name} in a pickled RPC request")
        return super().find_class(module, name)


def is_controller_request(payload: bytes) -> bool:
    return payload[:1] == PICKLE_PROTOCOL_BYTE


def read_controller_request(payload: bytes) -> Any:
    """The request's ``(args, kwargs)``; raises ``pickle.UnpicklingError`` on anything else."""
    return _ControllerUnpickler(io.BytesIO(payload)).load()
