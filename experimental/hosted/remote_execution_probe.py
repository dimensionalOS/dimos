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

"""Small module used by the real Host remote-execution probe."""

from __future__ import annotations

import os
import socket

from dimos.core.core import rpc
from dimos.core.module import Module


class RemoteExecutionProbe(Module):
    @rpc
    def execute(self, nonce: str, value: int) -> dict[str, str | int]:
        """Run deterministic work and identify the process that ran it."""
        return {
            "nonce": nonce,
            "input": value,
            "result": value * value + 1,
            "hostname": socket.gethostname(),
            "pid": os.getpid(),
        }

    @rpc
    def fail(self, message: str) -> None:
        """Raise a known error so the controller can verify error propagation."""
        raise RuntimeError(f"remote probe failure: {message}")
