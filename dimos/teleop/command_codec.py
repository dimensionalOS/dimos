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


def validate_cdr_payload(data: bytes) -> None:
    """Require plain XCDR1 at the WebXR command input boundary."""
    if len(data) < 4 or data[:2] not in (b"\x00\x00", b"\x00\x01"):
        raise ValueError("Expected standalone XCDR1 encapsulation")
