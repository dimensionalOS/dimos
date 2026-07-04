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

"""SDP-string workarounds for the aiortc + Cloudflare Realtime SFU combo.

Pure functions, easy to unit-test in isolation. The *why* lives in
``dimos/teleop/quest_hosted/README.md`` — these are the workarounds; the
README explains the bugs.
"""

from __future__ import annotations

import re


def propagate_bundle_candidates(sdp: str) -> str:
    """Workaround for aiortc MAX_BUNDLE candidate-dict overwrite (see README)."""
    sections = re.split(r"(?m)^(?=m=)", sdp)
    cand_lines: list[str] = []
    for s in sections:
        if s.startswith("m="):
            found = re.findall(r"(?m)^(a=(?:candidate:|end-of-candidates).*)$", s)
            if found:
                cand_lines = found
                break
    if not cand_lines:
        return sdp

    block = "\r\n".join(cand_lines) + "\r\n"
    out = []
    for s in sections:
        if s.startswith("m=") and not re.search(r"(?m)^a=candidate:", s):
            out.append(s.rstrip("\r\n") + "\r\n" + block)
        else:
            out.append(s)
    return "".join(out)
