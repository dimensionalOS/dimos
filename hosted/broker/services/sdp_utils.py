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

"""SDP introspection helpers — minimal regex-based extraction.

Not a full SDP parser. Only covers what the broker needs: finding the first
publishable m=video section in a peer's offer so we can register it with CF
Realtime's `/sessions/{id}/tracks/new` endpoint.
"""

from __future__ import annotations

import re


def _extract_track(sdp: str, media: str) -> tuple[str, str] | None:
    """Return (mid, track_name) for the first sendonly/sendrecv ``m=<media>``.

    Returns None if the offer has no such section, if every one is recvonly
    or inactive, or if the matching section lacks `a=mid` or `a=msid`.
    ``track_name`` is the *second* token of ``a=msid:<stream> <track>`` —
    browsers and aiortc emit both. CF Realtime's add_tracks(location="local")
    expects this trackId as ``trackName``.
    """
    for section in re.split(r"(?m)^(?=m=)", sdp):
        if not section.startswith(f"m={media}"):
            continue
        if re.search(r"(?m)^a=(recvonly|inactive)\s*$", section):
            continue
        mid = re.search(r"(?m)^a=mid:(\S+)", section)
        msid = re.search(r"(?m)^a=msid:\S+\s+(\S+)", section)
        if mid and msid:
            return mid.group(1), msid.group(1)
    return None


def extract_video_track(sdp: str) -> tuple[str, str] | None:
    """(mid, track_name) of the first publishable m=video, or None."""
    return _extract_track(sdp, "video")


def extract_audio_track(sdp: str) -> tuple[str, str] | None:
    """(mid, track_name) of the first publishable m=audio, or None.

    Used on the OPERATOR's offer — its mic track, which the robot then pulls.
    """
    return _extract_track(sdp, "audio")
