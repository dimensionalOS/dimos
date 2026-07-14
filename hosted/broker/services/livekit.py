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

"""LiveKit backend: room-scoped JWT minting.

LiveKit is its own SFU, so unlike the Cloudflare path the broker does no SDP
relay or datachannel bridging — it just mints access tokens and names the room
the robot and operator both join. The API key/secret stay server-side; only the
signed JWT (and the public server URL) reach clients.
"""

from datetime import timedelta
import logging

from config import settings
from livekit import api

log = logging.getLogger(__name__)


class LiveKitError(Exception):
    """LiveKit backend unavailable or misconfigured."""


def _require_configured() -> None:
    if not settings.livekit_configured:
        raise LiveKitError(
            "LiveKit not configured (set LIVEKIT_URL / LIVEKIT_API_KEY / LIVEKIT_API_SECRET)"
        )


def room_name(session_id: str) -> str:
    """Per-session room: a fresh robot connect = a fresh room, no stale state."""
    return f"sess-{session_id}"


def mint_token(*, identity: str, name: str, room: str, can_publish: bool) -> str:
    """Mint a room-scoped JWT.

    Identity must be unique per participant within a room — LiveKit evicts an
    earlier participant that reconnects with the same identity. ``can_publish``
    gates media uplink (the robot publishes its camera track; the operator is
    data-only); both sides may always publish data and subscribe.
    """
    _require_configured()
    grants = api.VideoGrants(
        room_join=True,
        room=room,
        can_publish=can_publish,
        can_subscribe=True,
        can_publish_data=True,
    )
    return (
        api.AccessToken(settings.livekit_api_key, settings.livekit_api_secret)
        .with_identity(identity)
        .with_name(name)
        .with_grants(grants)
        .with_ttl(timedelta(seconds=settings.livekit_token_ttl_sec))
        .to_jwt()
    )
