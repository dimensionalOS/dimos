"""Cloudflare Realtime SFU API client."""

import asyncio
import logging

import httpx

from config import settings

log = logging.getLogger(__name__)

# CF Realtime sessions report `pc.connectionState === 'connected'` to the
# client a few hundred ms before the server-side session is considered ready
# for /datachannels/new. Retry on that specific error before giving up.
_ADD_DC_RETRY_DELAYS_SEC = (0.25, 0.5, 1.0, 2.0)


def _is_session_not_ready(error_code: str, error_description: str) -> bool:
    return (
        error_code == "session_error"
        and "not ready" in (error_description or "").lower()
    )


class CloudflareRealtimeError(Exception):
    def __init__(self, status_code: int, detail: str):
        self.status_code = status_code
        self.detail = detail
        super().__init__(f"CF Realtime API error ({status_code}): {detail}")


class CloudflareRealtime:
    """Thin client for Cloudflare Realtime SFU REST API."""

    def __init__(self) -> None:
        self.base_url = settings.cf_api_url
        self.headers = {
            "Authorization": f"Bearer {settings.cf_teleop_app_secret}",
            "Content-Type": "application/json",
        }

    async def generate_ice_servers(self, ttl: int = 7200) -> list[dict]:
        """Mint short-lived TURN credentials from the CF TURN service.

        Returns RTCPeerConnection-shaped iceServers dicts (urls + username +
        credential), including turns:...:443?transport=tcp so UDP-blocked
        clients can relay over TLS. Caller handles fallback to STUN-only.
        """
        url = (
            f"{settings.cf_turn_base_url}/keys/"
            f"{settings.cf_turn_key_id}/credentials/generate-ice-servers"
        )
        headers = {
            "Authorization": f"Bearer {settings.cf_turn_api_token}",
            "Content-Type": "application/json",
        }
        async with httpx.AsyncClient() as client:
            resp = await client.post(url, headers=headers, json={"ttl": ttl}, timeout=10.0)
        if resp.status_code not in (200, 201):
            raise CloudflareRealtimeError(resp.status_code, resp.text)
        servers = resp.json().get("iceServers")
        # The endpoint returns a list; the older /credentials/generate shape
        # is a single object — normalize both.
        if isinstance(servers, dict):
            servers = [servers]
        if not servers:
            raise CloudflareRealtimeError(resp.status_code, "no iceServers in response")
        return servers

    async def create_session(self, sdp_offer: str) -> dict:
        """Create a new CF session from the offer. Returns sessionId + answer.

        Tracks are NOT declared here — CF ignores a `tracks` array on
        /sessions/new; publish/subscribe is done via /tracks/new (add_tracks)
        once the PC is connected.
        """
        body: dict = {"sessionDescription": {"type": "offer", "sdp": sdp_offer}}
        async with httpx.AsyncClient() as client:
            resp = await client.post(
                f"{self.base_url}/sessions/new",
                headers=self.headers,
                json=body,
                timeout=10.0,
            )
        if resp.status_code != 201:
            raise CloudflareRealtimeError(resp.status_code, resp.text)
        data = resp.json()
        return {
            "cf_session_id": data["sessionId"],
            "sdp_answer": data["sessionDescription"]["sdp"],
        }

    async def add_tracks(self, session_id: str, tracks: list[dict]) -> dict:
        """Pull/push tracks onto an existing connected session via /tracks/new.

        Used for the operator's video subscribe AFTER its PC is connected and
        datachannels are bridged. A remote (pulled) track makes CF set
        `requiresImmediateRenegotiation: true` in the response — the caller
        must drive setRemoteDescription(offer)/answer on the operator PC and
        POST the answer back via renegotiate(). Returns the raw CF response
        (sessionDescription + requiresImmediateRenegotiation)."""
        url = f"{self.base_url}/sessions/{session_id}/tracks/new"
        async with httpx.AsyncClient() as client:
            resp = await client.post(
                url,
                headers=self.headers,
                json={"tracks": tracks},
                timeout=30.0,
            )
        if resp.status_code not in (200, 201):
            raise CloudflareRealtimeError(resp.status_code, resp.text)
        data = resp.json()
        if data.get("errorCode"):
            raise CloudflareRealtimeError(
                resp.status_code,
                f"{data['errorCode']}: {data.get('errorDescription', '')}",
            )
        return data

    async def renegotiate(self, session_id: str, sdp_answer: str) -> None:
        """Submit the operator's SDP answer after a pull set
        requiresImmediateRenegotiation. PUT /sessions/{id}/renegotiate."""
        url = f"{self.base_url}/sessions/{session_id}/renegotiate"
        async with httpx.AsyncClient() as client:
            resp = await client.put(
                url,
                headers=self.headers,
                json={"sessionDescription": {"type": "answer", "sdp": sdp_answer}},
                timeout=30.0,
            )
        if resp.status_code not in (200, 201):
            raise CloudflareRealtimeError(resp.status_code, resp.text)
        data = resp.json()
        if data.get("errorCode"):
            raise CloudflareRealtimeError(
                resp.status_code,
                f"{data['errorCode']}: {data.get('errorDescription', '')}",
            )

    async def add_datachannels(self, session_id: str, channels: list[dict]) -> list[dict]:
        url = f"{self.base_url}/sessions/{session_id}/datachannels/new"
        attempts = 1 + len(_ADD_DC_RETRY_DELAYS_SEC)
        last_err: CloudflareRealtimeError | None = None

        for attempt in range(1, attempts + 1):
            try:
                async with httpx.AsyncClient() as client:
                    resp = await client.post(
                        url,
                        headers=self.headers,
                        json={"dataChannels": channels},
                        timeout=30.0,
                    )
            except httpx.HTTPError as e:
                log.error("CF add_datachannels %s failed: %r", type(e).__name__, e)
                raise
            if resp.status_code not in (200, 201):
                raise CloudflareRealtimeError(resp.status_code, resp.text)
            data = resp.json()
            error_code = data.get("errorCode")
            if not error_code:
                return data.get("dataChannels", [])

            last_err = CloudflareRealtimeError(
                resp.status_code,
                f"{error_code}: {data.get('errorDescription', '')}",
            )
            if not _is_session_not_ready(error_code, data.get("errorDescription", "")):
                raise last_err
            if attempt >= attempts:
                break
            delay = _ADD_DC_RETRY_DELAYS_SEC[attempt - 1]
            log.warning(
                "CF add_datachannels attempt=%d session-not-ready, retrying in %.2fs",
                attempt, delay,
            )
            await asyncio.sleep(delay)

        assert last_err is not None
        raise last_err

    async def close_datachannels(self, session_id: str, ids: list[int]) -> None:
        """Close datachannels by their numeric id (PUT /datachannels/close).

        CF keeps a local datachannel push registered until explicitly closed —
        it is NOT auto-reaped when the subscriber leaves (the 30s GC is
        media-only). So the robot's reverse push (state_reliable_back) must be
        closed on operator disconnect, else re-push errors repeated_local_track.
        Best-effort: log and swallow failures (the channel may already be gone).
        """
        url = f"{self.base_url}/sessions/{session_id}/datachannels/close"
        try:
            async with httpx.AsyncClient() as client:
                resp = await client.put(
                    url,
                    headers=self.headers,
                    json={"dataChannels": [{"id": i} for i in ids]},
                    timeout=10.0,
                )
            if resp.status_code not in (200, 201):
                log.warning("CF close_datachannels %s: %s", resp.status_code, resp.text[:200])
        except Exception as e:
            log.warning("CF close_datachannels failed: %r", e)


cf_client = CloudflareRealtime()
