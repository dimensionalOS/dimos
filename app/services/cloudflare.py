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

    async def create_session(
        self, sdp_offer: str, tracks: list[dict] | None = None
    ) -> dict:
        """Create a new CF session. Returns sessionId + SDP answer.

        Pass `tracks` to declare publisher (location=local) or subscriber
        (location=remote) tracks during the initial offer/answer. CF binds
        them as part of session setup, so the PC connects with media properly
        negotiated. Declaring an m=video here is required — a /tracks/new
        call deferred to after connect can't work, because the PC won't reach
        'connected' while the m-section is unbound (chicken-and-egg)."""
        body: dict = {"sessionDescription": {"type": "offer", "sdp": sdp_offer}}
        if tracks:
            body["tracks"] = tracks
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

    async def add_datachannels(self, session_id: str, channels: list[dict]) -> list[dict]:
        url = f"{self.base_url}/sessions/{session_id}/datachannels/new"
        attempts = 1 + len(_ADD_DC_RETRY_DELAYS_SEC)
        last_err: CloudflareRealtimeError | None = None

        for attempt in range(1, attempts + 1):
            log.info(
                "CF add_datachannels POST %s attempt=%d/%d body=%s",
                url, attempt, attempts, channels,
            )
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
            log.info(
                "CF add_datachannels attempt=%d status=%s body=%s",
                attempt, resp.status_code, resp.text[:500],
            )
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

    async def get_session(self, session_id: str) -> dict | None:
        """Get session info. Returns None if not found."""
        async with httpx.AsyncClient() as client:
            resp = await client.get(
                f"{self.base_url}/sessions/{session_id}",
                headers=self.headers,
                timeout=10.0,
            )
        if resp.status_code == 404:
            return None
        if resp.status_code != 200:
            raise CloudflareRealtimeError(resp.status_code, resp.text)
        return resp.json()

    async def close_session(self, session_id: str) -> None:
        """Close all tracks on a session (effectively ends it)."""
        # CF doesn't have a delete session endpoint — closing all tracks ends it.
        # Sessions auto-expire when no tracks remain.
        pass


cf_client = CloudflareRealtime()
