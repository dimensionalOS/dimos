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

"""Client for broker-minted LiveKit robot sessions."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import os

from dimos.teleop.hosted.robot_type import RobotType
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class LiveKitSession:
    """Connection material minted by the hosted teleop broker."""

    session_id: str
    url: str
    token: str
    room: str


class LiveKitBrokerClient:
    """Create, maintain, and close one broker-mediated LiveKit session."""

    def __init__(self, broker_url: str | None, api_key: str | None) -> None:
        self._broker_url = (broker_url or os.environ.get("TELEOP_BROKER_URL", "")).rstrip("/")
        self._api_key = api_key or os.environ.get("TELEOP_API_KEY", "")
        if not self._broker_url:
            raise RuntimeError("broker_url or TELEOP_BROKER_URL required")
        if not self._api_key:
            raise RuntimeError("api_key or TELEOP_API_KEY required")

    @property
    def _headers(self) -> dict[str, str]:
        return {"X-Robot-API-Key": self._api_key, "Content-Type": "application/json"}

    async def create_session(
        self, robot_id: str | None, robot_name: str, robot_type: RobotType | None
    ) -> LiveKitSession:
        import httpx

        payload: dict[str, str] = {"transport": "livekit", "robot_name": robot_name}
        if robot_id:
            payload["robot_id"] = robot_id
        if robot_type is not None:
            payload["robot_type"] = robot_type.value
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(
                f"{self._broker_url}/api/v1/sessions", headers=self._headers, json=payload
            )
        if response.status_code not in (200, 201):
            raise RuntimeError(
                f"LiveKit broker session create failed: {response.status_code} {response.text[:200]}"
            )
        data = response.json()
        try:
            return LiveKitSession(
                session_id=data["session_id"],
                url=data["url"],
                token=data["token"],
                room=data["room"],
            )
        except (KeyError, TypeError) as error:
            raise RuntimeError(
                "LiveKit broker response missing session_id, url, token, or room"
            ) from error

    async def heartbeat(self, session_id: str, heartbeat_hz: float) -> None:
        import httpx

        interval = 1.0 / max(heartbeat_hz, 0.1)
        async with httpx.AsyncClient(timeout=30.0) as client:
            while True:
                try:
                    response = await client.post(
                        f"{self._broker_url}/api/v1/sessions/{session_id}/heartbeat",
                        headers=self._headers,
                        json={},
                    )
                    if response.status_code != 200:
                        logger.warning("LiveKit heartbeat failed", status=response.status_code)
                except Exception:
                    logger.warning("LiveKit heartbeat failed", exc_info=True)
                await asyncio.sleep(interval)

    async def close_session(self, session_id: str) -> None:
        import httpx

        try:
            async with httpx.AsyncClient(timeout=30.0) as client:
                await client.delete(
                    f"{self._broker_url}/api/v1/sessions/{session_id}", headers=self._headers
                )
        except Exception:
            logger.warning("LiveKit broker session delete failed", exc_info=True)
