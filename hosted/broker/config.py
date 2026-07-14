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

from pydantic import model_validator
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    # Deployment environment: "prod" | "dev". Defaults to prod (fail-safe).
    environment: str = "prod"

    # Cloudflare Realtime SFU
    cf_teleop_app_id: str = ""
    cf_teleop_app_secret: str = ""
    cf_sfu_base_url: str = "https://rtc.live.cloudflare.com/v1/apps"

    # Cloudflare TURN service (same account, separate key — created in the
    # dashboard under Realtime → TURN). Optional: unset means STUN-only,
    # which only works for clients on UDP-open networks.
    cf_turn_key_id: str = ""
    cf_turn_api_token: str = ""
    cf_turn_base_url: str = "https://rtc.live.cloudflare.com/v1/turn"

    # LiveKit (alternative backend, per-session via `transport`). URL is dialed
    # by clients; key/secret mint room JWTs server-side. Unset → LiveKit sessions
    # 503; Cloudflare (the default) is unaffected.
    livekit_url: str = ""  # wss://<host> (self-hosted) or LiveKit Cloud project URL
    livekit_api_key: str = ""
    livekit_api_secret: str = ""
    livekit_token_ttl_sec: int = 3600  # JWT lifetime; clients reconnect within it

    # Cognito (operator auth). The broker only verifies tokens; sign-in
    # happens between the SPA and Cognito directly.
    cognito_region: str = "us-east-2"
    cognito_user_pool_id: str = ""
    cognito_client_id: str = ""

    # CORS
    public_origin: str = "https://teleop.dimensionalos.com"

    # Rate limiting (ratelimit.py). Passive by default: dry buckets only log
    # the would-be 429. Flip to true after the logs show clean limits.
    rate_limit_enforce: bool = False

    # Database
    database_url: str = "sqlite+aiosqlite:///./teleop.db"

    # Server. Loopback by default — Caddy is the only public entry; anything
    # that really wants a public bind must opt in with HOST=0.0.0.0.
    host: str = "127.0.0.1"
    port: int = 8450

    @model_validator(mode="after")
    def validate_secrets(self) -> "Settings":
        """Refuse to start misconfigured in production."""
        if self.environment != "dev":
            if not self.cognito_user_pool_id or not self.cognito_client_id:
                raise ValueError(
                    "COGNITO_USER_POOL_ID and COGNITO_CLIENT_ID must be set in production "
                    "(see terraform outputs cognito_user_pool_id / cognito_client_id)."
                )
            if not self.cf_teleop_app_secret:
                raise ValueError("CF_TELEOP_APP_SECRET must be set in production.")
        return self

    @property
    def cf_api_url(self) -> str:
        return f"{self.cf_sfu_base_url}/{self.cf_teleop_app_id}"

    @property
    def livekit_configured(self) -> bool:
        return bool(self.livekit_url and self.livekit_api_key and self.livekit_api_secret)

    @property
    def cognito_issuer(self) -> str:
        return (
            f"https://cognito-idp.{self.cognito_region}.amazonaws.com/{self.cognito_user_pool_id}"
        )

    model_config = {"env_file": ".env", "extra": "ignore"}


settings = Settings()
