from pydantic import model_validator
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    # Deployment environment: "prod" | "dev". Defaults to prod (fail-safe).
    environment: str = "prod"

    # Cloudflare Realtime SFU
    cf_teleop_app_id: str = ""
    cf_teleop_app_secret: str = ""
    cf_sfu_base_url: str = "https://rtc.live.cloudflare.com/v1/apps"

    # Auth
    jwt_secret: str = ""
    jwt_algorithm: str = "HS256"
    jwt_expire_hours: int = 24

    # CORS
    public_origin: str = "https://teleop.dimensionalos.com"

    # Database
    database_url: str = "sqlite+aiosqlite:///./teleop.db"

    # Server
    host: str = "0.0.0.0"
    port: int = 8450

    @model_validator(mode="after")
    def validate_secrets(self) -> "Settings":
        """Refuse to start with default secrets in production."""
        if self.environment != "dev":
            if not self.jwt_secret or self.jwt_secret == "change-me":
                raise ValueError(
                    "JWT_SECRET must be set in production. "
                    'Generate one with: python3 -c "import secrets; print(secrets.token_hex(32))"'
                )
            if not self.cf_teleop_app_secret:
                raise ValueError("CF_TELEOP_APP_SECRET must be set in production.")
        return self

    @property
    def cf_api_url(self) -> str:
        return f"{self.cf_sfu_base_url}/{self.cf_teleop_app_id}"

    model_config = {"env_file": ".env", "extra": "ignore"}


settings = Settings()

if settings.environment == "prod" and len(settings.jwt_secret) < 32:
    raise RuntimeError(
        "JWT_SECRET must be set to a random string of >=32 chars in prod"
    )
