from pydantic import model_validator
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    # Deployment environment: "prod" | "dev". Defaults to prod (fail-safe).
    environment: str = "prod"

    # Cloudflare Realtime SFU
    cf_teleop_app_id: str = ""
    cf_teleop_app_secret: str = ""
    cf_sfu_base_url: str = "https://rtc.live.cloudflare.com/v1/apps"

    # Cognito (operator auth). The broker only verifies tokens; sign-in
    # happens between the SPA and Cognito directly.
    cognito_region: str = "us-east-2"
    cognito_user_pool_id: str = ""
    cognito_client_id: str = ""

    # CORS
    public_origin: str = "https://teleop.dimensionalos.com"

    # Database
    database_url: str = "sqlite+aiosqlite:///./teleop.db"

    # Server
    host: str = "0.0.0.0"
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
    def cognito_issuer(self) -> str:
        return (
            f"https://cognito-idp.{self.cognito_region}.amazonaws.com/"
            f"{self.cognito_user_pool_id}"
        )

    model_config = {"env_file": ".env", "extra": "ignore"}


settings = Settings()
