"""Cognito JWT auth for operators and API key auth for robots.

Operators sign in against the Cognito user pool from the SPA; this module
only *verifies* the resulting RS256 ID tokens against the pool's JWKS.
The app-level identity is the (verified) email claim, so API key ownership,
robot-ID namespacing and session ownership are all keyed by email.
"""

import logging
import threading

import httpx
from fastapi import HTTPException, Security
from fastapi.security import APIKeyHeader, HTTPAuthorizationCredentials, HTTPBearer
from jose import JWTError, jwt

from config import settings

log = logging.getLogger(__name__)

bearer_scheme = HTTPBearer(auto_error=False)
api_key_header = APIKeyHeader(name="X-Robot-API-Key", auto_error=False)

# In-memory robot API keys (dev bootstrap only — prod uses dtk_* keys in DB)
ROBOT_API_KEYS: dict[str, str] = {}

# --- Cognito JWKS cache ---

_jwks_lock = threading.Lock()
_jwks_keys: dict[str, dict] = {}  # kid -> JWK


def _fetch_jwks() -> dict[str, dict]:
    url = f"{settings.cognito_issuer}/.well-known/jwks.json"
    resp = httpx.get(url, timeout=10)
    resp.raise_for_status()
    return {k["kid"]: k for k in resp.json()["keys"]}


def _get_jwk(kid: str) -> dict | None:
    with _jwks_lock:
        if kid not in _jwks_keys:
            try:
                _jwks_keys.update(_fetch_jwks())
            except Exception as e:
                log.error("JWKS fetch failed: %s", e)
        return _jwks_keys.get(kid)


def decode_token(token: str) -> dict:
    """Verify a Cognito ID token and return our app-level claims.

    The returned dict keeps the historical shape: `sub` is the user's email
    (everything downstream keys on it) and `role` is operator/admin.
    """
    try:
        kid = jwt.get_unverified_header(token).get("kid", "")
    except JWTError as e:
        raise HTTPException(status_code=401, detail=f"Invalid token: {e}")

    jwk = _get_jwk(kid)
    if jwk is None:
        raise HTTPException(status_code=401, detail="Invalid token: unknown key id")

    try:
        claims = jwt.decode(
            token,
            jwk,
            algorithms=["RS256"],
            audience=settings.cognito_client_id,
            issuer=settings.cognito_issuer,
        )
    except JWTError as e:
        raise HTTPException(status_code=401, detail=f"Invalid token: {e}")

    if claims.get("token_use") != "id":
        raise HTTPException(status_code=401, detail="Invalid token: not an ID token")
    email = claims.get("email")
    if not email or not claims.get("email_verified"):
        raise HTTPException(status_code=401, detail="Email not verified")

    role = "admin" if "admin" in claims.get("cognito:groups", []) else "operator"
    return {"sub": email, "role": role, "cognito_sub": claims["sub"]}


async def get_current_user(
    credentials: HTTPAuthorizationCredentials | None = Security(bearer_scheme),
) -> dict:
    """Extract user from Bearer Cognito ID token."""
    if credentials is None:
        raise HTTPException(status_code=401, detail="Not authenticated")
    return decode_token(credentials.credentials)


async def get_robot_id(api_key: str | None = Security(api_key_header)) -> str:
    """Authenticate robot via X-Robot-API-Key header."""
    if api_key:
        # Dev bootstrap keys first
        if api_key in ROBOT_API_KEYS:
            return ROBOT_API_KEYS[api_key]

        # DB-backed API keys (dtk_live_... / dtk_dev_...)
        from models.database import async_session
        from services.keys import validate_api_key

        async with async_session() as db:
            key_record = await validate_api_key(db, api_key)
            if key_record:
                return key_record.robot_id or key_record.owner_id

    raise HTTPException(status_code=401, detail="Invalid robot credentials")


async def get_operator_or_robot(
    credentials: HTTPAuthorizationCredentials | None = Security(bearer_scheme),
    api_key: str | None = Security(api_key_header),
) -> dict:
    """Accept either identity for endpoints both sides use (TURN credentials).

    Robot key wins when both are present (robots never send a Bearer token).
    """
    if api_key:
        robot_id = await get_robot_id(api_key)
        return {"sub": robot_id, "role": "robot"}
    if credentials is not None:
        return decode_token(credentials.credentials)
    raise HTTPException(status_code=401, detail="Not authenticated")


def register_robot_key(api_key: str, robot_id: str) -> None:
    """Register an in-memory API key for a robot (dev bootstrap)."""
    ROBOT_API_KEYS[api_key] = robot_id
