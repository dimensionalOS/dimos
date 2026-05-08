"""JWT auth for operators and API key auth for robots."""

from datetime import datetime, timedelta, timezone

from fastapi import Depends, HTTPException, Security
from fastapi.security import APIKeyHeader, HTTPAuthorizationCredentials, HTTPBearer
from jose import JWTError, jwt
from passlib.context import CryptContext
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from config import settings
from models.database import get_db
from models.user import User

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
bearer_scheme = HTTPBearer(auto_error=False)
api_key_header = APIKeyHeader(name="X-Robot-API-Key", auto_error=False)

# Legacy in-memory robot API keys (for dev bootstrap only)
ROBOT_API_KEYS: dict[str, str] = {}


def hash_password(password: str) -> str:
    return pwd_context.hash(password)


def verify_password(plain: str, hashed: str) -> bool:
    return pwd_context.verify(plain, hashed)


def create_token(user_id: str, role: str = "operator") -> str:
    expire = datetime.now(timezone.utc) + timedelta(hours=settings.jwt_expire_hours)
    payload = {"sub": user_id, "role": role, "exp": expire}
    return jwt.encode(payload, settings.jwt_secret, algorithm=settings.jwt_algorithm)


def decode_token(token: str) -> dict:
    try:
        return jwt.decode(token, settings.jwt_secret, algorithms=[settings.jwt_algorithm])
    except JWTError as e:
        raise HTTPException(status_code=401, detail=f"Invalid token: {e}")


async def get_current_user(
    credentials: HTTPAuthorizationCredentials | None = Security(bearer_scheme),
) -> dict:
    """Extract user from Bearer JWT token."""
    if credentials is None:
        raise HTTPException(status_code=401, detail="Not authenticated")
    return decode_token(credentials.credentials)


async def get_robot_id(
    api_key: str | None = Security(api_key_header),
    credentials: HTTPAuthorizationCredentials | None = Security(bearer_scheme),
) -> str:
    """Authenticate robot via API key header or Bearer token.

    If X-Robot-API-Key is provided, we ONLY check that path (no fallthrough to JWT).
    This prevents misconfigured robots from silently authenticating as operators.
    """
    if api_key:
        # Try legacy in-memory keys first
        if api_key in ROBOT_API_KEYS:
            return ROBOT_API_KEYS[api_key]

        # Try DB-backed API keys (dtk_live_... / dtk_dev_...)
        from models.database import async_session
        from services.keys import validate_api_key

        async with async_session() as db:
            key_record = await validate_api_key(db, api_key)
            if key_record:
                return key_record.robot_id or key_record.owner_id

        # API key was provided but invalid — fail immediately, no fallthrough
        raise HTTPException(status_code=401, detail="Invalid API key")

    # No API key provided — fall back to Bearer token
    if credentials:
        payload = decode_token(credentials.credentials)
        if payload.get("role") == "robot":
            return payload["sub"]

    raise HTTPException(status_code=401, detail="Invalid robot credentials")


# --- DB-backed user management ---


async def get_user_by_email(db: AsyncSession, email: str) -> User | None:
    result = await db.execute(select(User).where(User.email == email))
    return result.scalar_one_or_none()


async def create_user(db: AsyncSession, email: str, password: str, role: str = "operator") -> User:
    user = User(
        email=email,
        password_hash=hash_password(password),
        role=role,
    )
    db.add(user)
    await db.commit()
    await db.refresh(user)
    return user


def register_robot_key(api_key: str, robot_id: str) -> None:
    """Register a legacy in-memory API key for a robot."""
    ROBOT_API_KEYS[api_key] = robot_id
