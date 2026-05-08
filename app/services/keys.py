"""API key generation, validation, and management."""

import hashlib
import secrets
from datetime import datetime, timezone

from sqlalchemy import select, update
from sqlalchemy.ext.asyncio import AsyncSession

from models.api_key import APIKey

# Key format: dtk_live_<40 random hex chars> = 49 chars total
KEY_PREFIX_LIVE = "dtk_live_"
KEY_PREFIX_DEV = "dtk_dev_"

# Only update last_used_at if older than this (avoid write-per-request on SQLite)
LAST_USED_THROTTLE_SECONDS = 60


def generate_api_key(environment: str = "live") -> str:
    """Generate a new API key. Returns plaintext (shown to user once)."""
    prefix = KEY_PREFIX_LIVE if environment == "live" else KEY_PREFIX_DEV
    random_part = secrets.token_hex(20)  # 40 hex chars
    return f"{prefix}{random_part}"


def hash_key(plaintext_key: str) -> str:
    """SHA-256 hash of the API key for storage."""
    return hashlib.sha256(plaintext_key.encode()).hexdigest()


def get_key_prefix(plaintext_key: str) -> str:
    """Extract displayable prefix from a key (e.g. dtk_live_a3f2...)."""
    return plaintext_key[:14]


async def create_api_key(
    db: AsyncSession,
    owner_id: str,
    name: str,
    robot_id: str | None = None,
    environment: str = "live",
) -> tuple[APIKey, str]:
    """Create a new API key. Returns (db record, plaintext key).

    The plaintext key is returned ONCE and never stored.
    Robot IDs are namespaced to the owner to prevent cross-tenant collisions.
    """
    plaintext = generate_api_key(environment)

    # Namespace robot_id to prevent cross-tenant impersonation
    namespaced_robot_id = f"{owner_id}:{robot_id}" if robot_id else None

    key_record = APIKey(
        key_prefix=get_key_prefix(plaintext),
        key_hash=hash_key(plaintext),
        name=name,
        owner_id=owner_id,
        robot_id=namespaced_robot_id,
    )
    db.add(key_record)
    await db.commit()
    await db.refresh(key_record)
    return key_record, plaintext


async def validate_api_key(db: AsyncSession, plaintext_key: str) -> APIKey | None:
    """Validate an API key. Returns the key record if valid, None otherwise.

    Throttles last_used_at updates to avoid write-per-request on SQLite.
    """
    hashed = hash_key(plaintext_key)
    result = await db.execute(
        select(APIKey).where(APIKey.key_hash == hashed, APIKey.revoked == False)  # noqa: E712
    )
    key_record = result.scalar_one_or_none()
    if key_record:
        # Throttle last_used_at writes (only if >60s stale)
        now = datetime.now(timezone.utc)
        if (
            key_record.last_used_at is None
            or (now - key_record.last_used_at).total_seconds() > LAST_USED_THROTTLE_SECONDS
        ):
            await db.execute(
                update(APIKey)
                .where(APIKey.id == key_record.id)
                .values(last_used_at=now)
            )
            await db.commit()
    return key_record


async def list_api_keys(db: AsyncSession, owner_id: str) -> list[APIKey]:
    """List all API keys for an owner (non-revoked)."""
    result = await db.execute(
        select(APIKey)
        .where(APIKey.owner_id == owner_id, APIKey.revoked == False)  # noqa: E712
        .order_by(APIKey.created_at.desc())
    )
    return list(result.scalars().all())


async def revoke_api_key(db: AsyncSession, key_id: str, owner_id: str) -> bool:
    """Revoke an API key. Returns True if found and revoked."""
    result = await db.execute(
        update(APIKey)
        .where(APIKey.id == key_id, APIKey.owner_id == owner_id, APIKey.revoked == False)  # noqa: E712
        .values(revoked=True)
    )
    await db.commit()
    return result.rowcount > 0
