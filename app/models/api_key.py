"""API key model for developer robot authentication."""

import uuid
from datetime import datetime, timezone

from sqlalchemy import Boolean, Column, DateTime, String
from sqlalchemy.orm import Mapped

from models.database import Base


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


class APIKey(Base):
    __tablename__ = "api_keys"

    id: Mapped[str] = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    # The key prefix (first 14 chars) for display/identification
    key_prefix: Mapped[str] = Column(String(16), nullable=False)
    # SHA-256 hash of the full key (never store plaintext)
    key_hash: Mapped[str] = Column(String(64), nullable=False, unique=True, index=True)
    # Human-readable label
    name: Mapped[str] = Column(String(128), nullable=False)
    # Owner (email from JWT)
    owner_id: Mapped[str] = Column(String(256), nullable=False, index=True)
    # Robot ID this key is associated with (namespaced as owner:robot_id)
    robot_id: Mapped[str | None] = Column(String(256), nullable=True)

    # State
    revoked: Mapped[bool] = Column(Boolean, default=False)
    last_used_at: Mapped[datetime | None] = Column(DateTime(timezone=True), nullable=True)
    created_at: Mapped[datetime] = Column(DateTime(timezone=True), default=_utcnow)
