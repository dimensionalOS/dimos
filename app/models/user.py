"""Operator user model — persisted to DB (not in-memory)."""

import uuid
from datetime import datetime, timezone

from sqlalchemy import Column, DateTime, String
from sqlalchemy.orm import Mapped

from models.database import Base


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


class User(Base):
    __tablename__ = "users"

    id: Mapped[str] = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    email: Mapped[str] = Column(String(256), nullable=False, unique=True, index=True)
    password_hash: Mapped[str] = Column(String(128), nullable=False)
    role: Mapped[str] = Column(String(32), default="operator")  # operator | admin
    created_at: Mapped[datetime] = Column(DateTime(timezone=True), default=_utcnow)
