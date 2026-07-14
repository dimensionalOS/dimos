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

import logging

from config import settings
from sqlalchemy import inspect
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine
from sqlalchemy.orm import DeclarativeBase

log = logging.getLogger(__name__)

engine = create_async_engine(settings.database_url, echo=False)
async_session = async_sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)


class Base(DeclarativeBase):
    pass


# Nullable columns added to existing tables on startup. create_all() only
# creates missing tables, never alters existing ones, and we don't have
# Alembic — so each additive column on the model also gets listed here so
# old prod databases pick it up via ALTER TABLE on next boot.
_ADDITIVE_COLUMNS: list[tuple[str, str, str]] = [
    ("teleop_sessions", "published_video_mid", "VARCHAR"),
    ("teleop_sessions", "published_video_track_name", "VARCHAR"),
    ("teleop_sessions", "state_back_channel_id", "INTEGER"),
    ("teleop_sessions", "map_channel_id", "INTEGER"),
    ("teleop_sessions", "operator_audio_mid", "VARCHAR"),
    ("teleop_sessions", "operator_audio_track_name", "VARCHAR"),
    ("teleop_sessions", "owner_id", "VARCHAR"),
    # DEFAULT backfills existing rows as cloudflare (the only backend before
    # this); NOT NULL matches the ORM so migrated and fresh schemas agree.
    ("teleop_sessions", "transport", "VARCHAR DEFAULT 'cloudflare' NOT NULL"),
    ("teleop_sessions", "last_operator_heartbeat", "TIMESTAMP"),
]


async def init_db() -> None:
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
        await conn.run_sync(_apply_additive_columns)


def _apply_additive_columns(sync_conn) -> None:
    insp = inspect(sync_conn)
    for table, column, sql_type in _ADDITIVE_COLUMNS:
        existing = {c["name"] for c in insp.get_columns(table)}
        if column in existing:
            continue
        sync_conn.exec_driver_sql(f"ALTER TABLE {table} ADD COLUMN {column} {sql_type}")
        log.info("Added column %s.%s (%s)", table, column, sql_type)


async def get_db() -> AsyncSession:  # type: ignore[misc]
    async with async_session() as session:
        yield session
