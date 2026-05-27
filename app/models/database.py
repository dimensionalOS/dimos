import logging

from sqlalchemy import inspect
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine
from sqlalchemy.orm import DeclarativeBase

from config import settings

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
        sync_conn.exec_driver_sql(
            f"ALTER TABLE {table} ADD COLUMN {column} {sql_type}"
        )
        log.info("Added column %s.%s (%s)", table, column, sql_type)


async def get_db() -> AsyncSession:  # type: ignore[misc]
    async with async_session() as session:
        yield session
