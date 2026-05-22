import uuid
from datetime import datetime, timezone

from sqlalchemy import Column, DateTime, Float, Integer, String
from sqlalchemy.orm import Mapped

from models.database import Base


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


class TeleopSession(Base):
    __tablename__ = "teleop_sessions"

    id: Mapped[str] = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    robot_id: Mapped[str] = Column(String, nullable=False, index=True)
    robot_name: Mapped[str] = Column(String, nullable=False)
    state: Mapped[str] = Column(String, default="idle")  # idle | active | disconnected
    cf_session_id: Mapped[str] = Column(String, nullable=True)

    # Video the robot offered (sendonly m=video). Both extracted from the
    # robot's SDP at create_session and persisted, but the actual CF
    # add_tracks(local) publish happens later in bridge-datachannel — CF
    # rejects /tracks/new until the robot's PC is connected, which isn't true
    # yet at create_session time. Both None when the robot offered no video.
    published_video_mid: Mapped[str | None] = Column(String, nullable=True)
    published_video_track_name: Mapped[str | None] = Column(String, nullable=True)

    # Active operator (null = no one controlling)
    operator_id: Mapped[str | None] = Column(String, nullable=True)
    operator_cf_session_id: Mapped[str | None] = Column(String, nullable=True)

    # Connection quality (updated by robot heartbeat)
    rtt_ms: Mapped[float | None] = Column(Float, nullable=True)
    packet_loss_pct: Mapped[float | None] = Column(Float, nullable=True)
    video_bitrate_kbps: Mapped[int | None] = Column(Integer, nullable=True)
    command_rate_hz: Mapped[float | None] = Column(Float, nullable=True)

    created_at: Mapped[datetime] = Column(DateTime(timezone=True), default=_utcnow)
    last_heartbeat: Mapped[datetime | None] = Column(DateTime(timezone=True), nullable=True)
