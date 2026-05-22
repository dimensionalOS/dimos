"""Session lifecycle endpoints."""

import logging
from datetime import datetime, timezone

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from models.database import get_db
from models.session import TeleopSession
from services.auth import get_current_user, get_robot_id
from services.cloudflare import CloudflareRealtimeError, cf_client
from services.sdp_utils import extract_video_track

log = logging.getLogger(__name__)

router = APIRouter(prefix="/sessions", tags=["sessions"])

CMD_CHANNEL_NAME = "cmd_unreliable"
STATE_CHANNEL_NAME = "state_reliable"
# Robot publishes pongs on this channel — CF datachannel routing is strict
# publisher → subscriber per name, so the reverse direction needs its own name
# rather than reusing `state_reliable`.
STATE_BACK_CHANNEL_NAME = "state_reliable_back"

# Per-session map of channel-name → robot-side SCTP id. Holds both subscriber
# ids (cmd_unreliable, state_reliable that the robot reads) and the publisher
# id for state_reliable_back (which the robot writes). Heartbeat surfaces each
# id under a role-appropriate field name.
_robot_channel_ids: dict[str, dict[str, int]] = {}

# Operator SDP offer captured at join, consumed at bridge-datachannel time for
# the video subscribe renegotiation (CF needs the operator's offer to bind the
# remote track). Short-lived: join → bridge is seconds. Lost on restart, which
# is fine — the operator reconnects.
_operator_offers: dict[str, str] = {}


# ─── Request/Response schemas ────────────────────────────────────────


class CreateSessionRequest(BaseModel):
    robot_id: str
    robot_name: str
    sdp_offer: str


class CreateSessionResponse(BaseModel):
    session_id: str
    cf_session_id: str
    sdp_answer: str
    ice_servers: list[dict]


class JoinSessionRequest(BaseModel):
    role: str = "operator"  # operator | viewer
    sdp_offer: str


class JoinSessionResponse(BaseModel):
    cf_session_id: str
    sdp_answer: str
    robot_cf_session_id: str
    ice_servers: list[dict]
    role: str


class BridgeDatachannelResponse(BaseModel):
    cmd_channel_id: int
    state_channel_id: int
    state_back_channel_id: int
    # Renegotiated SDP answer the operator must apply if a video subscribe
    # happened. None when the session has no published video. The operator
    # applies it as a post-connect renegotiation to receive the routed track.
    video_sdp_answer: str | None = None


class HeartbeatRequest(BaseModel):
    rtt_ms: float | None = None
    packet_loss_pct: float | None = None
    video_bitrate_kbps: int | None = None
    command_rate_hz: float | None = None
    safety_state: str = "nominal"


class SessionInfo(BaseModel):
    session_id: str
    robot_id: str
    robot_name: str
    state: str
    operator_id: str | None
    rtt_ms: float | None
    packet_loss_pct: float | None
    created_at: datetime


class LeaveRequest(BaseModel):
    reason: str = "user_initiated"


ICE_SERVERS = [{"urls": "stun:stun.cloudflare.com:3478"}]


# ─── Robot endpoints ─────────────────────────────────────────────────


@router.post("", response_model=CreateSessionResponse, status_code=201)
async def create_session(
    body: CreateSessionRequest,
    robot_id: str = Depends(get_robot_id),
    db: AsyncSession = Depends(get_db),
):
    """Robot registers itself. Creates Cloudflare SFU session."""
    # Verify robot_id matches the authenticated key
    if body.robot_id != robot_id:
        raise HTTPException(status_code=403, detail="Robot ID mismatch")

    # Close existing session for this robot if any
    existing = await db.execute(
        select(TeleopSession).where(
            TeleopSession.robot_id == robot_id,
            TeleopSession.state != "disconnected",
        )
    )
    for old in existing.scalars():
        old.state = "disconnected"

    # Create CF session
    try:
        cf_result = await cf_client.create_session(body.sdp_offer)
    except CloudflareRealtimeError as e:
        raise HTTPException(status_code=502, detail=f"Cloudflare error: {e.detail}")
    except Exception as e:
        raise HTTPException(
            status_code=502,
            detail=f"Cloudflare session create failed ({type(e).__name__}): {e}",
        )

    # Extract the robot's sendonly m=video (if any) and remember it, but DON'T
    # call CF add_tracks here — CF rejects /tracks/new with "session not ready"
    # until the robot's PC is connected, and at this point the robot hasn't
    # even received this answer yet. The actual publish happens later in
    # bridge-datachannel, where both PCs are connected.
    published_mid: str | None = None
    published_track_name: str | None = None
    video = extract_video_track(body.sdp_offer)
    if video is not None:
        published_mid, published_track_name = video
        log.info(
            "Robot offered video robot=%s mid=%s trackName=%s (publish deferred to bridge)",
            robot_id, published_mid, published_track_name,
        )

    # Store session
    session = TeleopSession(
        robot_id=robot_id,
        robot_name=body.robot_name,
        state="idle",
        cf_session_id=cf_result["cf_session_id"],
        published_video_mid=published_mid,
        published_video_track_name=published_track_name,
    )
    db.add(session)
    await db.commit()
    await db.refresh(session)

    return CreateSessionResponse(
        session_id=session.id,
        cf_session_id=cf_result["cf_session_id"],
        sdp_answer=cf_result["sdp_answer"],
        ice_servers=ICE_SERVERS,
    )


@router.post("/{session_id}/heartbeat")
async def heartbeat(
    session_id: str,
    body: HeartbeatRequest,
    robot_id: str = Depends(get_robot_id),
    db: AsyncSession = Depends(get_db),
):
    """Robot reports connection quality metrics."""
    session = await db.get(TeleopSession, session_id)
    if not session or session.robot_id != robot_id:
        raise HTTPException(status_code=404, detail="Session not found")

    session.rtt_ms = body.rtt_ms
    session.packet_loss_pct = body.packet_loss_pct
    session.video_bitrate_kbps = body.video_bitrate_kbps
    session.command_rate_hz = body.command_rate_hz
    session.last_heartbeat = datetime.now(timezone.utc)
    await db.commit()

    chan_ids = _robot_channel_ids.get(session_id, {})
    return {
        "ack": True,
        "cmd_channel_subscriber_id": chan_ids.get(CMD_CHANNEL_NAME),
        "state_channel_subscriber_id": chan_ids.get(STATE_CHANNEL_NAME),
        "state_back_channel_publisher_id": chan_ids.get(STATE_BACK_CHANNEL_NAME),
    }


@router.delete("/{session_id}", status_code=204)
async def delete_session(
    session_id: str,
    robot_id: str = Depends(get_robot_id),
    db: AsyncSession = Depends(get_db),
):
    """Robot going offline."""
    session = await db.get(TeleopSession, session_id)
    if not session or session.robot_id != robot_id:
        raise HTTPException(status_code=404, detail="Session not found")

    session.state = "disconnected"
    _robot_channel_ids.pop(session_id, None)
    _operator_offers.pop(session_id, None)
    await db.commit()


# ─── Operator endpoints ──────────────────────────────────────────────


@router.get("", response_model=list[SessionInfo])
async def list_sessions(
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """List available robots (active sessions)."""
    result = await db.execute(
        select(TeleopSession).where(TeleopSession.state.in_(["idle", "active"]))
    )
    sessions = result.scalars().all()
    return [
        SessionInfo(
            session_id=s.id,
            robot_id=s.robot_id,
            robot_name=s.robot_name,
            state=s.state,
            operator_id=s.operator_id,
            rtt_ms=s.rtt_ms,
            packet_loss_pct=s.packet_loss_pct,
            created_at=s.created_at,
        )
        for s in sessions
    ]


@router.post("/{session_id}/join", response_model=JoinSessionResponse)
async def join_session(
    session_id: str,
    body: JoinSessionRequest,
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """Operator or viewer joins a session."""
    session = await db.get(TeleopSession, session_id)
    if not session or session.state == "disconnected":
        raise HTTPException(status_code=404, detail="Session not found")

    user_id = user["sub"]

    # Enforce single operator
    if body.role == "operator":
        if session.operator_id and session.operator_id != user_id:
            raise HTTPException(
                status_code=409,
                detail=f"Session already has operator: {session.operator_id}",
            )

    # Create a new CF session for this operator (they get their own PeerConnection)
    try:
        cf_result = await cf_client.create_session(body.sdp_offer)
    except CloudflareRealtimeError as e:
        raise HTTPException(status_code=502, detail=f"Cloudflare error: {e.detail}")
    except Exception as e:
        raise HTTPException(
            status_code=502,
            detail=f"Cloudflare session create failed ({type(e).__name__}): {e}",
        )

    operator_cf_id = cf_result["cf_session_id"]

    # Don't subscribe the operator to the robot's video here — CF rejects
    # /tracks/new until the operator's PC is connected, which won't happen
    # until after they apply this answer. Stash the offer; the subscribe runs
    # in bridge-datachannel (post-connect), same as the publisher side.
    if body.role == "operator":
        session.operator_id = user_id
        session.operator_cf_session_id = operator_cf_id
        session.state = "active"
        _operator_offers[session.id] = body.sdp_offer
        await db.commit()

    return JoinSessionResponse(
        cf_session_id=operator_cf_id,
        sdp_answer=cf_result["sdp_answer"],
        robot_cf_session_id=session.cf_session_id,
        ice_servers=ICE_SERVERS,
        role=body.role,
    )


@router.post(
    "/{session_id}/bridge-datachannel",
    response_model=BridgeDatachannelResponse,
)
async def bridge_datachannel(
    session_id: str,
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """Bridge cmd_unreliable. Call after operator's PC is 'connected' — CF
    rejects /datachannels/new on a half-negotiated session."""
    session = await db.get(TeleopSession, session_id)
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")
    if session.operator_id != user["sub"]:
        raise HTTPException(status_code=403, detail="Not the bound operator")
    if not session.operator_cf_session_id or not session.cf_session_id:
        raise HTTPException(status_code=409, detail="CF sessions not ready")

    # Publish the robot's video BEFORE bridging datachannels. An un-bound
    # sendonly m=video keeps the robot's CF session "not ready", which makes
    # the robot-side add_datachannels calls below fail — exactly the failure
    # we bisected during P1. Binding it here (robot PC connected long ago)
    # makes the session ready. Robot side needs no renegotiation: add_tracks
    # just labels an m-section already present in the robot's offer.
    if session.published_video_track_name and session.published_video_mid:
        try:
            await cf_client.add_tracks(
                session.cf_session_id,
                [
                    {
                        "location": "local",
                        "mid": session.published_video_mid,
                        "trackName": session.published_video_track_name,
                    }
                ],
            )
        except CloudflareRealtimeError as e:
            raise HTTPException(
                status_code=502,
                detail=f"Cloudflare add_tracks (robot publish) failed: {e.detail}",
            )
        except Exception as e:
            raise HTTPException(
                status_code=502,
                detail=f"Robot publish failed ({type(e).__name__}): {e}",
            )
        log.info(
            "Published video robot_cf=%s mid=%s trackName=%s",
            session.cf_session_id, session.published_video_mid,
            session.published_video_track_name,
        )

    # CF constraint: each /datachannels/new request body's `dataChannels`
    # array must be homogeneous in direction — all `location: "local"` OR
    # all `location: "remote"`. Mixing them yields
    #   errorCode=invalid_params
    #   errorDescription="Pushing and Pulling in the same request is currently unsupported"
    # So this is 4 separate calls (operator pub, robot sub, robot pub, operator sub),
    # not 2 bundled ones. Don't re-bundle in a future refactor.
    forward_names = [CMD_CHANNEL_NAME, STATE_CHANNEL_NAME]
    try:
        # operator → robot: cmd + state. Operator publishes, robot subscribes.
        op_pub = await cf_client.add_datachannels(
            session.operator_cf_session_id,
            [{"location": "local", "dataChannelName": name} for name in forward_names],
        )
        robot_sub = await cf_client.add_datachannels(
            session.cf_session_id,
            [
                {
                    "location": "remote",
                    "sessionId": session.operator_cf_session_id,
                    "dataChannelName": name,
                }
                for name in forward_names
            ],
        )
        # robot → operator: state_back. Robot publishes pongs, operator subscribes.
        robot_pub = await cf_client.add_datachannels(
            session.cf_session_id,
            [{"location": "local", "dataChannelName": STATE_BACK_CHANNEL_NAME}],
        )
        op_sub = await cf_client.add_datachannels(
            session.operator_cf_session_id,
            [
                {
                    "location": "remote",
                    "sessionId": session.cf_session_id,
                    "dataChannelName": STATE_BACK_CHANNEL_NAME,
                }
            ],
        )
    except CloudflareRealtimeError as e:
        raise HTTPException(
            status_code=502,
            detail=f"Cloudflare datachannel bridge failed: {e.detail}",
        )
    except Exception as e:
        raise HTTPException(
            status_code=502,
            detail=f"Datachannel bridge failed ({type(e).__name__}): {e}",
        )

    # Index by dataChannelName from the response, not by request position —
    # don't assume CF preserves order across the array.
    try:
        op_pub_ids = {entry["dataChannelName"]: int(entry["id"]) for entry in op_pub}
        robot_sub_ids = {entry["dataChannelName"]: int(entry["id"]) for entry in robot_sub}
        robot_pub_ids = {entry["dataChannelName"]: int(entry["id"]) for entry in robot_pub}
        op_sub_ids = {entry["dataChannelName"]: int(entry["id"]) for entry in op_sub}
    except (KeyError, TypeError, ValueError) as e:
        raise HTTPException(
            status_code=502,
            detail=f"Cloudflare returned malformed DataChannel entry: {e}",
        )

    missing = [n for n in forward_names if n not in op_pub_ids or n not in robot_sub_ids]
    if STATE_BACK_CHANNEL_NAME not in robot_pub_ids or STATE_BACK_CHANNEL_NAME not in op_sub_ids:
        missing.append(STATE_BACK_CHANNEL_NAME)
    if missing:
        raise HTTPException(
            status_code=502,
            detail=f"Cloudflare missing DataChannel id for: {', '.join(missing)}",
        )

    # Heartbeat surfaces robot-side ids. Robot subscribes to cmd + state,
    # publishes state_back — keep them all under one channel-name map.
    _robot_channel_ids[session.id] = {
        **robot_sub_ids,
        STATE_BACK_CHANNEL_NAME: robot_pub_ids[STATE_BACK_CHANNEL_NAME],
    }

    # Subscribe the operator to the robot's published video — AFTER the
    # datachannel bridge, so the operator's CF session is already proven
    # ready (the bridge calls above would have failed otherwise). CF
    # renegotiates: it returns a new SDP answer the operator must apply to
    # actually receive the routed track.
    video_sdp_answer: str | None = None
    operator_offer = _operator_offers.get(session.id)
    if session.published_video_track_name and operator_offer:
        try:
            sub_result = await cf_client.add_tracks(
                session.operator_cf_session_id,
                [
                    {
                        "location": "remote",
                        "sessionId": session.cf_session_id,
                        "trackName": session.published_video_track_name,
                    }
                ],
                sdp_offer=operator_offer,
            )
        except CloudflareRealtimeError as e:
            raise HTTPException(
                status_code=502,
                detail=f"Cloudflare add_tracks (operator subscribe) failed: {e.detail}",
            )
        except Exception as e:
            raise HTTPException(
                status_code=502,
                detail=f"Operator subscribe failed ({type(e).__name__}): {e}",
            )
        video_sdp_answer = sub_result.get("sessionDescription", {}).get("sdp")
        if not video_sdp_answer:
            log.warning(
                "CF add_tracks (operator subscribe) returned no SDP for operator_cf=%s; "
                "video will not be routed",
                session.operator_cf_session_id,
            )

    return BridgeDatachannelResponse(
        cmd_channel_id=op_pub_ids[CMD_CHANNEL_NAME],
        state_channel_id=op_pub_ids[STATE_CHANNEL_NAME],
        state_back_channel_id=op_sub_ids[STATE_BACK_CHANNEL_NAME],
        video_sdp_answer=video_sdp_answer,
    )


@router.post("/{session_id}/leave")
async def leave_session(
    session_id: str,
    body: LeaveRequest,
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """Operator or viewer leaves."""
    session = await db.get(TeleopSession, session_id)
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    user_id = user["sub"]

    if session.operator_id == user_id:
        session.operator_id = None
        session.operator_cf_session_id = None
        session.state = "idle"
        _robot_channel_ids.pop(session_id, None)
        _operator_offers.pop(session_id, None)
        await db.commit()

    return {"session_id": session_id, "state": session.state}


@router.get("/{session_id}/status", response_model=SessionInfo)
async def session_status(
    session_id: str,
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """Get session status and connection quality."""
    session = await db.get(TeleopSession, session_id)
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    return SessionInfo(
        session_id=session.id,
        robot_id=session.robot_id,
        robot_name=session.robot_name,
        state=session.state,
        operator_id=session.operator_id,
        rtt_ms=session.rtt_ms,
        packet_loss_pct=session.packet_loss_pct,
        created_at=session.created_at,
    )
