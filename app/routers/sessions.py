"""Session lifecycle endpoints."""

import asyncio
import logging
from datetime import datetime, timezone

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from config import settings
from models.database import get_db
from models.session import TeleopSession
from services.auth import get_current_user, get_operator_or_robot, get_robot_id
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


# ─── Request/Response schemas ────────────────────────────────────────


class CreateSessionRequest(BaseModel):
    # Optional: the broker derives the canonical robot_id from the API key.
    # When provided it must match (guards against misconfigured robots).
    robot_id: str | None = None
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
    # CF renegotiation offer from the post-bridge video pull. None when the
    # robot published no video or the pull failed (video degrades, datachannels
    # still work). Operator answers it via /renegotiate-answer.
    video_offer: str | None = None
    # Why video_offer is null, surfaced to the operator console: "ok" |
    # "no_published_track" | "publish_error" | "pull_error" | "no_offer".
    video_status: str = "ok"


class RenegotiateAnswerRequest(BaseModel):
    sdp_answer: str


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


class TurnCredentialsResponse(BaseModel):
    ice_servers: list[dict]


# Fallback when TURN is unconfigured (dev) or the mint fails: STUN-only,
# which still connects clients on UDP-open networks.
ICE_SERVERS = [{"urls": "stun:stun.cloudflare.com:3478"}]


async def _mint_ice_servers() -> list[dict]:
    """STUN + short-lived TURN relay credentials, STUN-only on any failure."""
    if not settings.cf_turn_key_id or not settings.cf_turn_api_token:
        return ICE_SERVERS
    try:
        return await cf_client.generate_ice_servers()
    except Exception:
        log.exception("TURN credential mint failed; falling back to STUN only")
        return ICE_SERVERS


@router.get("/turn-credentials", response_model=TurnCredentialsResponse)
async def turn_credentials(identity: dict = Depends(get_operator_or_robot)):
    """Short-lived ICE servers for either side of the call.

    Clients fetch this BEFORE building their RTCPeerConnection — TURN must be
    in the initial config for relay candidates to gather with the offer.
    """
    return TurnCredentialsResponse(ice_servers=await _mint_ice_servers())


# ─── Robot endpoints ─────────────────────────────────────────────────


@router.post("", response_model=CreateSessionResponse, status_code=201)
async def create_session(
    body: CreateSessionRequest,
    robot_id: str = Depends(get_robot_id),
    db: AsyncSession = Depends(get_db),
):
    """Robot registers itself. Creates Cloudflare SFU session.

    The canonical robot_id comes from the API key. A robot_id in the body is
    legacy (older clients echo their TELEOP_ROBOT_ID env) and carries no
    authority — mismatches are logged, never rejected, since a stale env var
    on the robot must not block a validly-keyed connection.
    """
    if body.robot_id is not None and body.robot_id != robot_id:
        log.warning(
            "Ignoring body robot_id %r; key is bound to %r", body.robot_id, robot_id
        )

    # Close existing session for this robot if any
    existing = await db.execute(
        select(TeleopSession).where(
            TeleopSession.robot_id == robot_id,
            TeleopSession.state != "disconnected",
        )
    )
    for old in existing.scalars():
        old.state = "disconnected"

    # Record the robot's sendonly m=video (mid + trackName) from the offer. The
    # actual publish happens later via /tracks/new in bridge_datachannel — CF
    # ignores a `tracks` array on /sessions/new, so we only stash the ids here.
    published_mid: str | None = None
    published_track_name: str | None = None
    video = extract_video_track(body.sdp_offer)
    if video is not None:
        published_mid, published_track_name = video

    try:
        cf_result = await cf_client.create_session(body.sdp_offer)
    except CloudflareRealtimeError as e:
        raise HTTPException(status_code=502, detail=f"Cloudflare error: {e.detail}")
    except Exception as e:
        raise HTTPException(
            status_code=502,
            detail=f"Cloudflare session create failed ({type(e).__name__}): {e}",
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
        # Static STUN: clients fetch minted TURN from /turn-credentials and
        # never read this field. Minting here would be a wasted CF round-trip.
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

    # Join datachannels-clean (no video track here). Video is pulled after the
    # bridge, once the operator PC is connected — see bridge_datachannel.
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

    if body.role == "operator":
        session.operator_id = user_id
        session.operator_cf_session_id = operator_cf_id
        session.state = "active"
        await db.commit()

    return JoinSessionResponse(
        cf_session_id=operator_cf_id,
        sdp_answer=cf_result["sdp_answer"],
        robot_cf_session_id=session.cf_session_id,
        ice_servers=ICE_SERVERS,  # see create_session
        role=body.role,
    )


# Backoff for the operator video pull. CF's tracks/new returns a per-track
# not_found_track_error when the robot's RTP hasn't reached the SFU yet (a
# propagation race right after connect); retry until the packets are visible.
_PULL_RETRY_DELAYS = (0.3, 0.6, 1.0, 1.5, 2.0)


async def _pull_robot_video(session: TeleopSession) -> tuple[str | None, str]:
    """Publish the robot's local video track, then pull it onto the operator.

    Returns ``(video_offer, video_status)``. CF ignores a `tracks` array on
    /sessions/new, so the publisher is registered here via /tracks/new once the
    robot PC is connected; the operator then pulls it (a remote pull returns
    CF's renegotiation offer). All best-effort — the caller degrades to no-video
    on any failure rather than failing the bridge.
    """
    if not session.published_video_track_name:
        return None, "no_published_track"

    # Register the robot's local (publishable) track. /sessions/new's tracks
    # array is silently ignored by CF, so this explicit publish is what actually
    # exposes the track for the operator to pull.
    try:
        await cf_client.add_tracks(
            session.cf_session_id,
            [{
                "location": "local",
                "mid": session.published_video_mid,
                "trackName": session.published_video_track_name,
            }],
        )
    except Exception as e:
        log.error("video: publish robot track failed session=%s: %r", session.id, e)
        return None, "publish_error"

    for attempt in range(1 + len(_PULL_RETRY_DELAYS)):
        try:
            pull = await cf_client.add_tracks(
                session.operator_cf_session_id,
                [{
                    "location": "remote",
                    "sessionId": session.cf_session_id,
                    "trackName": session.published_video_track_name,
                }],
            )
        except Exception as e:
            log.error("video: pull failed session=%s: %r", session.id, e)
            return None, "pull_error"

        sd = pull.get("sessionDescription") or {}
        if sd.get("sdp"):
            # Use CF's offer whenever present — don't also require
            # requiresImmediateRenegotiation (CF omits it when the operator's
            # recvonly m=video section already existed).
            return sd["sdp"], "ok"

        track_errs = [t.get("errorCode") for t in pull.get("tracks", []) if t.get("errorCode")]
        if "not_found_track_error" in track_errs and attempt < len(_PULL_RETRY_DELAYS):
            await asyncio.sleep(_PULL_RETRY_DELAYS[attempt])
            continue

        log.warning("video: pull gave no offer session=%s errs=%s", session.id, track_errs)
        return None, "no_offer"

    return None, "no_offer"


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

    # CF requires each /datachannels/new call to be one direction (all local OR
    # all remote) — mixing errors "Pushing and Pulling ... unsupported". Hence 4
    # separate calls below, not 2; don't re-bundle.
    forward_names = [CMD_CHANNEL_NAME, STATE_CHANNEL_NAME]
    # The robot's CF session is long-lived, so its previous state_reliable_back
    # local push lingers across a disconnect (CF doesn't auto-reap datachannel
    # pushes). If we re-push without closing it → repeated_local_track_error.
    # So close the stale push (by its stored id) before re-pushing fresh, then
    # the new operator can subscribe cleanly. cmd/state are operator-published
    # (robot just re-subscribes), so they don't have this problem.
    if session.state_back_channel_id is not None:
        await cf_client.close_datachannels(
            session.cf_session_id, [session.state_back_channel_id]
        )
        session.state_back_channel_id = None
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
        # robot → operator: state_back. Fresh push each connect (stale one closed
        # above); operator subscribes to it.
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
        op_sub_ids = {entry["dataChannelName"]: int(entry["id"]) for entry in op_sub}
        robot_pub_ids = {entry["dataChannelName"]: int(entry["id"]) for entry in robot_pub}
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
    # Persist the fresh state_back push id on the session row (survives operator
    # leave, unlike _robot_channel_ids) so the NEXT reconnect can close this
    # stale push before re-pushing.
    session.state_back_channel_id = robot_pub_ids[STATE_BACK_CHANNEL_NAME]
    await db.commit()

    # Pull the robot's video onto the operator session (best-effort: a failure
    # degrades to no-video, never 502s the now-working datachannel bridge).
    video_offer, video_status = await _pull_robot_video(session)

    return BridgeDatachannelResponse(
        cmd_channel_id=op_pub_ids[CMD_CHANNEL_NAME],
        state_channel_id=op_pub_ids[STATE_CHANNEL_NAME],
        state_back_channel_id=op_sub_ids[STATE_BACK_CHANNEL_NAME],
        video_offer=video_offer,
        video_status=video_status,
    )


@router.post("/{session_id}/renegotiate-answer")
async def renegotiate_answer(
    session_id: str,
    body: RenegotiateAnswerRequest,
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """Operator submits its SDP answer to the video-pull renegotiation offer
    returned by bridge-datachannel."""
    session = await db.get(TeleopSession, session_id)
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")
    if session.operator_id != user["sub"]:
        raise HTTPException(status_code=403, detail="Not the bound operator")
    if not session.operator_cf_session_id:
        raise HTTPException(status_code=409, detail="Operator CF session not ready")

    try:
        await cf_client.renegotiate(session.operator_cf_session_id, body.sdp_answer)
    except CloudflareRealtimeError as e:
        raise HTTPException(
            status_code=502,
            detail=f"Cloudflare renegotiate failed: {e.detail}",
        )
    return {"ok": True}


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
