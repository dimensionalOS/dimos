"""Session lifecycle endpoints."""

import asyncio
import logging
import uuid
from datetime import datetime, timezone
from typing import Literal

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from sqlalchemy import or_, select, update
from sqlalchemy.ext.asyncio import AsyncSession

from config import settings
from models.database import get_db
from models.session import TeleopSession
from services import livekit
from services.auth import get_current_user, get_operator_or_robot, get_robot_owner
from services.cloudflare import CloudflareRealtimeError, cf_client
from services.livekit import LiveKitError
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

# Per-session asyncio.Lock. Serializes the bridge handler so two concurrent
# /bridge-datachannel calls (e.g. operator double-click) can't both read
# session.state_back_channel_id stale, both close it, then both create new
# pushes — the second hitting repeated_local_track_error and leaving the
# session permanently un-bridgeable. dict.setdefault is GIL-atomic; we never
# delete entries (small memory cost vs. correctness — sessions are bounded).
_session_locks: dict[str, asyncio.Lock] = {}


def _session_lock(session_id: str) -> asyncio.Lock:
    return _session_locks.setdefault(session_id, asyncio.Lock())


# session_ids with a video-pull offer awaiting the operator's answer. Set by
# bridge_datachannel when _pull_robot_video returns a video_offer; consumed by
# renegotiate_answer. Without this, a replayed or fabricated answer would be
# forwarded to CF and could 4xx the session mid-stream.
_pending_video_renegotiations: set[str] = set()


# ─── Request/Response schemas ────────────────────────────────────────


class CreateSessionRequest(BaseModel):
    # Optional: the broker derives the canonical robot_id from the API key.
    # When provided it must match (guards against misconfigured robots).
    robot_id: str | None = None
    robot_name: str
    # Validated by the schema, so an unknown transport is a 422 before any
    # handler code runs (no manual check needed downstream).
    transport: Literal["cloudflare", "livekit"] = "cloudflare"
    # Required for cloudflare (broker relays it to CF); unused for livekit,
    # which does its own SDP negotiation directly with the LiveKit server.
    sdp_offer: str | None = None


class CreateSessionResponse(BaseModel):
    session_id: str
    cf_session_id: str
    sdp_answer: str
    ice_servers: list[dict]


class LiveKitSessionResponse(BaseModel):
    """Robot create / operator join response for the LiveKit backend."""

    session_id: str
    transport: str = "livekit"
    url: str
    token: str
    room: str
    role: str | None = None  # set on operator join, omitted on robot create


class JoinSessionRequest(BaseModel):
    role: str = "operator"  # operator | viewer
    # Required for cloudflare; unused for livekit (see CreateSessionRequest).
    sdp_offer: str | None = None


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
    transport: str = "cloudflare"  # so the operator app picks the right client
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


async def _create_livekit_session(
    body: CreateSessionRequest,
    owner_id: str,
    robot_id: str,
    db: AsyncSession,
) -> LiveKitSessionResponse:
    """Robot create for LiveKit: persist the row, mint the robot's publish token.
    No SDP/CF round-trip.

    Id assigned up front so the room name (derived from it) is known before the
    insert. Mint before commit so a failed mint never persists an unusable row."""
    if not settings.livekit_configured:
        raise HTTPException(status_code=503, detail="LiveKit backend not configured")

    session = TeleopSession(
        id=str(uuid.uuid4()),
        robot_id=robot_id,
        owner_id=owner_id,
        robot_name=body.robot_name,
        state="idle",
        transport="livekit",
    )
    room = livekit.room_name(session.id)

    try:
        token = livekit.mint_token(
            identity=f"robot-{session.id}",
            name=body.robot_name,
            room=room,
            can_publish=True,
        )
    except LiveKitError as e:
        raise HTTPException(status_code=503, detail=str(e))

    db.add(session)
    await db.commit()

    return LiveKitSessionResponse(
        session_id=session.id,
        url=settings.livekit_url,
        token=token,
        room=room,
    )


@router.post(
    "",
    response_model=CreateSessionResponse | LiveKitSessionResponse,
    status_code=201,
)
async def create_session(
    body: CreateSessionRequest,
    owner_id: str = Depends(get_robot_owner),
    db: AsyncSession = Depends(get_db),
):
    """Robot registers itself. Creates a backend session (Cloudflare SFU or
    LiveKit room, per ``body.transport``).

    owner_id (the API key's owner) is the tenant boundary. robot_id is a
    robot-supplied label distinguishing multiple robots under one key; empty
    is fine (the session is still unique by id), it just disables reconnect
    dedup below.
    """
    robot_id = body.robot_id or ""

    # Same robot reconnecting → close its stale session. Scoped to (owner,
    # robot_id) so one robot can't disconnect another's; skipped for unnamed
    # robots (would collapse distinct ones). Transport-agnostic.
    if robot_id:
        existing = await db.execute(
            select(TeleopSession).where(
                TeleopSession.owner_id == owner_id,
                TeleopSession.robot_id == robot_id,
                TeleopSession.state != "disconnected",
            )
        )
        for old in existing.scalars():
            old.state = "disconnected"

    if body.transport == "livekit":
        return await _create_livekit_session(body, owner_id, robot_id, db)

    # transport is a validated Literal, so anything here is "cloudflare".
    if not body.sdp_offer:
        raise HTTPException(status_code=422, detail="sdp_offer required for cloudflare transport")

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
        owner_id=owner_id,
        robot_name=body.robot_name,
        state="idle",
        cf_session_id=cf_result["cf_session_id"],
        published_video_mid=published_mid,
        published_video_track_name=published_track_name,
    )
    db.add(session)
    try:
        await db.commit()
        await db.refresh(session)
    except Exception:
        # The CF session is already minted; commit failed (constraint, disk,
        # ...). CF has no "delete session" — sessions GC only when all tracks
        # close. The robot's PC will tear down on the 502, which kills the
        # transport, but the SFU may keep session metadata for a while. Log
        # so the leak is auditable.
        log.exception(
            "DB commit failed after CF session create — leaking cf_session=%s "
            "robot=%s owner=%s",
            cf_result["cf_session_id"], robot_id, owner_id,
        )
        raise HTTPException(status_code=502, detail="Session persist failed")

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
    owner_id: str = Depends(get_robot_owner),
    db: AsyncSession = Depends(get_db),
):
    """Robot reports connection quality metrics."""
    session = await db.get(TeleopSession, session_id)
    if not session or session.owner_id != owner_id:
        raise HTTPException(status_code=404, detail="Session not found")

    session.rtt_ms = body.rtt_ms
    session.packet_loss_pct = body.packet_loss_pct
    session.video_bitrate_kbps = body.video_bitrate_kbps
    session.command_rate_hz = body.command_rate_hz
    session.last_heartbeat = datetime.now(timezone.utc)
    await db.commit()

    # LiveKit robots learn operator presence from room events directly, so there
    # are no SCTP ids to surface — heartbeat is metrics/liveness only.
    if session.transport == "livekit":
        return {"ack": True}

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
    owner_id: str = Depends(get_robot_owner),
    db: AsyncSession = Depends(get_db),
):
    """Robot going offline."""
    session = await db.get(TeleopSession, session_id)
    if not session or session.owner_id != owner_id:
        raise HTTPException(status_code=404, detail="Session not found")

    # Lock against a concurrent bridge — without it, the bridge could write
    # _robot_channel_ids[session_id] AFTER we pop, leaving stale ids visible
    # via heartbeat for a session that's already marked disconnected.
    async with _session_lock(session_id):
        # Close the robot's state_reliable_back push so a future reconnect
        # under the same robot doesn't hit repeated_local_track. The operator's
        # local pushes on the operator CF session die with the operator's PC
        # tearing down. CF has no /sessions delete; both CF sessions stay
        # registered until the SFU's track-GC reaps them — log so an operator
        # can clean up manually if needed.
        if session.transport == "cloudflare":
            if session.state_back_channel_id is not None and session.cf_session_id:
                await cf_client.close_datachannels(
                    session.cf_session_id, [session.state_back_channel_id]
                )
            if session.cf_session_id or session.operator_cf_session_id:
                log.info(
                    "delete_session: orphaning CF sessions (no delete API) "
                    "robot_cf=%s operator_cf=%s",
                    session.cf_session_id, session.operator_cf_session_id,
                )
        session.state = "disconnected"
        # Clear operator binding so any orphaned operator client gets a clean
        # 404/409 on subsequent calls instead of seeing themselves still bound.
        session.operator_id = None
        session.operator_cf_session_id = None
        session.state_back_channel_id = None
        _robot_channel_ids.pop(session_id, None)
        _pending_video_renegotiations.discard(session_id)
        await db.commit()


# ─── Operator endpoints ──────────────────────────────────────────────


def _owns(session: TeleopSession, user: dict) -> bool:
    """Operator may touch only their own robots (admin sees all)."""
    return user.get("role") == "admin" or session.owner_id == user["sub"]


@router.get("", response_model=list[SessionInfo])
async def list_sessions(
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """List available robots (active sessions) the caller owns."""
    q = select(TeleopSession).where(TeleopSession.state.in_(["idle", "active"]))
    if user.get("role") != "admin":
        q = q.where(TeleopSession.owner_id == user["sub"])
    result = await db.execute(q)
    sessions = result.scalars().all()
    return [
        SessionInfo(
            session_id=s.id,
            robot_id=s.robot_id,
            robot_name=s.robot_name,
            state=s.state,
            transport=s.transport,
            operator_id=s.operator_id,
            rtt_ms=s.rtt_ms,
            packet_loss_pct=s.packet_loss_pct,
            created_at=s.created_at,
        )
        for s in sessions
    ]


async def _claim_operator_slot(db: AsyncSession, session_id: str, user_id: str) -> bool:
    """Atomically take the operator slot on an active session.

    Returns True if this caller now owns it (or already did — same user
    re-joining is idempotent), False if another operator holds it or the row
    has gone disconnected. The portable single-UPDATE pattern avoids the
    read-then-write race where two concurrent /join calls both see the slot
    free and both create transport-layer sessions (CF/LiveKit) that leak.
    """
    stmt = (
        update(TeleopSession)
        .where(
            TeleopSession.id == session_id,
            TeleopSession.state != "disconnected",
            or_(
                TeleopSession.operator_id.is_(None),
                TeleopSession.operator_id == user_id,
            ),
        )
        .values(operator_id=user_id, state="active")
    )
    result = await db.execute(stmt)
    await db.commit()
    return result.rowcount > 0


async def _release_operator_slot(db: AsyncSession, session_id: str, user_id: str) -> None:
    """Undo a claim when post-claim work (CF create / LiveKit mint) fails.

    Guarded by the user_id in the WHERE so we never accidentally evict a
    different operator who somehow ended up bound (defensive — shouldn't be
    possible given the claim ordering)."""
    await db.execute(
        update(TeleopSession)
        .where(
            TeleopSession.id == session_id,
            TeleopSession.operator_id == user_id,
        )
        .values(operator_id=None, state="idle")
    )
    await db.commit()


@router.post(
    "/{session_id}/join",
    response_model=JoinSessionResponse | LiveKitSessionResponse,
)
async def join_session(
    session_id: str,
    body: JoinSessionRequest,
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """Operator or viewer joins a session."""
    session = await db.get(TeleopSession, session_id)
    if not session or session.state == "disconnected" or not _owns(session, user):
        raise HTTPException(status_code=404, detail="Session not found")

    user_id = user["sub"]

    # Operator role: take the slot atomically BEFORE any transport-layer work,
    # so a losing concurrent /join never creates a CF/LiveKit session it can't
    # use. Viewers don't claim and don't need ownership.
    if body.role == "operator":
        if not await _claim_operator_slot(db, session_id, user_id):
            # Re-read to disambiguate 404 (row gone/disconnected) from 409
            # (another operator owns it).
            current = await db.get(TeleopSession, session_id)
            if not current or current.state == "disconnected":
                raise HTTPException(status_code=404, detail="Session not found")
            raise HTTPException(
                status_code=409,
                detail=f"Session already has operator: {current.operator_id}",
            )
        # The claim persisted operator_id/state; reflect that on the in-handler
        # row so downstream code (and the final return) sees fresh values.
        await db.refresh(session)

    if session.transport == "livekit":
        if not settings.livekit_configured:
            if body.role == "operator":
                await _release_operator_slot(db, session_id, user_id)
            raise HTTPException(status_code=503, detail="LiveKit backend not configured")
        room = livekit.room_name(session.id)
        try:
            token = livekit.mint_token(
                identity=f"op-{user_id}",
                name=user_id,
                room=room,
                can_publish=False,  # operator drives via data; no media uplink
            )
        except LiveKitError as e:
            if body.role == "operator":
                await _release_operator_slot(db, session_id, user_id)
            raise HTTPException(status_code=503, detail=str(e))
        return LiveKitSessionResponse(
            session_id=session.id,
            url=settings.livekit_url,
            token=token,
            room=room,
            role=body.role,
        )

    if not body.sdp_offer:
        if body.role == "operator":
            await _release_operator_slot(db, session_id, user_id)
        raise HTTPException(status_code=422, detail="sdp_offer required for cloudflare transport")

    # Join datachannels-clean (no video track here). Video is pulled after the
    # bridge, once the operator PC is connected — see bridge_datachannel.
    try:
        cf_result = await cf_client.create_session(body.sdp_offer)
    except CloudflareRealtimeError as e:
        if body.role == "operator":
            await _release_operator_slot(db, session_id, user_id)
        raise HTTPException(status_code=502, detail=f"Cloudflare error: {e.detail}")
    except Exception as e:
        if body.role == "operator":
            await _release_operator_slot(db, session_id, user_id)
        raise HTTPException(
            status_code=502,
            detail=f"Cloudflare session create failed ({type(e).__name__}): {e}",
        )

    operator_cf_id = cf_result["cf_session_id"]

    if body.role == "operator":
        session.operator_cf_session_id = operator_cf_id
        try:
            await db.commit()
        except Exception:
            # The operator's CF session is already minted; commit failed. Log
            # so the leak is auditable, then release the operator slot and
            # surface a 502 — the client will retry from scratch.
            log.exception(
                "DB commit failed after operator CF session create — leaking "
                "cf_session=%s session=%s operator=%s",
                operator_cf_id, session_id, user_id,
            )
            await _release_operator_slot(db, session_id, user_id)
            raise HTTPException(status_code=502, detail="Join persist failed")

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
    if session.transport != "cloudflare":
        raise HTTPException(status_code=409, detail="bridge-datachannel is cloudflare-only")
    if session.operator_id != user["sub"]:
        raise HTTPException(status_code=403, detail="Not the bound operator")
    if not session.operator_cf_session_id or not session.cf_session_id:
        raise HTTPException(status_code=409, detail="CF sessions not ready")

    # Serialize per-session: prevents the close-then-republish race on
    # state_reliable_back when two bridges run concurrently for the same
    # session (operator double-click, retry). Without it both bridges close
    # the stale push id, both repush, the second hits repeated_local_track
    # and leaves the session permanently un-bridgeable.
    async with _session_lock(session_id):
        return await _bridge_datachannel_locked(session, db)


async def _bridge_datachannel_locked(
    session: TeleopSession, db: AsyncSession
) -> BridgeDatachannelResponse:
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
    # Track every local push we successfully created so far. If a later step
    # (next add_datachannels, parsing, or the missing-name check) fails, close
    # them — otherwise the next reconnect re-pushes under the same names and
    # hits repeated_local_track_error. Remote subscriptions are not tracked:
    # they're cheap and don't block reconnects.
    created_pushes: list[tuple[str, list[int]]] = []

    async def _rollback_pushes() -> None:
        for sid, ids in created_pushes:
            await cf_client.close_datachannels(sid, ids)

    try:
        # operator → robot: cmd + state. Operator publishes, robot subscribes.
        op_pub = await cf_client.add_datachannels(
            session.operator_cf_session_id,
            [{"location": "local", "dataChannelName": name} for name in forward_names],
        )
        op_pub_ids = {e["dataChannelName"]: int(e["id"]) for e in op_pub}
        created_pushes.append((session.operator_cf_session_id, list(op_pub_ids.values())))

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
        robot_sub_ids = {e["dataChannelName"]: int(e["id"]) for e in robot_sub}

        # robot → operator: state_back. Fresh push each connect (stale one
        # closed above); operator subscribes to it.
        robot_pub = await cf_client.add_datachannels(
            session.cf_session_id,
            [{"location": "local", "dataChannelName": STATE_BACK_CHANNEL_NAME}],
        )
        robot_pub_ids = {e["dataChannelName"]: int(e["id"]) for e in robot_pub}
        created_pushes.append((session.cf_session_id, list(robot_pub_ids.values())))

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
        op_sub_ids = {e["dataChannelName"]: int(e["id"]) for e in op_sub}
    except CloudflareRealtimeError as e:
        await _rollback_pushes()
        raise HTTPException(
            status_code=502,
            detail=f"Cloudflare datachannel bridge failed: {e.detail}",
        )
    except (KeyError, TypeError, ValueError) as e:
        await _rollback_pushes()
        raise HTTPException(
            status_code=502,
            detail=f"Cloudflare returned malformed DataChannel entry: {e}",
        )
    except Exception as e:
        await _rollback_pushes()
        raise HTTPException(
            status_code=502,
            detail=f"Datachannel bridge failed ({type(e).__name__}): {e}",
        )

    missing = [n for n in forward_names if n not in op_pub_ids or n not in robot_sub_ids]
    if STATE_BACK_CHANNEL_NAME not in robot_pub_ids or STATE_BACK_CHANNEL_NAME not in op_sub_ids:
        missing.append(STATE_BACK_CHANNEL_NAME)
    if missing:
        await _rollback_pushes()
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
    # Track the pending renegotiation so a stale/forged answer can't reach CF
    # via /renegotiate-answer; clear any leftover from a prior bridge.
    if video_offer:
        _pending_video_renegotiations.add(session.id)
    else:
        _pending_video_renegotiations.discard(session.id)

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
    if session.transport != "cloudflare":
        raise HTTPException(status_code=409, detail="renegotiate-answer is cloudflare-only")
    if session.operator_id != user["sub"]:
        raise HTTPException(status_code=403, detail="Not the bound operator")
    if not session.operator_cf_session_id:
        raise HTTPException(status_code=409, detail="Operator CF session not ready")
    if session_id not in _pending_video_renegotiations:
        raise HTTPException(
            status_code=409,
            detail="No pending video renegotiation — re-bridge to get a fresh offer",
        )

    # Consume the pending marker either way: a failed renegotiate shouldn't
    # let the client retry the same stale answer (CF would reject again).
    _pending_video_renegotiations.discard(session_id)
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
    if not session or not _owns(session, user):
        raise HTTPException(status_code=404, detail="Session not found")

    user_id = user["sub"]

    if session.operator_id == user_id:
        # Same rationale as delete_session: serialize with bridges.
        async with _session_lock(session_id):
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
    if not session or not _owns(session, user):
        raise HTTPException(status_code=404, detail="Session not found")

    return SessionInfo(
        session_id=session.id,
        robot_id=session.robot_id,
        robot_name=session.robot_name,
        state=session.state,
        transport=session.transport,
        operator_id=session.operator_id,
        rtt_ms=session.rtt_ms,
        packet_loss_pct=session.packet_loss_pct,
        created_at=session.created_at,
    )
