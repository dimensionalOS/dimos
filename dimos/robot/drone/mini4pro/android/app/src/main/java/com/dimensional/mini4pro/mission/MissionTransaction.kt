package com.dimensional.mini4pro.mission

import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavFrame
import io.dronefleet.mavlink.common.MavMissionResult
import io.dronefleet.mavlink.common.MavMissionType
import io.dronefleet.mavlink.common.MavResult
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.MissionAck
import io.dronefleet.mavlink.common.MissionClearAll
import io.dronefleet.mavlink.common.MissionCount
import io.dronefleet.mavlink.common.MissionItem
import io.dronefleet.mavlink.common.MissionItemInt
import io.dronefleet.mavlink.common.MissionRequest
import io.dronefleet.mavlink.common.MissionRequestInt
import io.dronefleet.mavlink.common.MissionRequestList
import io.dronefleet.mavlink.common.MissionRequestPartialList
import io.dronefleet.mavlink.common.MissionSetCurrent
import io.dronefleet.mavlink.common.MissionWritePartialList
import io.dronefleet.mavlink.common.Statustext
import io.dronefleet.mavlink.util.EnumValue

/**
 * **One owner, one state machine, one lock** for every mission-protocol message in both
 * directions.
 *
 * Runs on the `mavlink-rx` thread, like `HandshakeResponder` and `CommandDispatcher`, and must
 * return quickly: during a download QGroundControl gives us **250 ms** to answer each
 * `MISSION_REQUEST_INT` before it re-asks (`PlanManager.h:61`), and the phone's inbound path is
 * the same thread that carries telemetry. Answering from a pre-built list in memory, on the
 * receiving thread, is therefore not an optimisation — it is the design.
 *
 * ```
 * IDLE ──MISSION_COUNT──────────► RECEIVING ──all items valid──► IDLE (store committed, ACK ACCEPTED)
 *   │                                 │
 *   │                                 └──any refusal / timeout──► IDLE (pending discarded, ACK <reason>)
 *   │
 *   ├──MISSION_REQUEST_LIST────► (reading) ──MISSION_ACK from QGC──► (idle)
 *   │                                      └──idle timeout─────────► (idle, silently)
 *   │
 *   └──MISSION_CLEAR_ALL──────► IDLE (store cleared, ACK ACCEPTED)
 * ```
 *
 * ## Why the read is a flag rather than a phase
 *
 * The design document draws upload and download as two mutually exclusive phases, and for the
 * *upload* direction that is exactly right. But §2.4's fourth row requires that a
 * `MISSION_REQUEST_LIST` arriving mid-upload be **answered from the committed store without
 * disturbing the pending buffer** — a read is non-destructive and the committed store is a true
 * answer to it, and this genuinely happens because QGC's Fly and Plan views both hold a
 * `PlanMasterController`. A read that could evict `RECEIVING` would break that rule. So a read is
 * never a phase; it is [download], a snapshot plus two timestamps that live alongside whatever the
 * upload machine is doing.
 *
 * ## A download answers from the plan it began with (§2.4 row 3)
 *
 * `MISSION_COUNT` and the `MISSION_ITEM_INT`s that follow it are **one answer delivered in
 * instalments**, and until 2026-07-27 each instalment was served from [store] at its own instant.
 * A commit landing between item 2 and item 3 therefore handed QGC items 0–2 of the old plan and
 * 3–4 of the new one: a plan neither end ever held, assembled without a single error, shown to the
 * operator as theirs. A *shorter* new plan produced an `INVALID_SEQUENCE` in the middle of a read
 * instead.
 *
 * So the count is computed from an immutable [MissionPlan] snapshot and that snapshot is what
 * every subsequent request is served from — the store already hands out immutable snapshots, so
 * the whole fix is holding a reference. The read's own idle timeout is what ends it, and a request
 * beyond the snapshot's range is `MAV_MISSION_INVALID_SEQUENCE`, which is the true answer: the
 * plan that read was reading has that many items and no more.
 *
 * The corollary is that a `MISSION_COUNT` **no longer cancels a download in flight**. It cancels
 * the *upload* machine's view of the world, which is what §2.4's "the write is the newer intent"
 * was about; a reader mid-instalment is a different conversation with a different ground station
 * (or a different QGC view), and dropping its snapshot would put the splice straight back.
 *
 * ## Nothing partial ever reaches the store (§2.2)
 *
 * An upload accumulates into [pending] and the store is replaced at exactly one instant: after
 * the last item has arrived, after the whole-plan check has passed, and before
 * `MISSION_ACK(ACCEPTED)` goes out. The reason is not tidiness — the execution half may be
 * reading the store on its own 10 Hz thread, and a half-written plan is a plan with a
 * discontinuity in it that an aircraft would fly. *"Refuse to start"* is a policy; *"cannot be in
 * that state"* is a property, and this layer can afford the property.
 *
 * The corollary is what makes a failed upload safe: **a failed upload leaves the previous plan
 * exactly as it was.** QGC discards its own half on the same edge
 * (`_clearAndDeleteWriteMissionItems`, `:843`), so after a refusal both ends agree that the old
 * plan is the plan — a state an operator can act on, unlike "one of us has half a plan".
 *
 * ## Our own timeouts, and why they are asymmetric (§2.3)
 *
 * QGC's retry behaviour differs by *where* the silence falls. The row that decides our design:
 * once we have requested item 0, QGC does **not** retry a stalled upload — it fails with
 * *"Vehicle did not request all items from ground station"* (`:217`). So an abandoned transaction
 * that held `RECEIVING` forever would make the next upload collide with a ghost, and our own
 * timeout must end in an explicit `MISSION_ACK`, never in silence.
 *
 * | our timer | value | on expiry |
 * |---|---|---|
 * | RECEIVING: no item for | [ITEM_TIMEOUT_MS] | discard pending, `MISSION_ACK(OPERATION_CANCELLED)`, `STATUSTEXT`, → IDLE |
 * | RECEIVING: whole transaction | [TRANSACTION_TIMEOUT_MS] | as above |
 * | reading: no request and no closing ack for | [READ_IDLE_TIMEOUT_MS] | → idle, **no message** (QGC has its own error path and a second one from us is noise) |
 *
 * ## What replaced `HandshakeResponder`'s mission branches
 *
 * All five of them, **deleted rather than extended** (JC-10). They existed to make an absent
 * feature legible; once the feature exists they are dead code that would answer first, and a
 * second place mission messages are handled is the shape of a bug that only appears after a
 * reconnect.
 */
class MissionTransaction(
    private val store: MissionStore,
    /** Must be safe to call from the receiving thread — `Bridge.sendOffMain` is. */
    private val send: (Any) -> Unit,
    /**
     * The command interlock. §2.5 and M2 §Q2: with commands off, the answer must be **the answer
     * that existed before the feature**.
     */
    private val interlockEnabled: () -> Boolean,
    /** The execution half, or [NoMissionExecution] while it does not exist. */
    private val execution: () -> MissionExecution = { NoMissionExecution },
    /** DJI's home at the instant of commit, or null. Provenance only — never used as a datum. */
    private val homeAtUpload: () -> GeoPoint? = { null },
    /** `TelemetryEncoder.amslMetres` at the instant of commit. Provenance only. */
    private val amslDatumAtUpload: () -> Double? = { null },
    private val log: (String) -> Unit = {},
    /**
     * **Monotonic** clock, injected so every timeout is testable without sleeping.
     *
     * No default, deliberately. It used to default to `System.currentTimeMillis()` beside a KDoc
     * that said monotonic — a wall clock that steps when NTP corrects it or the operator changes
     * time zone, and every timer here is a difference of two readings of it. `Bridge` passes
     * `SystemClock.elapsedRealtime()` and always did, so the default was never anything but a trap
     * laid for the next caller. Making it required is one character shorter than making it honest.
     */
    private val nowMs: () -> Long,
    /** Our own ids, used only to decide whether a targeted message is for us. */
    private val systemId: Int = 1,
    private val componentId: Int = 1,
) {

    companion object {
        /**
         * No `MISSION_ITEM_INT` for this long ends the upload. ~3× QGC's own 1500 ms passive ack
         * timeout, the same "3× the measured round trip" reasoning as `guided/` JC-6's
         * engage-confirm.
         */
        const val ITEM_TIMEOUT_MS = 5_000L

        /**
         * A whole upload may not take longer than this. Matches M3's manoeuvre timeout; a plan
         * that takes a minute to upload over this link has a different problem.
         */
        const val TRANSACTION_TIMEOUT_MS = 60_000L

        /**
         * A download with no request and no closing `MISSION_ACK` for this long is abandoned
         * silently. Silence rather than a message because QGC already has its own error path for
         * it, and a second complaint from us is noise.
         */
        const val READ_IDLE_TIMEOUT_MS = 10_000L

        /** `MAV_COMP_ID_ALL`; some ground stations broadcast mission messages to it. */
        private const val COMPONENT_ID_ALL = 0

        private const val DEFAULT_GCS_SYSTEM_ID = 255
        private const val DEFAULT_GCS_COMPONENT_ID = 190
    }

    private enum class Phase { IDLE, RECEIVING }

    private val lock = Any()

    private var phase = Phase.IDLE
    private var pending: Array<StoredItem?> = emptyArray()
    private var expectedCount = 0
    private var expectedSeq = 0
    private var receivedCount = 0
    private var lastItemAtMs = 0L
    private var transactionStartedAtMs = 0L
    private var uploadTargetSystem = DEFAULT_GCS_SYSTEM_ID
    private var uploadTargetComponent = DEFAULT_GCS_COMPONENT_ID

    /**
     * One download in flight: the items `MISSION_COUNT` was computed from, and the two timestamps
     * the idle timeout runs on.
     *
     * [items] is the whole point — see the class KDoc. It is an immutable list taken straight off
     * a [MissionPlan], so holding it costs a reference and buys the property that a read cannot
     * observe half of one plan and half of another. [planId] is carried only so the log can say
     * *why* a request went out of range.
     */
    private class Download(
        val items: List<StoredItem>,
        val planId: Int,
        val startedAtMs: Long,
        var lastActivityMs: Long,
    )

    /** Non-null while a download is believed to be in progress. */
    private var download: Download? = null

    /** JC-3's "once per link session". Reset by [reset], which `Bridge.start` calls. */
    private var clearAnnounced = false

    /** For the status screen and the tests: whether an upload is part-way through. */
    val isReceiving: Boolean get() = synchronized(lock) { phase == Phase.RECEIVING }

    /** How many items of the current upload have arrived. Zero when idle. */
    val received: Int get() = synchronized(lock) { receivedCount }

    /**
     * Drops any in-flight transaction and the per-session announcement state.
     *
     * Called when a link opens. **It does not touch the store** — that is the point of §3.2: QGC
     * re-reads the plan on every connect, and a plan that died with the socket would show an
     * empty Plan view for an aircraft that is flying one.
     */
    fun reset() {
        synchronized(lock) {
            discardPending()
            download = null
            clearAnnounced = false
        }
    }

    /**
     * Routes one inbound payload. Returns true if this object owned it, purely so the caller can
     * log honestly; nothing depends on the value.
     */
    fun onInbound(
        payload: Any,
        senderSystemId: Int = DEFAULT_GCS_SYSTEM_ID,
        senderComponentId: Int = DEFAULT_GCS_COMPONENT_ID,
    ): Boolean = when (payload) {
        is MissionRequestList -> {
            if (addressedToUs(payload.targetSystem(), payload.targetComponent())) {
                onRequestList(payload.missionType().value(), senderSystemId, senderComponentId)
            }
            true
        }

        is MissionRequestInt -> {
            if (addressedToUs(payload.targetSystem(), payload.targetComponent())) {
                onRequestItem(payload.seq(), payload.missionType().value(), senderSystemId, senderComponentId)
            }
            true
        }

        // Field-identical to MISSION_REQUEST_INT, and QGC decodes both as mission_request_int
        // (`PlanManager.cc:655-658`). Accepted for the ground stations that still send #40.
        is MissionRequest -> {
            if (addressedToUs(payload.targetSystem(), payload.targetComponent())) {
                onRequestItem(payload.seq(), payload.missionType().value(), senderSystemId, senderComponentId)
            }
            true
        }

        is MissionCount -> {
            if (addressedToUs(payload.targetSystem(), payload.targetComponent())) {
                onCount(payload.count(), payload.missionType().value(), senderSystemId, senderComponentId)
            }
            true
        }

        is MissionItemInt -> {
            if (addressedToUs(payload.targetSystem(), payload.targetComponent())) {
                onItem(payload, senderSystemId, senderComponentId)
            }
            true
        }

        is MissionItem -> {
            if (addressedToUs(payload.targetSystem(), payload.targetComponent())) {
                onFloatItem(payload.missionType().value(), senderSystemId, senderComponentId)
            }
            true
        }

        is MissionClearAll -> {
            if (addressedToUs(payload.targetSystem(), payload.targetComponent())) {
                onClearAll(payload.missionType().value(), senderSystemId, senderComponentId)
            }
            true
        }

        is MissionSetCurrent -> {
            if (addressedToUs(payload.targetSystem(), payload.targetComponent())) {
                onSetCurrent(payload.seq())
            }
            true
        }

        // §1.5: dead protocol for this ground station — `grep -rn "write_partial|request_partial"`
        // over the whole QGC tree returns nothing outside MockLink. Another GCS could send them,
        // and the design is the one this project already uses for REQUEST_DATA_STREAM: answer,
        // honestly, and do not implement. Not silence, which makes a sender retry into a timeout
        // and blame the link; not MAV_MISSION_ERROR, because "unspecified error" says less than
        // "we do not do this".
        is MissionWritePartialList -> {
            if (addressedToUs(payload.targetSystem(), payload.targetComponent())) {
                ack(MavMissionResult.MAV_MISSION_UNSUPPORTED, payload.missionType().value(),
                    senderSystemId, senderComponentId)
            }
            true
        }

        is MissionRequestPartialList -> {
            if (addressedToUs(payload.targetSystem(), payload.targetComponent())) {
                ack(MavMissionResult.MAV_MISSION_UNSUPPORTED, payload.missionType().value(),
                    senderSystemId, senderComponentId)
            }
            true
        }

        // §1.4: an inbound MISSION_ACK is a normal event, not a transaction start — it is how the
        // *reader* says it is done. Today's HandshakeResponder has no branch for it at all and it
        // falls to `else -> Unit`, which is accidentally correct; here it is deliberate.
        is MissionAck -> {
            synchronized(lock) { download = null }
            true
        }

        else -> false
    }

    /**
     * Runs the timeouts. Called from `Bridge.tick`; safe to call at any rate, and cheap when
     * nothing is in flight.
     */
    fun tick() {
        val now = nowMs()
        synchronized(lock) {
            download?.let { read ->
                val idle = now - read.lastActivityMs
                val total = now - read.startedAtMs
                if (idle >= READ_IDLE_TIMEOUT_MS || total >= READ_IDLE_TIMEOUT_MS + TRANSACTION_TIMEOUT_MS) {
                    log("mission read abandoned after ${idle}ms idle — no message sent")
                    download = null
                }
            }

            if (phase != Phase.RECEIVING) return
            val sinceItem = now - lastItemAtMs
            val sinceStart = now - transactionStartedAtMs
            if (sinceItem < ITEM_TIMEOUT_MS && sinceStart < TRANSACTION_TIMEOUT_MS) return

            val got = receivedCount
            val wanted = expectedCount
            val target = uploadTargetSystem
            val component = uploadTargetComponent
            discardPending()
            log("mission upload abandoned: $got of $wanted items after ${sinceItem}ms idle")
            // MAV_MISSION_OPERATION_CANCELLED is the honest code: nobody refused anything, the
            // conversation stopped. QGC's _missionResultToString has no case for it and prints
            // "Unknown error: 15" (`:791`) — ugly, and still better than a lie. The STATUSTEXT
            // beside it carries the real sentence, including that the old plan is untouched.
            ack(MavMissionResult.MAV_MISSION_OPERATION_CANCELLED, MissionTypes.MISSION, target, component)
            announce(MissionStatusTexts.uploadAbandoned(got, wanted))
        }
    }

    // ------------------------------------------------------------------------------ download

    /**
     * A read is **always allowed**, in every state (§2.5), and it reports the store honestly even
     * with the interlock off — JC-1.
     *
     * M2 §Q2 says the safe state must be the pre-feature behaviour, which for an upload was
     * `MAV_MISSION_UNSUPPORTED` and for a read was `MISSION_COUNT 0`. The refusals are kept byte
     * for byte; the read is not. §Q2 exists so that switching commands off cannot leave a
     * half-armed actuating path, and **a read actuates nothing**. Reporting zero while we hold a
     * plan would be a lie told to make a rule easier to state — and the operator most likely to
     * have the interlock off is exactly the one checking what the bridge is holding.
     */
    private fun onRequestList(missionType: Int, senderSystem: Int, senderComponent: Int) {
        // The snapshot is taken *here*, once, and every item of this download comes out of it.
        // One read of `store.plan()`, not one per instalment — see the class KDoc.
        val plan = if (missionType == MissionTypes.MISSION) store.plan() else null
        val items = plan?.items ?: emptyList()
        val count = items.size
        val now = nowMs()
        synchronized(lock) {
            download = Download(
                items = items,
                planId = plan?.planId ?: 0,
                startedAtMs = now,
                lastActivityMs = now,
            )
        }
        log("MISSION_REQUEST_LIST type=$missionType → count $count (planId ${plan?.planId ?: 0})")
        send(
            MissionCount.builder()
                .targetSystem(senderSystem)
                .targetComponent(senderComponent)
                .count(count)
                // Echoing the type is not politeness: QGC checks it on every inbound message and
                // silently drops a mismatch (`PlanManager.cc:323`, `:411`, `:497`, `:561`), so it
                // is the difference between a reply and silence.
                .missionType(missionTypeValue(missionType))
                .build()
        )
    }

    private fun onRequestItem(seq: Int, missionType: Int, senderSystem: Int, senderComponent: Int) {
        val read = synchronized(lock) { download?.also { it.lastActivityMs = nowMs() } }
        val items = when {
            missionType != MissionTypes.MISSION -> emptyList()
            // The instalment path: served from the plan this download began with, whatever the
            // store holds now. This is the whole of the fix.
            read != null -> read.items
            // A request with no `MISSION_COUNT` of ours in front of it. There is no snapshot to be
            // consistent with, so the store at this instant is the only answer that exists — and
            // one item is not a plan, so there is nothing to splice.
            else -> store.items
        }
        val item = items.getOrNull(seq)
        if (item == null) {
            // Includes the case the snapshot exists to make legible: the plan moved on under a
            // download and the reader asked for an item the plan it is reading never had. Saying
            // INVALID_SEQUENCE is true of that read; serving item `seq` of the *new* plan would
            // not be true of anything.
            val movedOn = read != null && read.planId != store.planId
            log(
                "MISSION_REQUEST_INT seq=$seq out of range (${items.size} items)" +
                    if (movedOn) " — plan changed from ${read.planId} to ${store.planId} mid-read" else ""
            )
            ack(MavMissionResult.MAV_MISSION_INVALID_SEQUENCE, missionType, senderSystem, senderComponent)
            return
        }
        send(itemMessage(item, senderSystem, senderComponent))
    }

    /**
     * The stored item, back on the wire **with the bytes QGC gave us**. Nothing is recomputed and
     * nothing is normalised — see [StoredItem]. A read-back that paraphrased the plan would show
     * an operator something other than what they authored.
     */
    private fun itemMessage(item: StoredItem, senderSystem: Int, senderComponent: Int): MissionItemInt =
        MissionItemInt.builder()
            .targetSystem(senderSystem)
            .targetComponent(senderComponent)
            .seq(item.seq)
            .frame(EnumValue.create(MavFrame::class.java, item.frame))
            .command(EnumValue.create(MavCmd::class.java, item.command))
            .current(item.current)
            .autocontinue(item.autocontinue)
            .param1(item.param1)
            .param2(item.param2)
            .param3(item.param3)
            .param4(item.param4)
            .x(item.x)
            .y(item.y)
            .z(item.z)
            .missionType(missionTypeValue(item.missionType))
            .build()

    // -------------------------------------------------------------------------------- upload

    private fun onCount(count: Int, missionType: Int, senderSystem: Int, senderComponent: Int) {
        if (missionType != MissionTypes.MISSION) {
            // Fence and rally have no capability bit, so QGC skips those connect states entirely
            // and this can only be a foreign ground station. Refused, never stored.
            log("mission upload of type $missionType refused — we hold only plain missions")
            ack(MavMissionResult.MAV_MISSION_UNSUPPORTED, missionType, senderSystem, senderComponent)
            return
        }

        // Interlock first, and that order is deliberate: M2 §Q2's rule is that with commands off
        // the answer is the one that existed before the feature, and that must not depend on what
        // any other component happens to be doing.
        //
        // **It is consulted here and nowhere else in the upload, and that is a decision, not an
        // oversight.** Switching the interlock off mid-upload lets the upload run to completion:
        // the answer at the moment of asking was correct, and what follows is items arriving into
        // a buffer and a list being replaced in memory. Holding a plan commands nothing — the
        // interlock guards *actuation*, and the actuating door is Start, which is the execution
        // half's and checks the interlock at its own instant. Raised by the review of 2026-07-27
        // and settled as no change; recorded here so it is not raised a third time.
        if (!interlockEnabled()) {
            log("mission upload refused — command interlock off")
            ack(MavMissionResult.MAV_MISSION_UNSUPPORTED, missionType, senderSystem, senderComponent)
            return
        }

        // An upload of zero items is MAVLink's other way of saying "clear the plan", and it is
        // the only reading under which the transaction can complete at all: there is no item to
        // request, so there is nothing to wait for. Routed to exactly the same code as
        // MISSION_CLEAR_ALL — including its refusal while the executor is running — because
        // otherwise one operator intent would get two different answers depending on which
        // message QGC happened to choose.
        if (count <= 0) {
            clearPlan(missionType, senderSystem, senderComponent)
            return
        }

        // Note what is deliberately **absent** here: the executor-running veto. M4-12 overruled
        // it. An upload arriving mid-flight is accepted into the store, the running mission
        // continues on the snapshot it began with, and the new plan takes effect at the next
        // Start — announced, at the commit, with PLAN_STORED_FOR_NEXT_START. See that constant
        // for why the live-swap reading was not chosen.

        // The one check that does not wait for an item, because the count is the only failure
        // with no index to report — a 500-item survey is refused in one message rather than
        // after 500 round trips.
        MissionAdmission.countRefusal(count)?.let { refusal ->
            log("mission upload refused: ${refusal.reason}")
            ack(refusal.result, missionType, senderSystem, senderComponent)
            announce(refusal.reason)
            return
        }

        synchronized(lock) {
            if (phase == Phase.RECEIVING) {
                // §2.4 rows 1 and 2. QGC re-sends MISSION_COUNT when it saw no request from us
                // (`:205-214`), so a duplicate means it believes we heard nothing — and the only
                // state both ends agree on is the start. A *different* count has no reading under
                // which the old buffer is still valid, so it is the same answer.
                log("duplicate MISSION_COUNT while receiving — restarting from seq 0")
            }
            phase = Phase.RECEIVING
            pending = arrayOfNulls(count)
            expectedCount = count
            expectedSeq = 0
            receivedCount = 0
            lastItemAtMs = nowMs()
            transactionStartedAtMs = nowMs()
            uploadTargetSystem = senderSystem
            uploadTargetComponent = senderComponent
            // A write is the newer intent and is never refused because a download is open — that
            // is §2.4's rule and it is unchanged. What *is* changed (2026-07-27) is that the
            // download is no longer torn down here: a reader mid-instalment holds a snapshot, and
            // dropping it would drop that reader back onto the live store, which is exactly the
            // splice the snapshot exists to prevent. The read's own idle timeout ends it.
        }
        requestItem(0, senderSystem, senderComponent)
    }

    private fun requestItem(seq: Int, senderSystem: Int, senderComponent: Int) {
        // MISSION_REQUEST_INT (#51), not MISSION_REQUEST (#40): QGC accepts either and decodes
        // both as mission_request_int (`:492`, `:655-658`), and the int form is the one that
        // matches what it sends back.
        send(
            MissionRequestInt.builder()
                .targetSystem(senderSystem)
                .targetComponent(senderComponent)
                .seq(seq)
                .missionType(missionTypeValue(MissionTypes.MISSION))
                .build()
        )
    }

    /**
     * One `MISSION_ITEM_INT`, and **only from the ground station whose `MISSION_COUNT` opened this
     * transaction**.
     *
     * [senderSystem]/[senderComponent] are checked against the pair captured at [onCount] for the
     * same reason every other branch carries them: with two ground stations on the link, items
     * from A folded into B's transaction produce a well-formed plan **nobody authored** — every
     * item admissible, the count right, the ack sent, and a list of waypoints no operator ever put
     * in that order. There is no refusal that can be sent for it either, because the message was
     * not part of the conversation it would end; so it is dropped and logged, the same answer a
     * mission-type mismatch gets, and the real owner's own 5 s timer is what resolves the stall.
     */
    private fun onItem(payload: MissionItemInt, senderSystem: Int, senderComponent: Int) {
        val itemType = payload.missionType().value()
        val item = StoredItem(
            seq = payload.seq(),
            frame = payload.frame().value(),
            command = payload.command().value(),
            current = payload.current(),
            autocontinue = payload.autocontinue(),
            param1 = payload.param1(),
            param2 = payload.param2(),
            param3 = payload.param3(),
            param4 = payload.param4(),
            x = payload.x(),
            y = payload.y(),
            z = payload.z(),
            missionType = itemType,
        )

        var refusal: MissionRefusal? = null
        var complete: List<StoredItem>? = null
        var nextSeq: Int? = null
        var target = DEFAULT_GCS_SYSTEM_ID
        var component = DEFAULT_GCS_COMPONENT_ID

        synchronized(lock) {
            if (phase != Phase.RECEIVING) {
                log("MISSION_ITEM_INT seq=${item.seq} outside a transaction — dropped")
                return
            }
            // A mission-type mismatch is dropped, never answered: QGC does the same to us, and
            // answering would end a transaction over a message that was not part of it.
            if (itemType != MissionTypes.MISSION) {
                log("MISSION_ITEM_INT type=$itemType mid-upload — dropped")
                return
            }
            // Whose transaction is this? See the KDoc: a plan nobody authored is worse than a
            // stalled upload, and the stall has a timer.
            if (senderSystem != uploadTargetSystem || senderComponent != uploadTargetComponent) {
                log(
                    "MISSION_ITEM_INT seq=${item.seq} from $senderSystem/$senderComponent but the " +
                        "transaction belongs to $uploadTargetSystem/$uploadTargetComponent — dropped"
                )
                return
            }

            target = uploadTargetSystem
            component = uploadTargetComponent

            if (item.seq != expectedSeq) {
                // A duplicate is a retransmission, not a disagreement: accept the first, drop the
                // second, keep going — and treat it as evidence QGC is alive.
                if (item.seq in 0 until expectedCount && pending[item.seq] != null) {
                    lastItemAtMs = nowMs()
                    log("duplicate MISSION_ITEM_INT seq=${item.seq} — dropped, transaction continues")
                    return
                }
                // A sequence gap. Drop it, do **not** advance, do **not** ack, and let our own
                // 5 s timer run. This is the case MAV_MISSION_INVALID_SEQUENCE was invented for,
                // and under our PX4 identity sending it *kills the transaction* (`:568` — QGC
                // tolerates it only for ArduPilot). Dropping costs one retry cycle; nacking costs
                // the whole upload.
                log("MISSION_ITEM_INT seq=${item.seq} but expecting $expectedSeq — dropped, no ack")
                return
            }

            // JC-2: validate as it arrives, so the offending item's index is the last one we
            // requested and QGC names it, the command and the offending value in its own dialog.
            refusal = MissionAdmission.itemRefusal(item, expectedSeq, expectedCount)
            if (refusal != null) {
                // Ended here, inside the section that decided it — see below.
                discardPending()
            } else {
                pending[expectedSeq] = item
                receivedCount++
                expectedSeq++
                lastItemAtMs = nowMs()
                if (expectedSeq < expectedCount) {
                    nextSeq = expectedSeq
                } else {
                    // The order is load-bearing: the finished list is taken off `pending` first,
                    // because `discardPending` empties it.
                    complete = pending.map { it!! }
                    // **The transaction ends here, not after the commit.** [tick] runs on
                    // `Bridge`'s thread and this one is about to leave the lock to run the
                    // whole-plan check and the commit; a `RECEIVING` phase surviving into that
                    // window lets the 60 s cap discard a transaction that is already decided and
                    // send `OPERATION_CANCELLED` plus *"plan unchanged"* — after which this thread
                    // commits and sends `ACCEPTED`. Two acks for one upload, and an operator told
                    // "unchanged" about a plan that is in the store. Unreachable today at ≤ 20
                    // items in under 5 s, and the fix is to end the phase where it was decided.
                    discardPending()
                }
            }
        }

        refusal?.let { bad ->
            log("mission item ${item.seq} refused: ${bad.result} — ${bad.reason}")
            ack(bad.result, MissionTypes.MISSION, target, component)
            announce(bad.reason)
            return
        }

        nextSeq?.let { requestItem(it, target, component) }

        complete?.let { items ->
            // The whole-plan check, after the last item and **before** the commit. A refusal here
            // has no natural index and QGC will blame the last item we requested, so the sentence
            // must carry the real reason on its own.
            val planRefusal = MissionAdmission.planRefusal(items)
            if (planRefusal != null) {
                log("mission plan refused: ${planRefusal.result} — ${planRefusal.reason}")
                ack(planRefusal.result, MissionTypes.MISSION, target, component)
                announce(planRefusal.reason)
                return
            }
            // Read once, and used for two things: the first-leg check below and the plan's
            // provenance. One reading, so a plan cannot be admitted against one home and recorded
            // against another.
            val home = homeAtUpload()

            // The last check, and the only one that is not a fact about the list — see
            // MissionAdmission.firstLegRefusal. It runs only when home is known and it is a
            // courtesy rather than a guarantee: the guarantee is the execution half's at Start.
            // Refusing here costs the operator seconds at the desk instead of a walk back to the
            // aircraft with propellers turning.
            MissionAdmission.firstLegRefusal(items, home)?.let { legRefusal ->
                log("mission plan refused: ${legRefusal.result} — ${legRefusal.reason}")
                ack(legRefusal.result, MissionTypes.MISSION, target, component)
                announce(legRefusal.reason)
                return
            }

            // Asked *before* the commit, because after it the answer is about a plan the executor
            // has not been offered yet and the distinction is the whole announcement.
            val flying = execution().isRunning()
            store.commit(
                items = items,
                homeAtUpload = home,
                amslDatumAtUpload = amslDatumAtUpload(),
                uploadedAtMs = nowMs(),
            )
            // The final MISSION_ACK is mandatory and it is ours to send. Without it QGC reports
            // "Mission write failed, vehicle failed to send final ack" after 1500 ms and — this
            // is the part that matters — **throws the plan away** (`:203`, `:843`). A successful
            // upload we forget to ack is an upload that did not happen, on both sides.
            ack(MavMissionResult.MAV_MISSION_ACCEPTED, MissionTypes.MISSION, target, component)
            // M4-12: the upload succeeded and the aircraft is still flying the *old* plan. An
            // accepted upload that silently does not take effect is exactly the kind of belief
            // this project's honesty boundaries exist to prevent, so it is said out loud.
            if (flying) announce(MissionStatusTexts.PLAN_STORED_FOR_NEXT_START)
        }
    }

    /**
     * `MISSION_ITEM` (#39), the float form. QGC never sends it to a PX4 vehicle
     * (`writeArduPilotGuidedMissionItem` is the only producer and it is APM-only,
     * `MissionManager.cc:25`) and we do not claim `MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT`.
     *
     * Refused rather than silently converted to the int form: a float latitude is ~1 m of
     * precision lost — the same defect `DO_SET_HOME` has, and it was *measured* there
     * (`HandshakeResponder.kt:143`) — and converting would hide it.
     */
    private fun onFloatItem(missionType: Int, senderSystem: Int, senderComponent: Int) {
        synchronized(lock) { discardPending() }
        log("MISSION_ITEM (float form) refused — we take only MISSION_ITEM_INT")
        ack(MavMissionResult.MAV_MISSION_UNSUPPORTED, missionType, senderSystem, senderComponent)
        announce(MissionStatusTexts.itemRefused(0, "float items not taken"))
    }

    // --------------------------------------------------------------------------------- other

    /**
     * `MISSION_CLEAR_ALL`, and it is the message §0 made honest.
     *
     * `HandshakeResponder` refused it, and the comment explaining why was exactly right at the
     * time: *"the aircraft may well hold a DJI mission uploaded by DJI Fly, and we do not clear
     * it."* Now that the plan we fly is ours and only ours, clearing it is a true statement about
     * the thing QGC is asking about. The DJI Fly route is a different object and stays untouched
     * — which is now something the operator has to be **told** (JC-3), not something that
     * justifies a refusal.
     */
    private fun onClearAll(missionType: Int, senderSystem: Int, senderComponent: Int) =
        clearPlan(missionType, senderSystem, senderComponent)

    /**
     * The one implementation of "remove the plan", shared by `MISSION_CLEAR_ALL` and by a
     * `MISSION_COUNT` of zero — see [onCount] for why they must not diverge.
     *
     * **Still refused while the executor is running, where an upload is not.** M4-12 answered the
     * upload question and its argument carries over; its *consequence* does not. An emptied store
     * withholds `MISSION_CURRENT` entirely (JC-7) and answers a read with `MISSION_COUNT 0`, so
     * QGC would show an empty Plan view for an aircraft that is visibly flying a plan — the exact
     * state §3.2 exists to prevent. Replacing a plan leaves the operator something to look at;
     * removing one does not.
     */
    private fun clearPlan(missionType: Int, senderSystem: Int, senderComponent: Int) {
        if (missionType != MissionTypes.MISSION) {
            // We hold no fence and no rally store, so "cleared" would be a claim about something
            // that does not exist here.
            ack(MavMissionResult.MAV_MISSION_UNSUPPORTED, missionType, senderSystem, senderComponent)
            return
        }
        if (!interlockEnabled()) {
            log("mission clear refused — command interlock off")
            ack(MavMissionResult.MAV_MISSION_UNSUPPORTED, missionType, senderSystem, senderComponent)
            return
        }
        if (execution().isRunning()) {
            log("mission clear refused — the executor is flying the plan")
            ack(MavMissionResult.MAV_MISSION_DENIED, missionType, senderSystem, senderComponent)
            announce(MissionStatusTexts.MISSION_RUNNING)
            return
        }
        synchronized(lock) { discardPending() }
        store.clear()
        ack(MavMissionResult.MAV_MISSION_ACCEPTED, missionType, senderSystem, senderComponent)
        announceClearOnce()
    }

    /**
     * `MISSION_SET_CURRENT` (#41), the deprecated **message** form of set-current.
     *
     * It has no acknowledgement of any kind, so a refusal can only be a `STATUSTEXT` — which is
     * exactly why §1.6 answers `MAV_CMD_DO_SET_MISSION_CURRENT` (224) rather than letting QGC
     * fall back to this. Handled anyway, for the ground stations that only know this form.
     *
     * The sentence carries **only what the result code carries** — see
     * [MissionStatusTexts.setCurrentRefused] for the reason that is a rule and not a preference.
     *
     * [seq] is passed through **unaltered**: QGC has already subtracted the planned-home marker's
     * 1 before sending (`Vehicle.cc:2109-2111`), so our sequence space is the wire's and adding or
     * removing one here would jump the aircraft to the wrong item.
     */
    private fun onSetCurrent(seq: Int) {
        val result = execution().setCurrent(seq)
        log("MISSION_SET_CURRENT seq=$seq → $result")
        if (result != MavResult.MAV_RESULT_ACCEPTED) {
            announce(MissionStatusTexts.setCurrentRefused(seq, result))
        }
    }

    /**
     * `MAV_CMD_DO_SET_MISSION_CURRENT` (224), the command form. Registered with
     * `HandshakeResponder` by `Bridge`, so the reply is a real `COMMAND_ACK`.
     *
     * **Our first answer decides, for the lifetime of QGC's vehicle instance, which protocol it
     * speaks to us** (`MavCommandQueue.cc:129-160`): an `UNSUPPORTED` caches the fallback and
     * every later press goes straight to the reply-less message. So the refusal comes from the
     * executor as `DENIED`, and QGC raises a modal naming the command — a refusal that can be
     * *heard*.
     */
    fun onSetCurrentCommand(seq: Int): MavResult {
        val result = execution().setCurrent(seq)
        log("MAV_CMD_DO_SET_MISSION_CURRENT seq=$seq → $result")
        return result
    }

    // ------------------------------------------------------------------------------- helpers

    private fun discardPending() {
        phase = Phase.IDLE
        pending = emptyArray()
        expectedCount = 0
        expectedSeq = 0
        receivedCount = 0
    }

    private fun announceClearOnce() {
        val announce = synchronized(lock) {
            if (clearAnnounced) false else { clearAnnounced = true; true }
        }
        if (announce) announce(MissionStatusTexts.PLAN_CLEARED)
    }

    private fun ack(
        result: MavMissionResult,
        missionType: Int,
        targetSystem: Int,
        targetComponent: Int,
    ) {
        send(
            MissionAck.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .type(result)
                .missionType(missionTypeValue(missionType))
                .build()
        )
    }

    /**
     * The operator-facing half of a refusal, at `MAV_SEVERITY_ERROR` — the only severity QGC
     * surfaces (`StatusTextHandler.cc:18-24`), measured twice on this project already.
     */
    private fun announce(text: String) {
        send(
            Statustext.builder()
                .severity(MavSeverity.MAV_SEVERITY_ERROR)
                .text(text)
                .build()
        )
    }

    private fun missionTypeValue(missionType: Int): EnumValue<MavMissionType> =
        EnumValue.create(MavMissionType::class.java, missionType)

    private fun addressedToUs(targetSystem: Int, targetComponent: Int): Boolean =
        (targetSystem == 0 || targetSystem == systemId) &&
            (targetComponent == COMPONENT_ID_ALL || targetComponent == componentId)
}
