package com.dimensional.mini4pro.mission

import io.dronefleet.mavlink.common.MissionCurrent
import io.dronefleet.mavlink.common.MissionItemReached
import io.dronefleet.mavlink.util.EnumValue

/**
 * The two messages that tell a ground station where in the plan we are.
 *
 * Owns its own cadence, the way `GimbalManager` does, because owning it here is what makes it
 * testable: [messages] is asked on every `Bridge` tick and decides for itself whether anything is
 * due. `Bridge` does not hold a divisor for it.
 *
 * ## `MISSION_CURRENT` (#42) — the only channel QGC actually reads
 *
 * `MissionManager::_handleMissionCurrent` (`MissionManager.cc:270-275`) reads **`seq` and nothing
 * else**. `total`, `mission_state` and `mission_mode` have zero readers in the entire QGC tree
 * (`grep -rn "mission_state\|MISSION_STATE_" src/` → nothing). We populate them honestly anyway:
 * they are free (MAVLink 2 truncates trailing zeros), other ground stations read them, and the
 * flight recorder gets a better trace. That this QGC ignores them is recorded here so nobody
 * debugs their absence.
 *
 * `seq` is **wire-numbered** (§4.1). QGC adds 1 back for display, because it deleted the
 * planned-home marker and renumbered everything down by one before uploading
 * (`PlanManager::writeMissionItems`, `:70-92`; `MissionController::currentMissionIndex():1724`).
 * Getting this wrong is invisible in a two-item plan and highlights the wrong waypoint in a
 * twelve-item one.
 *
 * ### JC-7: withheld entirely when the store is empty
 *
 * `seq` has no "none" value, and sending `seq = 0` for a vehicle with no mission makes QGC
 * highlight plan item 1. Silence over a wrong sentinel — the same rule as `HOME_POSITION`, which
 * is simply not sent when home is unknown, and the same rule `PLAN.md` states as *sentinels or
 * silence*.
 *
 * ### The one place two rules had to be reconciled
 *
 * The seam contract says *"[MissionExecution.currentSeq] null → emit nothing"*, and §7.1 says
 * *"before the mission starts, `seq = 0` — the next item we will fly is 0, which is true"*. Those
 * cannot both be read literally, because an executor that has not started has no cursor to
 * report and would therefore silence a plan QGC is holding and wants to draw. Resolved as:
 * **the store decides whether anything is sent, and the executor decides what it says.** An empty
 * store is silent (JC-7, unconditionally); a committed plan with no cursor reports 0, which is
 * §7.1's own sentence.
 *
 * The same reconciliation applies to `mission_state`: an executor that does not exist reports
 * `NO_MISSION`, which would contradict the `total` field in the very same message. With a plan
 * committed and the executor reporting `NO_MISSION`, we send `NOT_STARTED` — the one fact
 * transport itself owns.
 *
 * ### And the third one, which has no reconciliation available (M4-12)
 *
 * `seq` is the executor's and `total` is the store's, and since M4-12 those can be **two different
 * plans**: an upload accepted mid-flight replaces the store while the running mission carries on
 * with the snapshot it began with. A 12-item mission at item 8 with a fresh 3-item plan uploaded
 * publishes `seq = 8, total = 3`. Unlike the two above there is no true value to substitute, so
 * the message is withheld — see [messages]. The real fix belongs to the execution half and is
 * named there.
 *
 * ## `MISSION_ITEM_REACHED` (#46) — knowingly redundant (JC-6)
 *
 * `grep -rn "ITEM_REACHED\|itemReached" src/` over QGC returns **nothing**: not decoded, not
 * displayed, not used to advance anything. We send it anyway — one message per waypoint, other
 * ground stations use it, and it puts an unambiguous arrival timestamp on the wire that the
 * flight recorder and any pcap can see. Recorded as redundant so nobody optimises it away and
 * nobody debugs its absence.
 *
 * **But it must never be the only channel for anything.** QGC's Plan view animates from
 * `MISSION_CURRENT` alone; if the plan view does not advance, this is not the thing to look at.
 */
class MissionProgress(
    private val store: MissionStore,
    private val execution: () -> MissionExecution = { NoMissionExecution },
    /** Must be safe to call from whatever thread reports an arrival. */
    private val send: (Any) -> Unit,
    private val log: (String) -> Unit = {},
) {

    companion object {
        /** MAVLink says stream `MISSION_CURRENT` always; 1 Hz is the nominal rate. */
        const val PERIOD_MS = 1_000L
    }

    private data class Emitted(val seq: Int, val total: Int, val state: MissionRunState)

    @Volatile private var lastEmitted: Emitted? = null

    /**
     * When the last `MISSION_CURRENT` went out, or null if none has since the last [reset].
     *
     * Null rather than `Long.MIN_VALUE`: `nowMs - Long.MIN_VALUE` **overflows** for every positive
     * clock, wrapping to a large negative number and reading as *"not due"* — the exact opposite
     * of what the sentinel was chosen to mean. It has never bitten because [lastEmitted] is
     * cleared in lockstep and the change test fires first, so the value is a trap rather than a
     * bug. Written so it cannot be either.
     */
    @Volatile private var lastSentAtMs: Long? = null

    /**
     * Whatever is due right now: a `MISSION_CURRENT` at 1 Hz, **and immediately on change**.
     * Empty whenever the store holds no plan.
     */
    fun messages(nowMs: Long): List<Any> {
        val plan = store.plan()
        if (plan == null || plan.items.isEmpty()) {
            // Not merely "nothing due" — the whole message is withheld, and the memory of what we
            // last sent goes with it, so a fresh upload announces itself immediately.
            lastEmitted = null
            return emptyList()
        }

        val executor = execution()
        val seq = executor.currentSeq() ?: 0
        val total = plan.items.size

        // The second contradiction of the same kind as the one below, and it arrived with M4-12.
        // An upload accepted mid-flight replaces the store while **the running mission carries on
        // with the snapshot it began with**, so `seq` and `total` are then measurements of two
        // different plans: a 12-item mission at item 8 with a fresh 3-item plan uploaded publishes
        // `seq = 8, total = 3`, and QGC highlights item 9 of a three-item plan.
        //
        // There is no honest `seq` to publish here. Clamping would claim the mission is at the
        // last item of a plan it has never flown a metre of; publishing anyway is the wrong
        // highlight. So the message is withheld, on the same rule as JC-7 — *sentinels or
        // silence*, and a cursor that is not an index into the plan we are reporting has no
        // sentinel. The memory goes with it, so the first consistent tick announces itself.
        //
        // **This is the honest guard, not the fix.** The fix is the executor reporting `seq`
        // against the plan it is actually flying, which means reporting that plan's identity or
        // its length alongside the cursor — an execution-half API that does not exist and must not
        // be invented from this side. Until then this layer refuses to publish a pair it cannot
        // stand behind.
        if (seq >= total) {
            log("mission cursor $seq is not an index into the stored plan ($total items) — withheld")
            lastEmitted = null
            return emptyList()
        }

        val state = when (val reported = executor.runState()) {
            // Transport's own truth wins over an executor that does not exist: a plan is
            // committed, so NO_MISSION would contradict `total` in the same message.
            MissionRunState.NO_MISSION -> MissionRunState.NOT_STARTED
            else -> reported
        }
        val next = Emitted(seq, total, state)

        val changed = next != lastEmitted
        val due = lastSentAtMs?.let { nowMs - it >= PERIOD_MS } ?: true
        if (!changed && !due) return emptyList()

        lastEmitted = next
        lastSentAtMs = nowMs
        return listOf(
            MissionCurrent.builder()
                .seq(next.seq)
                .total(next.total)
                .missionState(EnumValue.of(next.state.wire))
                // mission_mode: 0 = "unknown / not supported". We do not claim a mission mode,
                // and inventing one would be a claim about a scheduler that does not exist yet.
                .missionMode(0)
                .build()
        )
    }

    /**
     * The executor's arrival edge, pushed. One message per item, at the instant arrival is
     * declared — the timestamp is the entire value of it.
     */
    val reachedSink = MissionReachedSink { seq ->
        log("mission item $seq reached")
        send(MissionItemReached.builder().seq(seq).build())
    }

    /** Drops the change memory so a new link is told the current state on its first tick. */
    fun reset() {
        lastEmitted = null
        lastSentAtMs = null
    }
}
