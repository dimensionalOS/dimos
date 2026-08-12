package com.dimensional.mini4pro.mission

import com.dimensional.mini4pro.command.StatusTexts
import io.dronefleet.mavlink.common.MavResult

/**
 * The operator-facing half of every mission refusal.
 *
 * `MISSION_ACK` carries a `MAV_MISSION_RESULT` and QGroundControl turns it into a sentence of its
 * own, appending *"Item #N Command: &lt;friendly name&gt;"* from the last index we requested
 * (`PlanManager::_lastMissionReqestString`, `:692-739`). That gets the operator the index and the
 * command for free — but it never says **why**. This object says why.
 *
 * Every sentence goes out at `MAV_SEVERITY_ERROR`, and that is measured rather than chosen: only
 * EMERGENCY / ALERT / CRITICAL / ERROR satisfy `StatusText::severityIsError()`
 * (`StatusTextHandler.cc:18-24`), and anything below is filed silently in a list nobody opens
 * after pressing a button that appeared to do nothing. The same finding already forced
 * `HandshakeResponder.MODE_REFUSAL_TEXT` and the parameter refusals up to ERROR.
 *
 * Every sentence is under `STATUSTEXT`'s hard 50-byte field width, through
 * [StatusTexts.clamp] — the one UTF-8-safe cut in this project. The byte counts below are
 * counted and pinned by tests, exactly as `MODE_REFUSAL_TEXT` is.
 *
 * The `Item N refused: <reason>` form follows `StatusTexts.preferring`'s rule: the *reason* word
 * is what survives when the framing will not fit.
 */
object MissionStatusTexts {

    /**
     * `Item 5 refused: no camera control` — 33 bytes for a one-digit index.
     *
     * The reason word is the operator's whole action item: QGC has already told them which item
     * and which command, so repeating either would spend bytes on what they can already read.
     */
    fun itemRefused(seq: Int, reason: String): String =
        StatusTexts.clamp("Item $seq refused: $reason")

    /**
     * `Item 3 set-current: denied` — the refusal of a `MISSION_SET_CURRENT` (#41), which has no
     * acknowledgement of any kind and can therefore only be refused in words.
     *
     * **It says what the result code says and not one word more.** Until 2026-07-27 this path
     * announced *"Item N refused: no mission running"* for every non-`ACCEPTED` result, which was
     * true of the stub that was the only executor at the time and is a lie the moment a real one
     * refuses for a bad index or an envelope violation: the operator is told no mission is running
     * while watching one fly. A sentence must not name a state we have not observed, and a
     * `MAV_RESULT` is the entire observation available here.
     *
     * The mapping is MAVLink's own reading of each code, kept to the one word that survives:
     * `DENIED` is "the command is invalid", `TEMPORARILY_REJECTED` is "valid, but not now",
     * `UNSUPPORTED` is "not supported", `FAILED` is "valid, and execution failed". Anything else
     * gets the bare fact of the refusal rather than a guess at its meaning.
     */
    fun setCurrentRefused(seq: Int, result: MavResult): String =
        StatusTexts.clamp("Item $seq set-current: ${setCurrentReason(result)}")

    private fun setCurrentReason(result: MavResult): String = when (result) {
        MavResult.MAV_RESULT_DENIED -> "denied"
        MavResult.MAV_RESULT_TEMPORARILY_REJECTED -> "rejected for now"
        MavResult.MAV_RESULT_UNSUPPORTED -> "unsupported"
        MavResult.MAV_RESULT_FAILED -> "failed"
        else -> "refused"
    }

    /**
     * `Mission alt must be relative, not AMSL` — 38 bytes, counted.
     *
     * The refusal that will surprise an operator, so it says what to do rather than what
     * happened. QGC's Plan view lets an altitude mode be chosen per item and plan-wide, and a
     * plan authored in AMSL simply will not upload here. The reason is
     * `docs/measurements/2026-07-26-amsl-datum.md`: DJI's "AMSL" is pressure altitude on the
     * 1013.25 hPa reference and moved 41.5 m between two sessions, so an absolute altitude
     * composed anywhere else is in a different datum by tens of metres.
     */
    const val ABSOLUTE_ALTITUDE = "Mission alt must be relative, not AMSL"

    /** `Item 4 frame 10 refused: not relative` — a frame that is neither relative nor MISSION. */
    fun frameRefused(seq: Int, frame: Int): String =
        StatusTexts.clamp("Item $seq frame $frame refused: not relative")

    /**
     * `Item 3 speed 8.0 refused: 3.0 max` — 33 bytes.
     *
     * Refused, never clamped. M3 Q1's rule: a clamped command is one the operator believes was
     * obeyed, and an upload is a re-doable transaction, so there is no cost to saying no.
     */
    fun speedRefused(seq: Int, requestedMps: Double, maxMps: Double): String = StatusTexts.clamp(
        "Item $seq speed ${oneDecimal(requestedMps)} refused: ${oneDecimal(maxMps)} max"
    )

    /** `Item 2 alt 45.0 refused: 30m max` — an altitude outside the Q1 ceiling. */
    fun altitudeRefused(seq: Int, requestedM: Double, maxM: Double): String = StatusTexts.clamp(
        "Item $seq alt ${oneDecimal(requestedM)} refused: ${maxM.toInt()}m max"
    )

    /** `Plan too long: 137 items, 20 max` — checked at `MISSION_COUNT`, before any round trip. */
    fun planTooLong(count: Int, maxItems: Int): String =
        StatusTexts.clamp("Plan too long: $count items, $maxItems max")

    /** `Leg 4 is 180m: 100m max` — a single leg beyond the supervisable distance. */
    fun legTooLong(seq: Int, metres: Double, maxM: Double): String =
        StatusTexts.clamp("Leg $seq is ${metres.toInt()}m: ${maxM.toInt()}m max")

    /** `Plan is 812m long: 500m max` — the whole path's extent. Names no item on purpose. */
    fun planTooFar(metres: Double, maxM: Double): String =
        StatusTexts.clamp("Plan is ${metres.toInt()}m long: ${maxM.toInt()}m max")

    /**
     * `Plan has no waypoint to fly` — a plan of nothing but `DO_*` items.
     *
     * A whole-plan failure has no natural index and QGC will blame the *last* item we requested,
     * which is slightly misleading. So this sentence deliberately **names no item**: it has to
     * carry the real reason on its own.
     */
    const val NO_NAVIGABLE_ITEM = "Plan has no waypoint to fly"

    /**
     * `Plan stored - takes effect at next Start` — **40 bytes, counted here**, not the 39 the
     * decision doc states. The sentence is kept verbatim because it is the operator-facing string
     * that was specified; only the arithmetic is corrected, and the test pins the measurement
     * rather than the claim.
     *
     * **M4-12, and it replaced a refusal.** The transport design refused an upload arriving while
     * the aircraft was flying a plan (`MAV_MISSION_DENIED`); Ivan overruled that with *"YES"*.
     * Of the two things "yes" could mean, this is the safe one: the upload is accepted into the
     * store, **the running mission continues on the snapshot it began with**, and the new plan
     * takes effect at the next Start. The alternative — swapping the plan under a moving aircraft
     * — leaves the cursor indexing a list nobody flew a metre of, so leg 4 of the new plan may be
     * nowhere near leg 4 of the old one and the aircraft's next move is a leg that was never
     * drawn. That reading was considered and deliberately not chosen.
     *
     * The store's immutable-snapshot design already makes this nearly free, which is why the
     * sentence is the whole implementation cost. It is not optional: without it the operator's
     * upload appears to have taken effect on an aircraft that is still flying the old plan.
     */
    const val PLAN_STORED_FOR_NEXT_START = "Plan stored - takes effect at next Start"

    /**
     * `Mission running - stop it before clearing` — 41 bytes, counted. (The design document's
     * `…before uploading` variant is gone with the refusal it explained; M4-12 accepts uploads.)
     *
     * Paired with `MAV_MISSION_DENIED`, which QGC renders as *"Not accepting any mission
     * commands"* (`PlanManager.cc:788`) — precisely true while the plan is being flown, and
     * deliberately different from the `UNSUPPORTED` we use for a command we will never fly
     * (JC-4).
     *
     * **Why a clear is still refused when an upload is not.** M4-12 answered the *upload*
     * question, and its argument — the executor holds its own snapshot, so the store may change
     * underneath it — carries over unchanged. What does not carry over is the consequence for the
     * ground station: an emptied store means `MISSION_CURRENT` is withheld entirely (JC-7) and a
     * read-back returns `MISSION_COUNT 0`, so QGC would show **an empty Plan view for an aircraft
     * that is visibly flying a plan**. That is the exact state §3.2 exists to prevent, and it is
     * the reason the store outlives a link at all. Replacing a plan leaves the operator something
     * to look at; removing one does not.
     *
     * A **read** is always allowed, in every state; it is the one thing an operator watching an
     * aircraft fly a plan actually needs.
     */
    const val MISSION_RUNNING = "Mission running - stop it before clearing"

    /**
     * `Upload stopped: 4 of 12 items, plan unchanged` — 45 bytes at these digits.
     *
     * The second clause is the whole point. A failed upload leaves the previous plan exactly as
     * it was (§2.2), QGC throws its own half away (`_clearAndDeleteWriteMissionItems`, `:843`),
     * and so after a refusal both ends agree that the old plan is the plan. That is a state an
     * operator can act on, unlike "one of us has half a plan" — but only if they are told.
     *
     * Sent alongside `MAV_MISSION_OPERATION_CANCELLED`, for which QGC has no case and falls
     * through to *"Unknown error: 15"* (`:791`). Ugly, and still better than a lie; this sentence
     * is the real one.
     */
    fun uploadAbandoned(received: Int, expected: Int): String =
        StatusTexts.clamp("Upload stopped: $received of $expected items, plan unchanged")

    /**
     * `Plan cleared. DJI Fly routes untouched` — 38 bytes, counted. JC-3.
     *
     * Announced because of the belief it prevents. "The aircraft now has no mission at all" is
     * what an operator would be *right* to conclude about any other autopilot, and here it is
     * false: the plan we cleared is ours, held in this phone, and a route uploaded from DJI Fly
     * is a different object we never touch. Once per link session, so a routine action does not
     * become alarm fatigue.
     */
    const val PLAN_CLEARED = "Plan cleared. DJI Fly routes untouched"

    /**
     * One decimal place with a `.` regardless of locale — `String.format` without an explicit
     * [java.util.Locale] renders `3,0` in half of Europe, which is where this runs. Same reason
     * [StatusTexts] carries its own copy.
     */
    private fun oneDecimal(value: Double): String =
        String.format(java.util.Locale.ROOT, "%.1f", value)
}
