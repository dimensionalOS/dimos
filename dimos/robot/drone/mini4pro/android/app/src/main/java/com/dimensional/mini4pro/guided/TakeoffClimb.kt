package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.ClimbArm
import com.dimensional.mini4pro.command.CommandDispatcher

/**
 * The two-phase takeoff's **phase machine** — when DJI's own takeoff is finished, and therefore
 * when the commanded climb may begin. No DJI, no MAVLink, no Android, no clock it did not
 * receive, and no way to command anything: it answers one question and `TakeoffClimbTest` asks it
 * without an aircraft.
 *
 * `docs/m4-mission-execution.md` §3.6 specifies the shape and [com.dimensional.mini4pro.command
 * .PendingClimb] explains why the state lives on the guided side. This class is the part that
 * needed judgement, and the judgement is a reading of real records rather than a guess.
 *
 * ## What "DJI's takeoff is finished" means, off the wire
 *
 * Not *"we sent the command"* — landmine 5's rule applies here exactly as it does to engagement:
 * a claim comes from an observation. Not *"the aircraft reports flying"* either, and that is the
 * whole point of this class. Two real records from 2026-07-27 (`tmp/session-logs/`), both a DJI
 * takeoff on this airframe, `t` in seconds from the recorder's start:
 *
 * ```
 * orbit-first.jsonl                        orbit-real-air.jsonl
 *   30.802  motorsOn   false -> true         26.611  motorsOn   false -> true
 *   30.802  flightMode GPS_ATTI -> MOTOR_START       flightMode GPS_ATTI -> MOTOR_START
 *   32.403  flightMode -> AUTO_TAKE_OFF      28.011  flightMode -> AUTO_TAKE_OFF
 *   33.802  isFlying   false -> true         28.611  isFlying   false -> true
 *   35.803  flightMode -> GPS_ATTI           31.011  flightMode -> GPS_ATTI
 * ```
 *
 * **`isFlying` goes true 1.4 s and 0.6 s *into* `AUTO_TAKE_OFF`, with 2.0 s and 2.4 s of DJI's
 * climb still to run.** Engaging virtual stick there is two controllers on one aircraft while it
 * is leaving the ground. The signal that DJI has *let go* is the flight mode leaving the takeoff
 * family and returning to a normal one (`GPS_ATTI` in both records), and the conjunction of that
 * with `isFlying` is what this machine waits for:
 *
 *  1. `isFlying` observed **false** while armed — DJI's takeoff has not happened yet, and the
 *     aircraft is where a takeoff starts. See [Pending.sawNotFlying] for why the transition is
 *     required rather than the level.
 *  2. one of [TAKEOFF_MODES] observed — DJI's takeoff actually **began**. See
 *     [Pending.sawTakeoffMode]; this is what makes a stale flight-mode reading unable to look
 *     like a finished takeoff.
 *  3. then `isFlying` **true**,
 *  4. **and** [AircraftState.flightMode][com.dimensional.mini4pro.telemetry.AircraftState
 *     .flightMode] known and not in [TAKEOFF_MODES].
 *
 * Both records satisfy all four exactly once, at the moment DJI's own mode trail ends. A null
 * mode never satisfies conjunct 4: DJI has not told us, which is not the same as "not taking
 * off", and this is the one gate whose failure mode is engaging too early.
 *
 * ## Everything else is bounded rather than trusted
 *
 * [WAIT_LIMIT_MS] ends a wait that is not going anywhere with a sentence, because an armed
 * intention with no expiry is one that fires into a situation nobody remembers creating.
 * [cancel] is called by every rung of the engine's abort ladder. And [arm] refuses to arm at all
 * for a request DJI's own hop already satisfies ([NO_CLIMB_BELOW_M]).
 *
 * **Not thread-safe.** The engine touches it only under its own lock, which is also the lock
 * every abort takes, so an abort and a firing tick cannot interleave.
 */
class TakeoffClimb {

    companion object {

        /**
         * DJI flight-mode names that mean *"DJI is flying the takeoff right now"*, so the
         * commanded climb waits.
         *
         * `MOTOR_START` and `AUTO_TAKE_OFF` are the two observed in both 2026-07-27 records, in
         * that order, and they are the whole of the measured trail. `ASSISTED_TAKE_OFF` and
         * `MOTORS_JUST_STARTED` are added from the `FCFlightMode` surface `telemetry/Px4Mode`
         * already enumerates (`Px4Mode.kt:248-254`, `:536-540`): neither has been seen on this
         * airframe, and including them can only ever make this machine wait longer, which is the
         * safe direction. A mode this set does not know about reads as "DJI has let go", so the
         * set must stay a list of *takeoff* modes and must never grow into a list of modes we
         * happen not to like.
         */
        val TAKEOFF_MODES = setOf(
            "MOTOR_START",
            "MOTORS_JUST_STARTED",
            "AUTO_TAKE_OFF",
            "ASSISTED_TAKE_OFF",
        )

        /**
         * How long an armed climb waits for DJI to fly its takeoff before it is dropped with a
         * sentence, in milliseconds.
         *
         * The bound exists because the alternative is an intention with no expiry: an operator
         * whose takeoff was refused by the flight controller on a channel we cannot hear — the
         * measured case, where DJI answered a `performAction` with an RC beep and neither
         * callback (`docs/measurements/2026-07-26-m2-first-command.md`) — would otherwise have a
         * climb sitting armed for the rest of the session, ready to fire the moment the aircraft
         * next left the ground under someone else's hand.
         *
         * 30 s is about seven times the **measured** duration of DJI's takeoff itself — 4.4 s
         * from `MOTOR_START` to the mode trail ending, in both 2026-07-27 records. What is *not*
         * measured is the delay between our `KeyStartTakeoff` and the motors spinning, because
         * both of those takeoffs were flown from the RC; that gap is what the margin is for, and
         * it is the first thing the bench should time.
         */
        const val WAIT_LIMIT_MS = 30_000L

        /**
         * At or below this many metres above the takeoff datum there is nothing worth flying
         * after DJI's hop, so no climb is armed and the operator is told the old truth.
         *
         * It is DJI's own [CommandDispatcher.DJI_TAKEOFF_HEIGHT_M] plus the reposition law's own
         * [RepositionGuidance.VERTICAL_ACCEPT_M], and the sum is the point rather than a padded
         * 1.2: a target within the vertical acceptance of where the aircraft already is would
         * satisfy the arrival test on the first tick, so arming it would enable virtual stick,
         * announce a manoeuvre, arrive immediately and disengage — a whole engagement to fly
         * nowhere. Both halves are referenced rather than copied so that moving either moves this.
         *
         * Unreachable from a stock QGC (its takeoff slider floors at 3.048 m) and checked
         * regardless, because this bridge has a second command origin coming and *"a feature that
         * can never succeed"* is not the same as *"a case that cannot arrive"*.
         */
        val NO_CLIMB_BELOW_M = CommandDispatcher.DJI_TAKEOFF_HEIGHT_M + RepositionGuidance.VERTICAL_ACCEPT_M
    }

    /** What [observe] concluded from one look at the aircraft. */
    sealed interface Decision {

        /** Nothing is armed. The overwhelmingly common answer, and it costs one null check. */
        object Idle : Decision

        /** Something is armed and DJI's takeoff is not finished (or not knowable yet). */
        object Waiting : Decision

        /**
         * **DJI has let go.** The four conjuncts are all satisfied and the aircraft is ours.
         *
         * [relAltM] is what to climb to, in metres above this bridge's own takeoff datum, or
         * **null** when this machine was armed by [armWatch] and the caller carries its own
         * target — the mission executor's takeoff item, whose altitude lives in the step and is
         * flown by the mission's own leg law. A null is never produced by [arm].
         *
         * **The pending climb is consumed by this decision** — it is returned exactly once, so a
         * caller that fails to fly it does not get a second chance from a machine that has
         * forgotten why it was armed.
         *
         * [aimCameraNadir] rides the handback because the handback is the moment a takeoff
         * sequence's camera move is due ([com.dimensional.mini4pro.command.PendingClimb]
         * .armTakeoffClimb): consumed with the climb, so a cancelled or expired intention can
         * never aim a camera later. [armWatch] carries it too, since 2026-07-30 — a mission's
         * `NAV_TAKEOFF` is a takeoff door like the other two and wants the same camera, and
         * landing16 is the flight that measured what it cost to be the one door that did not.
         *
         * [origin] is the door the takeoff came in through, riding the same path for the same
         * reason: the climb the handback fires is an engagement, and the engagement's liveness
         * watchdog is evaluated within the commanding origin ([ControlOrigin]) — a phone
         * takeoff's climb labelled MAVLINK dies `link-lost` 1.6 s after engaging on a QGC-less
         * flight (landing08, `datasets/landing08/20260729-112216.001.jsonl`, t=32.33→33.93).
         * Null exactly when [relAltM] is null — [armWatch]'s form, where the mission carries its
         * own origin — and never null from [arm].
         */
        data class HandedBack(
            val relAltM: Double?,
            val aimCameraNadir: Boolean = false,
            val origin: ControlOrigin? = null,
        ) : Decision

        /** [WAIT_LIMIT_MS] elapsed with no takeoff. The pending climb is dropped. */
        object Expired : Decision
    }

    private class Pending(
        val relAltM: Double?,
        val armedAtMs: Long,
        /** See [Decision.HandedBack.aimCameraNadir] — carried, never acted on by this machine. */
        val aimCameraNadir: Boolean = false,
        /** See [Decision.HandedBack.origin] — carried, never acted on by this machine. */
        val origin: ControlOrigin? = null,
    ) {
        /**
         * Whether the aircraft has been observed **not flying** since this climb was armed.
         *
         * Required, rather than starting from the level, because "it is flying now" cannot
         * distinguish DJI's takeoff having completed from the aircraft having been airborne all
         * along. The second case is the dangerous one: a takeoff commanded at 30 m would arm a
         * climb whose target is 3 m, and the vertical law would fly the aircraft *down* to it.
         * QGC only offers Takeoff on the ground and DJI refuses one with the motors running, so
         * this should be unobservable — and if it ever is observed, the machine waits for a
         * transition that never comes and expires with a sentence, which is the failure worth
         * having.
         */
        var sawNotFlying = false

        /**
         * Whether one of [TAKEOFF_MODES] has been observed since this climb was armed — DJI's
         * takeoff **started**, not merely finished.
         *
         * This is what closes the one race the other conjuncts leave open. `flightMode` is a
         * change-driven key with **no staleness limit** (`telemetry/SampleAges`: `FLIGHT_MODE`
         * carries none, deliberately, because an hour of silence there means the mode has not
         * changed). So without this flag a `GPS_ATTI` left over from *before* the takeoff, read
         * on the tick `isFlying` flips, would look exactly like DJI having let go — and the climb
         * would engage in the middle of `AUTO_TAKE_OFF`, which is the one failure this whole
         * class exists to prevent.
         *
         * Both 2026-07-27 records make it free rather than a trade: the mode reaches
         * `MOTOR_START` **3.0 s and 2.0 s before** `isFlying` goes true, so there is no honest
         * sequence in which the flag is unset when it is needed. The same trail was observed
         * under the MSDK simulator (`docs/measurements/2026-07-26-simulated-flight-session.md`:
         * `MOTOR_START → AUTO_TAKE_OFF`), so it is not a property of real air only.
         *
         * **The cost, stated plainly:** a DJI takeoff that ever completed without publishing a
         * takeoff mode would never produce a climb. That aircraft sits at ~1.2 m and is told so
         * by [GuidedStatusTexts.TAKEOFF_CLIMB_EXPIRED] — which is exactly the behaviour this
         * feature replaced, so the fail-closed direction costs nothing that was not already the
         * status quo, while the fail-open direction costs two controllers on one climbing
         * aircraft.
         */
        var sawTakeoffMode = false
    }

    private var pending: Pending? = null

    /** What an armed climb will fly to, metres above the takeoff datum, or null if none is armed. */
    val armedAltitudeM: Double? get() = pending?.relAltM

    /** Whether a climb is waiting for DJI to finish. */
    val armed: Boolean get() = pending != null

    /**
     * Arm the climb for a takeoff that has just been dispatched to DJI, replacing any previous
     * one. [now] is the monotonic clock [WAIT_LIMIT_MS] is measured on.
     *
     * The ceiling is applied **here**, at arm time, rather than being left to the accept path the
     * climb eventually takes. Both would cap; only this one lets the operator read the
     * substitution before the aircraft leaves the ground, beside the sentence about DJI's 1.2 m,
     * instead of half a minute later when the climb starts.
     *
     * [origin] names the door the takeoff came in through and rides the pending intention to the
     * handback unchanged (the [aimCameraNadir] pattern). It defaults to
     * [ControlOrigin.MAVLINK] because that is the fail-safe label — a caller that forgets to say
     * gets the origin whose liveness demands evidence, never the one that is alive by identity —
     * so [ControlOrigin.PHONE] must always be named explicitly by the phone door.
     */
    fun arm(
        requestedRelAltM: Double,
        now: Long,
        aimCameraNadir: Boolean = false,
        origin: ControlOrigin = ControlOrigin.MAVLINK,
    ): ClimbArm {
        pending = null
        // NaN is false in every comparison, so it is rejected explicitly rather than left to fall
        // through the `<=` below into an armed climb toward a target that is not a number.
        // Note a NothingToDo also drops [aimCameraNadir]: with no climb there is no handback to
        // hang the camera move on, and the phone's constant height (10 m) cannot reach this
        // branch — see `CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M`.
        if (!requestedRelAltM.isFinite() || requestedRelAltM <= NO_CLIMB_BELOW_M) return ClimbArm.NothingToDo
        val capped = requestedRelAltM > GuidedEnvelope.CEILING_M
        val target = if (capped) GuidedEnvelope.CEILING_M else requestedRelAltM
        pending = Pending(target, now, aimCameraNadir, origin)
        return ClimbArm.Armed(target, capped)
    }

    /**
     * Arm the **same wait with no climb behind it** — the four conjuncts, the [WAIT_LIMIT_MS]
     * bound and [cancel], answering only *"has DJI let go?"*. Replaces any previous arming.
     *
     * This is what the mission executor's takeoff item uses. It needs the question answered
     * exactly as the operator's Takeoff button does — and for the more urgent reason, because the
     * item after a takeoff is a leg the mission will fly *horizontally*, and advancing the cursor
     * on `isFlying` alone would start it 1.4–2.4 s before DJI stopped climbing (the two records
     * quoted in this class's KDoc). What it does not need is a target: the height lives in
     * [MissionStep.relAltM][com.dimensional.mini4pro.guided.MissionStep.relAltM] and is flown by
     * the mission's own leg law, so there is one place a mission's altitude comes from.
     *
     * That is also why this does not go through [arm]: [arm]'s [NO_CLIMB_BELOW_M] answer is *"do
     * not engage to fly nowhere"*, which is right for a live command that would have to engage
     * virtual stick on purpose, and wrong here — a mission is already engaged and must wait for
     * the handback whether or not a climb follows it.
     *
     * What it *does* carry, since 2026-07-30, is [aimCameraNadir]: the camera half of a takeoff
     * sequence is not a climb and does not need a target, and it rides this pending intention
     * exactly as it rides [arm]'s — consumed once at the handback, and dropped with the watch by
     * every rung of the abort ladder, so a mission takeoff that never happened never aims a
     * camera. Before this, a mission's `NAV_TAKEOFF` was the only one of the three takeoff doors
     * that left the camera where the RC wheel had it; landing16
     * (`datasets/landing16/20260730-161329.001.jsonl`) is the flight that cost.
     */
    fun armWatch(now: Long, aimCameraNadir: Boolean) {
        pending = Pending(null, now, aimCameraNadir)
    }

    /**
     * One look at what DJI reports. [flying] and [flightMode] are
     * [AircraftState.isFlying][com.dimensional.mini4pro.telemetry.AircraftState.isFlying] and
     * [AircraftState.flightMode][com.dimensional.mini4pro.telemetry.AircraftState.flightMode]
     * verbatim — nulls included, because a null is DJI declining to say and is never read as an
     * answer.
     *
     * Order matters and is the safety property: the expiry is checked **first**, so a machine
     * that has waited too long cannot be rescued by a late-arriving pair of readings, and the
     * two remaining exits (climb, keep waiting) are decided only inside the window.
     */
    fun observe(flying: Boolean?, flightMode: String?, now: Long): Decision {
        val p = pending ?: return Decision.Idle
        if (now - p.armedAtMs > WAIT_LIMIT_MS) {
            pending = null
            return Decision.Expired
        }
        // Recorded before anything is decided, and on every tick regardless of `flying`: DJI's
        // takeoff *starting* is an observation in its own right, and in both measured records it
        // happens seconds before the aircraft reports flying at all.
        if (flightMode != null && flightMode in TAKEOFF_MODES) p.sawTakeoffMode = true
        if (flying == false) {
            p.sawNotFlying = true
            return Decision.Waiting
        }
        // Null (component gone, or nothing delivered yet) is not "flying" and is not "not
        // flying": it is no statement at all, and it waits.
        if (flying != true) return Decision.Waiting
        if (!p.sawNotFlying) return Decision.Waiting
        if (!p.sawTakeoffMode) return Decision.Waiting
        if (flightMode == null || flightMode in TAKEOFF_MODES) return Decision.Waiting
        pending = null
        return Decision.HandedBack(p.relAltM, p.aimCameraNadir, p.origin)
    }

    /**
     * Drop any armed climb. Returns true if there was one, so the caller can say so exactly once
     * rather than announcing a cancellation of nothing on every abort.
     */
    fun cancel(): Boolean {
        val had = pending != null
        pending = null
        return had
    }
}
