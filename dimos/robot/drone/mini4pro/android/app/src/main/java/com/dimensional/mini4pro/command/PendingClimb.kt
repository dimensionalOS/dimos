package com.dimensional.mini4pro.command

import com.dimensional.mini4pro.guided.ControlOrigin

/**
 * The seam between M2.5's takeoff and M3's guidance law — **the only thing the MAVLink command
 * layer knows about the second phase of a two-phase takeoff.**
 *
 * `FlightControllerKey.KeyStartTakeoff` is `DJIActionKeyInfo<EmptyMsg, EmptyMsg>` and the aircraft
 * stops at its own ~1.2 m ([CommandDispatcher.DJI_TAKEOFF_HEIGHT_M]); QGC's Takeoff button asks
 * for at least 3.048 m and sends it in `param7`. Until 2026-07-27 the bridge accepted that
 * mismatch and announced it, and the aircraft sat at 1.2 m — honest, and not what the operator
 * asked for. `docs/m4-mission-execution.md` §3.6 specifies the answer for a mission's takeoff item
 * and this is the same shape for the live command: **DJI's own hop, then a commanded vertical-only
 * climb to the requested altitude**, flown by the ordinary law under the ordinary envelope, the
 * ordinary ceiling and the ordinary abort ladder.
 *
 * This interface exists so that [CommandDispatcher] — which contains no DJI code, no Android and
 * no control loop — can *arm* that second phase without knowing what flies it. Everything about
 * *when* the climb starts, and every way it can be cancelled, lives on the other side, in
 * `guided/GuidedStickEngine` and its `guided/TakeoffClimb` phase machine, because that is where
 * the abort ladder already is. Splitting it the other way — a pending intention held in the
 * command layer, poked at by the engine — would have put an armed climb somewhere no abort could
 * reach it, which is the one failure this feature must not have.
 *
 * Null is a legitimate implementation: `Bridge` supplies `{ guidedStick }`, which is null before
 * `start()` and after `stop()`. A takeoff with no engine attached is still dispatched to DJI and
 * still announced; it simply does not grow a second phase, and the operator is told the old
 * sentence ([StatusTexts.takeoffHeight]) because it is then true again.
 */
interface PendingClimb {

    /**
     * Arm the climb that follows DJI's own takeoff. Called on the `mavlink-rx` thread from
     * [CommandDispatcher.onTakeoff] and **only** on the path where a takeoff actually reached
     * `FlightActions` and was answered `MAV_RESULT_ACCEPTED` — a refused or undispatched takeoff
     * must never leave anything armed.
     *
     * [requestedRelAltM] is the height above the aircraft the operator asked for, already
     * recovered from QGC's AMSL `param7` against our own published datum
     * ([CommandDispatcher.relativeTakeoffAltitude]) and already inside
     * [CommandDispatcher.MAX_TAKEOFF_HEIGHT_M]. The implementation decides whether there is
     * anything worth flying and returns [ClimbArm] saying so, so that the sentence the operator
     * reads describes what will actually happen.
     *
     * Arming replaces any previously armed climb. It never engages anything by itself: the
     * aircraft is on the ground and DJI is about to fly it.
     *
     * [aimCameraNadir]: when true, the implementation points the camera at nadir (−90°) through
     * the bridge's own gimbal path at the moment DJI hands the aircraft back — never earlier,
     * and never at all if the climb is cancelled or expires, so an aborted takeoff aborts the
     * whole sequence. Until 2026-07-29 this was the phone door's one addition ("QGC's operator
     * owns their camera" — the generic-GCS argument); it is now passed true by **both** doors,
     * the sole operator having declared the camera-down sequence wanted on every takeoff. The
     * flag rides the armed intention rather than being a second call from the phone because the
     * *when* is the engine's to know (only it observes DJI's handback), and because every abort
     * rung that kills the climb must kill the camera move with it — one intention, one ladder.
     *
     * [aimCameraNadir] and [origin] ride this seam together but are **independent facts on the
     * climb**: the origin decides whose liveness ends the engagement, the flag decides where the
     * camera goes at handback, and since 2026-07-29 a MAVLINK-origin climb carries the flag too
     * — neither is ever derived from the other.
     *
     * [origin] is the door the takeoff came in through — [ControlOrigin.MAVLINK] for QGC's
     * `MAV_CMD_NAV_TAKEOFF`, [ControlOrigin.PHONE] for the phone's Take off button — and it
     * rides the armed intention to the climb's engagement, whose liveness watchdog is evaluated
     * within that origin. It travels here rather than being re-derived engine-side because only
     * the dispatcher's doors know which one admitted the command; landing08
     * (`datasets/landing08/20260729-112216.001.jsonl`) measured the cost of not saying — a
     * phone takeoff's climb, labelled MAVLINK by default, released `link-lost` 1.6 s after
     * engaging on a flight with no QGC to be heard from. The default is MAVLINK because that is
     * the fail-safe label (its liveness demands evidence); PHONE must be named explicitly.
     */
    fun armTakeoffClimb(
        requestedRelAltM: Double,
        aimCameraNadir: Boolean = false,
        origin: ControlOrigin = ControlOrigin.MAVLINK,
    ): ClimbArm

    /**
     * Drop any armed climb — idempotent, safe from any thread, and safe when nothing is armed.
     *
     * [reason] is a short word for the operator and the flight record. Called from every rung of
     * the abort ladder inside the engine, and from [CommandDispatcher.reportAsyncDjiError],
     * because a DJI error arriving after a takeoff we started is evidence that the takeoff is in
     * doubt and a climb must not be waiting behind it.
     */
    fun cancelTakeoffClimb(reason: String)
}

/**
 * What [PendingClimb.armTakeoffClimb] decided, and therefore what the operator is told before the
 * aircraft moves.
 *
 * Deliberately not a `Boolean`: the three outcomes read differently to a pilot — nothing will
 * happen after the hop, something will, or something smaller than was asked for will — and each
 * one has its own sentence.
 */
sealed interface ClimbArm {

    /**
     * The requested height is at or below what DJI's own takeoff already delivers, so there is
     * nothing to fly and nothing was armed. Announced with the pre-existing
     * [StatusTexts.takeoffHeight] sentence, which is exactly true in this case.
     *
     * Unreachable from a stock QGC, whose takeoff slider is floored at `FirmwarePlugin.h:204`'s
     * 3.048 m, and checked anyway: engaging virtual stick to fly a manoeuvre that is arrived at
     * the instant it starts is two controllers on one aircraft for no gain.
     */
    object NothingToDo : ClimbArm

    /**
     * A climb is armed and will be flown when DJI reports its takeoff finished.
     *
     * [relAltM] is what will **actually** be flown to, in metres above this bridge's own takeoff
     * datum — already capped at the M3 ceiling, which is why it is returned rather than assumed
     * equal to the request. [capped] says the request was above the ceiling and was bound to it,
     * so the operator gets the JC-2/JC-5 substitution sentence as well: capped, announced, never
     * silently clamped.
     */
    data class Armed(val relAltM: Double, val capped: Boolean) : ClimbArm
}
