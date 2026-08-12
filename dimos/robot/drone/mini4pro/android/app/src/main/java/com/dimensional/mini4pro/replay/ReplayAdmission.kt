package com.dimensional.mini4pro.replay

/**
 * **When a past flight may be put on the screen, and what is not allowed while it is there.**
 *
 * The situation view can be driven from the aircraft or from a recording, and the two produce
 * the same shapes in the same frame. On a phone strapped beside an armed interlock that is a
 * genuinely dangerous pair of modes: an operator glances down, sees an aircraft moving, and
 * believes it. This object is the *behavioural* half of keeping them apart. The structural half
 * is `situation/Situation` — a value type with no member that reaches a command path, which is
 * what a replayed state is turned into before it leaves the replay controller.
 *
 * ## Three rules, and why they are these three
 *
 * 1. **A replay may not begin while the command path is live.** Not "is discouraged" — refused.
 *    The mistake being prevented is not a wrong tap; it is an operator who opens a recording to
 *    check something, gets absorbed in it, and stops watching an aircraft that QGC can still
 *    move. If commands are switched on, or a manoeuvre of ours is engaged, the answer is no and
 *    the sentence says which.
 *
 * 2. **The interlock may not be armed while a replay is on screen.** This is the rule that
 *    makes the first one hold: without it, rule 1 is a door you walk through and then unlock
 *    from the inside. Together they mean *live commands and a replayed picture never coexist*,
 *    in either order of arrival.
 *
 * 3. **A replay may not be *published* while an aircraft is connected.** Added 2026-07-28 with
 *    the MAVLink-out and Zenoh-out switches, and it is the rule those switches exist under.
 *
 *    Rules 1 and 2 keep a recording out of the *command* path. This one keeps it out of the
 *    *description* path, and the hazard is different in kind: a published replay puts a moving
 *    aircraft on QGroundControl's map and a moving aircraft on DiMOS's bus. If a real one is
 *    connected at the same time, the two streams race for the same downstream — same system id,
 *    same key expressions — and what arrives is one aircraft's position interleaved with
 *    another's, at 5 Hz, with nothing in either message saying which. That is not a degraded
 *    picture; it is a **fabricated** one, and it is fabricated in the one place an operator has
 *    no way to check it.
 *
 *    So: connected aircraft, no publishing. Not "the bridge is running" — telemetry flowing to a
 *    ground station is not a hazard and gating on it would forbid the honest workflow. The test
 *    is whether there is a *flight controller on the other end*, which is what
 *    `AircraftState.fcConnected` answers and what the rest of this project already treats as the
 *    difference between a live feed and a dead one.
 *
 * ### What rule 3 does **not** try to do
 *
 * It does not stop a replay from being *watched* beside a connected aircraft. That was already
 * safe and is often the point — land, look at what just happened, with the aircraft still on the
 * bench. Only the switches are refused, and only while an aircraft is there.
 *
 * ## What is deliberately *not* here
 *
 * Stopping a replay is unconditional, and there is no method for it to consult. Ending a
 * picture is a withdrawal in the same sense the abort ladder means it: it can only ever make
 * the screen more honest, so nothing may refuse it.
 *
 * Nor is there any rule about the *bridge* running. Telemetry flowing to a ground station is
 * not a hazard and does not become one because the phone is showing something else; gating on
 * it would buy nothing and would make the honest workflow — land, stop commanding, review the
 * flight with the link still up — impossible.
 */
object ReplayAdmission {

    /** Refusal when the interlock is on: the aircraft can still be moved from QGC. */
    const val REASON_COMMANDS_LIVE =
        "Commands are live. Switch them off before replaying a recording."

    /** Refusal when one of our own manoeuvres is engaged. */
    const val REASON_MANOEUVRE =
        "A manoeuvre is flying. Stop it before replaying a recording."

    /** Refusal when the arming switch is reached for while a recording is on screen. */
    const val REASON_REPLAYING =
        "Replay is on screen. Close it before allowing commands."

    /**
     * Refusal when a publishing switch is reached for while a real aircraft is on the link.
     *
     * Names the hazard rather than the rule: an operator who is told "not allowed" looks for a
     * way round it, and one who is told two aircraft would be sharing one stream does not.
     */
    const val REASON_AIRCRAFT_CONNECTED =
        "An aircraft is connected. Publishing a recording would put two flights on one stream."

    /** A yes, or a no with the sentence to show for it. */
    data class Decision(val allowed: Boolean, val reason: String? = null) {
        companion object {
            val YES = Decision(true)
            fun no(reason: String) = Decision(false, reason)
        }
    }

    /**
     * Whether a recording may be opened now.
     *
     * @param interlockEnabled `Bridge.commandInterlock.enabled` — read fresh, never cached.
     * @param manoeuvreEngaged true when `GuidedStickEngine` is anything but IDLE.
     */
    fun mayReplay(interlockEnabled: Boolean, manoeuvreEngaged: Boolean): Decision = when {
        interlockEnabled -> Decision.no(REASON_COMMANDS_LIVE)
        manoeuvreEngaged -> Decision.no(REASON_MANOEUVRE)
        else -> Decision.YES
    }

    /**
     * Whether the command interlock may be armed now.
     *
     * The only thing that can refuse it here is a replay on screen — every other condition on
     * arming lives in `CommandInterlock` and its dialog, and is none of this file's business.
     */
    fun mayArm(replayActive: Boolean): Decision =
        if (replayActive) Decision.no(REASON_REPLAYING) else Decision.YES

    /**
     * Whether a recording may be **published** — put on the MAVLink link, the Zenoh bus, or both.
     *
     * Rule 3, and rules 1 and 2 asked again. The last two are already true whenever a replay is
     * on screen, because that is what [mayReplay] and [mayArm] between them guarantee — asking
     * them again costs two boolean reads and makes this function total, so a future caller that
     * reaches it by some path those two do not cover is refused rather than trusted.
     *
     * **Order matters and is the same rule the encoder's gates follow**: the first failing input,
     * in the order the hazard is worst. A connected aircraft comes first because it is the one
     * this function exists for and the one whose sentence explains something the operator cannot
     * see; the interlock second, because it is the more dangerous state but is also already
     * impossible here and would be a confusing thing to be told.
     *
     * Switching publishing **off** consults nothing, and there is no method for it to consult —
     * `mayReplay`'s own argument about withdrawal, unchanged: ending a stream can only make the
     * world more honest.
     *
     * @param aircraftConnected `AircraftState.fcConnected` from the **live** source, read fresh.
     *   Never the replayed one — the recording's own `fcConnected` is true for most of any
     *   flight worth replaying, and reading it here would refuse every publication there is.
     */
    fun mayPublish(
        aircraftConnected: Boolean,
        interlockEnabled: Boolean,
        manoeuvreEngaged: Boolean,
    ): Decision = when {
        aircraftConnected -> Decision.no(REASON_AIRCRAFT_CONNECTED)
        interlockEnabled -> Decision.no(REASON_COMMANDS_LIVE)
        manoeuvreEngaged -> Decision.no(REASON_MANOEUVRE)
        else -> Decision.YES
    }
}
