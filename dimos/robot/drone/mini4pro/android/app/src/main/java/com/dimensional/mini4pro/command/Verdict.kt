package com.dimensional.mini4pro.command

/**
 * What this bridge decided about a command, said without naming the wire it arrived on.
 *
 * ## Why the type exists
 *
 * The decisions in `CommandDispatcher`, `GuidedStickEngine` and `GimbalManager` are already
 * transport-free — plain value types in, plain strings out, no `handshake/` imports, every gate
 * exercised by a unit test with no aircraft and no socket. The one MAVLink-shaped thing left in
 * their signatures was the **return type**: `io.dronefleet.mavlink.common.MavResult`, a generated
 * enum from a MAVLink dialect jar, threaded all the way down to `deny()`.
 *
 * That was harmless while MAVLink was the only interface. It stops being harmless the moment a
 * second one exists, because the natural repair at that point — a second decision path that
 * produces the second transport's verdicts — is exactly the repair that moves the gates.
 * `docs/zenoh-dimos-transport.md` §3.1 states the requirement: after the change there must still
 * be **exactly one** way to move the aircraft, and every property currently guarded must still be
 * guarded by the same code. So the decisions keep their single implementation and lose the wire
 * type instead; each transport maps this enum at its own edge.
 *
 * ## The five values, and why exactly five
 *
 * **A 1:1 image of the `MavResult` values this project actually returns**, and nothing more. The
 * MAVLink enum has more (`IN_PROGRESS`, `CANCELLED`, `COMMAND_LONG_ONLY`, …); none of them is
 * reachable from any decision here, and `HandshakeResponder.registerCommandHandler` explicitly
 * forbids `IN_PROGRESS` — QGC would hold the command pending forever. Copying values nobody
 * returns would invite somebody to start returning one, on a path with no measured QGC behaviour
 * behind it.
 *
 * The mapping to the wire lives at the MAVLink edge, in `handshake/Verdicts.kt`, and is written
 * as an exhaustive `when` with **no `else`**: a sixth value added here cannot be silently
 * absorbed into some default, it stops the build until somebody says what it means on each wire.
 *
 * ## What this type deliberately does not carry
 *
 * No reason string, no severity, no detail. Refusal *reasons* already have a channel that is
 * transport-neutral in exactly the same way — `StatusTexts`, `GuidedStatusTexts` and
 * `GimbalStatusTexts` compose plain sentences — and duplicating the reason into the verdict would
 * create two places for one fact to be right in.
 */
enum class Verdict {
    /**
     * The bridge took the command: it reached the DJI layer, or the target was taken, or the
     * request was already under way and this is a retry of the one that took it.
     *
     * **The dangerous value.** On `MAV_CMD_NAV_TAKEOFF` the resulting `MAV_RESULT_ACCEPTED` is
     * not a report but an instruction — it makes QGC arm on its own initiative
     * (`PX4FirmwarePlugin.cc:307-315`) — which is why `CommandDispatcher.perform` and
     * `GuidedStickEngine.acceptTarget` each keep this to a single `return`.
     */
    ACCEPTED,

    /**
     * Asked, and answered no: a gate refused it, or the aircraft itself refused it.
     *
     * One code for every refusal rather than a taxonomy, for the reason `onTakeoff`'s KDoc gives
     * — QGC renders each result as one canned line and acts on none of them differently, so a
     * finer taxonomy would multiply the ways the table can be got wrong while changing nothing an
     * operator sees. The sentence that accompanies the refusal is where the reason lives.
     */
    DENIED,

    /**
     * There is no such capability here at all — as distinct from "not this time".
     *
     * Reserved for two things and used for nothing else: the interlock being off, where the reply
     * must be byte-for-byte the reply that existed before commands did
     * (`docs/decisions/2026-07-25-m2-command-safety.md` §Q2); and DJI declaring a key
     * unperformable, which is a statement about the airframe rather than about this attempt.
     */
    UNSUPPORTED,

    /**
     * Not now, plausibly in ten seconds — a gimbal with no product connected, a DJI layer not yet
     * attached to a manager that is otherwise working.
     */
    TEMPORARILY_REJECTED,

    /**
     * The attempt itself broke: the layer below threw. Never a success, and deliberately distinct
     * from [DENIED], which claims somebody decided something.
     */
    FAILED,
}
