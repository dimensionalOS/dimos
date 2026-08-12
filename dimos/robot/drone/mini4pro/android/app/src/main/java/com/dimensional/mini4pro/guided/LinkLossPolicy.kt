package com.dimensional.mini4pro.guided

/**
 * What the bridge does with virtual stick when the ground station vanishes mid-engagement —
 * M3 Q4, built exactly as the decision requires: **a swappable policy, not inline logic**.
 *
 * `docs/decisions/2026-07-26-m3-guided-control.md` Q4, Ivan: *"yes stop for now, but we need to
 * be able to iterate on this easily."* So the behaviour lives behind this one small interface,
 * the shipped implementation and the two named alternatives sit side by side in this file, and
 * switching is a one-line change to [SHIPPED] plus a build. It is deliberately **not** runtime-
 * or MAVLink-configurable — same reasoning as the Q1 envelope: changing what happens when the
 * link dies should require a build, not a packet. The armed policy's [name] is logged and
 * recorded at engage time, so a flight log always says which one was in force.
 *
 * A policy only *describes* the terminal sequence; [GuidedStickEngine] executes it. That keeps
 * every implementation a pure value — trivially testable, and impossible for a policy to send
 * anything on its own.
 *
 * ## The argument for the shipped choice, in brief
 *
 * §Q4 of the M2 decisions forbids this bridge acquiring an opinion about what the aircraft
 * should do. Handing authority back is the opposite of that: it removes our opinion and
 * restores the pre-M3 state, in which DJI's own tested failsafes are the only thing flying the
 * aircraft. Continuing to stream a setpoint after the operator has vanished is not neutrality —
 * it is flying an aircraft with nobody watching. The deceleration first is the bounded
 * termination of a command already in progress, in the only direction that reduces energy.
 */
interface LinkLossPolicy {

    /** Logged and recorded at engage time. Class-stable, so a log line names the code. */
    val name: String

    /** The terminal sequence to run. A pure description; the engine executes it. */
    fun plan(): LinkLossPlan

    companion object {
        /**
         * The policy that ships — Ivan's Q4 answer. Changing this line (plus a build) is the
         * entire act of switching policies, which is what "iterate on this easily" bought.
         */
        val SHIPPED: LinkLossPolicy = DecelerateThenHandback()
    }
}

/**
 * One terminal sequence: ramp the commanded velocity to zero over [rampToZeroMs], hold zero for
 * [holdZeroMs] (null = hold forever, i.e. stay engaged at zero velocity — DJI position hold),
 * then, if [release], disable virtual stick and hand the aircraft back to the RC.
 *
 * Invariants a plan cannot express and the engine enforces regardless: the ramp is monotone
 * toward zero (being wrong about the aircraft's state can only make it stop sooner), and a
 * fresh `MANUAL_CONTROL` arriving before the release step has run resumes passthrough — the
 * operator's link came back, and their hand outranks our wind-down.
 */
data class LinkLossPlan(
    val rampToZeroMs: Long,
    val holdZeroMs: Long?,
    val release: Boolean,
)

/**
 * **Shipped.** Decelerate to zero over ~0.5 s, hold zero briefly, then disable virtual stick
 * and give the aircraft back to the RC — the Q4 default, chosen because it ends with DJI's own
 * tested failsafes as the only controller, at zero commanded energy, with no step change.
 */
class DecelerateThenHandback : LinkLossPolicy {
    override val name: String = "DecelerateThenHandback"
    override fun plan(): LinkLossPlan = LinkLossPlan(
        rampToZeroMs = GuidedEnvelope.RAMP_TO_ZERO_MS,
        holdZeroMs = HOLD_MS,
        release = true,
    )

    companion object {
        /** "Hold briefly" from the decision text: long enough to be demonstrably stopped. */
        const val HOLD_MS = 1_000L
    }
}

/**
 * Rejected alternative, kept buildable: ramp to zero and **stay engaged**, holding position
 * indefinitely, so a returning link resumes control without re-engaging. Its cost, from the
 * decision doc: it leaves the bridge holding flight authority through a comms outage.
 */
class FreezeAndHold : LinkLossPolicy {
    override val name: String = "FreezeAndHold"
    override fun plan(): LinkLossPlan = LinkLossPlan(
        rampToZeroMs = GuidedEnvelope.RAMP_TO_ZERO_MS,
        holdZeroMs = null,
        release = false,
    )
}

/**
 * Rejected alternative, kept buildable: no deceleration — release virtual stick immediately.
 * Simplest, and the decision doc's objection stands: handing back at speed with the pilot not
 * expecting it is its own hazard. (Under the Q1 envelope "at speed" is bounded to 3 m/s, which
 * is why this is a defensible alternative rather than an absurd one.)
 */
class InstantHandback : LinkLossPolicy {
    override val name: String = "InstantHandback"
    override fun plan(): LinkLossPlan = LinkLossPlan(
        rampToZeroMs = 0L,
        holdZeroMs = 0L,
        release = true,
    )
}
