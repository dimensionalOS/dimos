package com.dimensional.mini4pro.simulator

/**
 * **What the simulator's phase means for the command interlock** — the whole of that rule, as
 * pure logic, testable without an `Activity` and without an aircraft.
 *
 * The problem this solves is a three-step dance the operator was doing by hand: arm the
 * interlock, hand control to QGC, then remember to start the simulator — and then, at the end,
 * remember that stopping the simulator does *not* disarm anything. Two of those three steps are
 * bookkeeping and the third is a hazard.
 *
 * ## 1. A confirmed simulator arms the interlock, and holds it only while it is confirmed
 *
 * [effectOf] is an **edge detector on DJI's own report**, not on our request:
 *
 *  - entering [SimulatorPhase.ACTIVE] → [Effect.ENABLE]
 *  - leaving [SimulatorPhase.ACTIVE] → [Effect.DISABLE]
 *  - anything else → [Effect.NONE]
 *
 * ACTIVE is the only phase that means *DJI reports it running and this process asked for it*.
 * [SimulatorPhase.STARTING] is our request and claims nothing, which is why the enable does not
 * hang off the button press.
 *
 * **The disable is the safety-carrying half, and it is strictly safer than what came before.**
 * Until this existed, the interlock survived the simulator: a simulator that quietly died left
 * commands live against a real aircraft, with the screen still showing a green arming switch
 * the operator had armed for a simulation. That failure has a real trigger — DJI delivers
 * `KeyIsSimulatorStarted` as **null** when the component goes away, which
 * `SimulatorControl.onStartedDelivery` correctly refuses to read as "stopped" but which does
 * take the phase out of ACTIVE. Under this rule that same delivery drops the interlock. Under
 * the old one it changed nothing at all.
 *
 * The disable fires whoever armed it, deliberately. An interlock armed by hand *before* the
 * simulator started is still an interlock that is now live over an aircraft whose simulation has
 * gone, and disarming can only ever make things safer — the same asymmetry the arming dialog
 * has, and the same one the abort ladder has.
 *
 * ## 2. The arming dialog, and why this no longer has an opinion about it
 *
 * There was a second rule here — `confirmationRequired(phase) = phase != ACTIVE` — whose whole job
 * was to *suppress* the arming dialog when a confirmed simulator meant the question had no content.
 * On 2026-07-30 the dialog itself went away (Ivan: *"We don't need validation confirmation"*; the
 * reasoning is in `CockpitDefaults.COMMAND_INTERLOCK`), and a rule for skipping a dialog that no
 * longer exists is dead code with tests around it — the shape this project deletes rather than
 * maintains. Its tests went with it.
 *
 * Nothing about §1 changed: the simulator still arms the interlock on entering ACTIVE and, far more
 * importantly, still **disarms on leaving it**. That half was always the safety-carrying one.
 *
 * ## What this does not do
 *
 * It does not enable anything itself. It returns a value; the *caller* acts, and the caller is
 * the UI layer. That is not stylistic — `CommandInterlock`'s third property is that **no inbound
 * MAVLink message can enable it**, and that property is only structural for as long as
 * `enable()` has no caller reachable from a wire. A human pressing the simulator button is a
 * human at the phone; a `SimulatorPhase` arriving on a DJI callback is not, which is exactly why
 * the decision is computed here and applied there.
 */
object SimulatorInterlock {

    /** What a phase change asks of the interlock. */
    enum class Effect {
        /** Nothing to do. Every phase change that neither enters nor leaves ACTIVE. */
        NONE,

        /** DJI has confirmed our simulator. Arm the command path. */
        ENABLE,

        /** The confirmed simulator is gone. Disarm, whoever armed it. */
        DISABLE,
    }

    /**
     * The effect of moving from [previous] to [current].
     *
     * @param previous null before the first phase has ever been observed — a fresh process. A
     *   first observation of ACTIVE is an *entry* and arms, which is the right answer for an app
     *   that has just been opened over a simulator this process started before it was killed…
     *   except that it cannot have, because `SimulatorControl` reports a simulator this process
     *   did not start as [SimulatorPhase.FOREIGN]. So the null case is reachable only as a
     *   genuine first ACTIVE, and treating it as an entry is both correct and unsurprising.
     */
    fun effectOf(previous: SimulatorPhase?, current: SimulatorPhase): Effect = when {
        current == SimulatorPhase.ACTIVE && previous != SimulatorPhase.ACTIVE -> Effect.ENABLE
        previous == SimulatorPhase.ACTIVE && current != SimulatorPhase.ACTIVE -> Effect.DISABLE
        else -> Effect.NONE
    }

    /**
     * The status-text line for why the interlock is where it is, or null when the simulator has
     * nothing to say about it.
     *
     * A drop that happens silently is a drop the operator finds out about from a refused
     * command, which is the worst moment to learn it.
     */
    fun interlockLine(phase: SimulatorPhase, interlockEnabled: Boolean): String? = when {
        phase == SimulatorPhase.ACTIVE && interlockEnabled -> "held by the simulator"
        phase == SimulatorPhase.ACTIVE -> "simulator active, but commands are OFF"
        else -> null
    }

    /** Said once when a confirmed simulator goes away and takes the command path with it. */
    const val DROPPED = "Simulator gone — commands switched off"
}
