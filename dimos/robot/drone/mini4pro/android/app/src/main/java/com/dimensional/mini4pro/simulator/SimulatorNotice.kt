package com.dimensional.mini4pro.simulator

import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.Statustext
import io.dronefleet.mavlink.util.EnumValue

/**
 * Everything an operator or a ground station is told about the simulator, in one place, as pure
 * functions of a [SimulatorPhase].
 *
 * Split out of [SimulatorControl] for the same reason `StatusTexts` is split out of
 * `CommandDispatcher`: the exact words are a safety property, and a safety property that lives
 * inside a state machine is one nobody writes a test for.
 *
 * ## What goes on the wire, and the three candidates that were weighed
 *
 * The requirement is `PLAN.md`'s honesty boundary — *a simulated aircraft reported to a ground
 * station as real is exactly the class of lie this project forbids* — under the constraint that
 * whatever we send must be **true**. Three mechanisms exist. Only one survives both halves.
 *
 * ### Rejected: `MAV_MODE_FLAG_HIL_ENABLED` in the heartbeat's `base_mode`
 *
 * Superficially the designed-for answer, and it is the wrong one twice over.
 *
 * *It is not true of us.* MAVLink defines bit 0x20 as *"hardware in the loop simulation. All
 * motors / actuators are blocked, but internal software is full operational."* Under DJI's
 * simulator the flight controller runs its own loop against a simulated airframe and reports
 * `SimulatorState.areMotorsOn` — DJI's own words for that getter are *"Return whether the
 * aircraft motor in the simulator has started to spin."* **Whether the physical motors turn is
 * UNVERIFIED on this airframe** (§ `docs/simulator.md`), so "all motors are blocked" is a
 * sentence we cannot currently assert. Sending a spec-defined flag whose spec text we cannot
 * stand behind is fabrication even when the intent is honest.
 *
 * *And it warns nobody.* Searched across `ref/qgroundcontrol` at master `da14fad28`,
 * `MAV_MODE_FLAG_HIL_ENABLED` appears exactly once, in `Vehicle::hilMode()`
 * (`src/Vehicle/Vehicle.h:540`), and `hilMode()` has exactly one caller:
 * `PX4AutoPilotPlugin::vehicleComponents()` (`src/AutoPilotPlugins/PX4/PX4AutoPilotPlugin.cc:62`),
 * where its only effect is to **omit the Sensors page** from the vehicle setup screen. There is
 * no banner, no indicator, no flight-view change, nothing in QML. So the flag would remove a
 * setup page and tell the operator nothing — worse than nothing, since a reader of this code
 * would believe the warning had been delivered. *(master; unchecked against the 5.0.8 binary
 * Ivan flies, and `docs/measurements/2026-07-26-qgc-master-mode-sweep.md` proves the two differ —
 * but the argument does not depend on the QGC side: the flag is rejected on truth grounds first.)*
 *
 * ### Rejected: overriding the heartbeat's `system_status`
 *
 * `MAV_STATE` has no member meaning "simulating". `TelemetryEncoder.systemStatus` currently
 * reports `ACTIVE` when the motors are on or the aircraft is flying, and under the simulator both
 * of those are things the flight controller genuinely reports about itself. Replacing a true
 * `ACTIVE` with some other state to smuggle in a warning would make the field say something false
 * about the aircraft in order to say something true about the setup. The heartbeat is left
 * exactly as it is.
 *
 * ### Chosen: a repeating `STATUSTEXT`
 *
 * It is the only channel that can carry the actual sentence rather than a flag whose meaning
 * somebody else defined, and the sentence is unconditionally true at every instant it is sent —
 * it goes out only when `KeyIsSimulatorStarted` has *delivered* `true`
 * ([SimulatorControl.noticeIfDue]), never on our own start request.
 *
 * **Repeating, not once on connect**, because a one-shot has three ways to reach nobody: QGC
 * connecting before the simulator started, an operator opening QGC mid-session, and QGC's message
 * panel scrolling the line away. Every [SimulatorControl.NOTICE_PERIOD_MS] the claim is re-made,
 * so it is true *now* rather than true once. At 5 s that is about 12 bytes per second.
 *
 * `MAV_SEVERITY_CRITICAL`, matching `CommandDispatcher`'s emergency-stop notice rather than its
 * routine ones: every number on the operator's screen is fabricated, which is the most misleading
 * condition this bridge can be in.
 *
 * **What the absence of this message does not prove.** It proves only that this bridge has not
 * observed a running simulator — not that the aircraft is real. Before the first
 * `KeyIsSimulatorStarted` delivery the phase is [SimulatorPhase.UNKNOWN] and nothing is sent. The
 * app screen shows `UNKNOWN` for exactly that window; the wire has no way to express it, because
 * "I might be lying to you" is not a claim a telemetry stream can usefully make at 1 Hz.
 */
object SimulatorNotice {

    /**
     * 47 bytes, counted, pinned by a test — `STATUSTEXT.text` is a hard 50-byte `char[50]` with
     * no continuation (`command/StatusTexts.kt`), and an over-long warning is silently cut on the
     * wire rather than rejected.
     */
    const val WIRE_ACTIVE = "SIMULATOR ACTIVE - telemetry is not real flight"

    /** 41 bytes. The state that needs a human to work out what started it. */
    const val WIRE_FOREIGN = "SIMULATOR ON - not started by this bridge"

    /** `STATUSTEXT.text` field width in bytes. Same constant as `command/StatusTexts.MAX_BYTES`. */
    const val MAX_WIRE_BYTES = 50

    /** Flight-recorder event codes. Kept here rather than grown into `record/LogEntry.kt`. */
    const val EVENT_PHASE = "sim_phase"
    const val EVENT_REQUEST = "sim_request"

    /** Wraps a notice body in the message QGC will show. */
    fun statusText(text: String): Statustext = Statustext.builder()
        .severity(EnumValue.of(MavSeverity.MAV_SEVERITY_CRITICAL))
        .text(text)
        .build()

    /**
     * The one line readable at arm's length, with its background colour, or null when the
     * simulator has nothing to say.
     *
     * Null for [SimulatorPhase.OFF] and for [SimulatorPhase.UNKNOWN]. `OFF` because a banner that
     * is always present is a banner nobody reads; `UNKNOWN` because it is the normal state of the
     * first second of every session, and a warning that cries wolf every launch trains the
     * operator to ignore the one that matters. `UNKNOWN` is still shown in the text block below
     * the banner, where it costs no attention.
     *
     * Deliberately outranks every existing banner state in `MainActivity`. "AIRCRAFT CONNECTED —
     * telemetry only" is not false under the simulator, but it is the wrong sentence: it invites
     * the operator to believe the numbers.
     */
    fun banner(phase: SimulatorPhase): Pair<String, Int>? = when (phase) {
        SimulatorPhase.ACTIVE ->
            "⬤ SIMULATOR ACTIVE — TELEMETRY IS NOT REAL" to COLOUR_ACTIVE
        SimulatorPhase.STOPPING ->
            "⬤ SIMULATOR STILL RUNNING — STOP NOT CONFIRMED" to COLOUR_ACTIVE
        SimulatorPhase.FOREIGN ->
            "⬤ SIMULATOR ON — NOT STARTED BY THIS APP" to COLOUR_FOREIGN
        SimulatorPhase.STARTING ->
            "⬤ SIMULATOR STARTING — DJI HAS NOT CONFIRMED" to COLOUR_PENDING
        SimulatorPhase.OFF, SimulatorPhase.UNKNOWN -> null
    }

    /**
     * The `simulator:` line in the status dump. Always present, in every phase, including off —
     * the text block is where an operator goes to check, and a missing line reads as a missing
     * feature rather than as "no simulator".
     */
    fun statusLine(snapshot: SimulatorControl.Snapshot): String = when (snapshot.phase) {
        SimulatorPhase.UNKNOWN -> "UNKNOWN — DJI has not reported KeyIsSimulatorStarted"
        SimulatorPhase.OFF -> "off — DJI reports not started"
        SimulatorPhase.STARTING -> "STARTING — asked DJI, nothing confirmed"
        SimulatorPhase.ACTIVE -> "ACTIVE — started here${forSuffix(snapshot.activeForMs)}"
        SimulatorPhase.FOREIGN ->
            "ON, NOT STARTED BY THIS BRIDGE${forSuffix(snapshot.activeForMs)}"
        SimulatorPhase.STOPPING -> "STOPPING — asked DJI, still reported running"
    }

    /**
     * The simulated aircraft's own reported state, or null when there is nothing to show.
     *
     * Position is printed with its axes unnamed on purpose: DJI documents `getPositionX` as *"the
     * X-axis coordinate of the aircraft in the simulator"* and states neither origin nor unit
     * (`tools/djidoc ISimulatorManager_SimulatorState`), so labelling them north/east/down would
     * be inventing a frame.
     */
    fun aircraftLine(aircraft: SimulatedAircraft?): String? {
        val a = aircraft ?: return null
        val motors = when (a.motorsOn) { true -> "spinning"; false -> "off"; null -> "?" }
        val flying = when (a.flying) { true -> "flying"; false -> "grounded"; null -> "?" }
        return "motors $motors, $flying, xyz=${num(a.positionX)}/${num(a.positionY)}/" +
            "${num(a.positionZ)} (frame undocumented)"
    }

    /**
     * The simulator button's label and whether it is pressable, from the observed phase.
     *
     * Here rather than in `MainActivity` because it is the same category of thing as the banner:
     * a control that offers "Start simulator" while one is already running, or that offers to
     * stop a foreign one without saying it is foreign, is a display error with consequences.
     *
     * Disabled while a request is in flight — one press is one action, the rule
     * `CommandDispatcher` already enforces against QGC's triple-`SET_MODE` burst.
     *
     * **Every running phase's label states the condition before it offers the action** — "⬤
     * SIMULATOR ON — tap to stop", not "Stop simulator". A control labelled with what it *does*
     * is read only by someone already looking for it; one labelled with what is *true* is read by
     * someone glancing at the screen for another reason. Until [banner] is wired into
     * `MainActivity.renderBanner` this label is the loudest simulator warning on the phone, so it
     * has to carry the state rather than the verb.
     */
    fun buttonLabel(phase: SimulatorPhase): Pair<String, Boolean> = when (phase) {
        SimulatorPhase.OFF -> "Start simulator" to true
        // Offered, because refusing to act on a state we cannot see would leave an operator with
        // no way forward on a link that never delivered the flag. DJI gets the last word.
        SimulatorPhase.UNKNOWN -> "Start simulator (state unknown)" to true
        SimulatorPhase.STARTING -> "Starting simulator…" to false
        SimulatorPhase.ACTIVE -> "⬤ SIMULATOR ON — tap to stop" to true
        SimulatorPhase.FOREIGN -> "⬤ SIMULATOR ON, not started here — tap to stop" to true
        SimulatorPhase.STOPPING -> "⬤ SIMULATOR ON — stopping…" to false
    }

    private fun num(v: Double?): String = v?.let { String.format("%.1f", it) } ?: "?"

    private fun forSuffix(ms: Long?): String = ms?.let { ", ${it / 1000}s" } ?: ""

    /** Deep purple: not red (commands live), not amber, not green. A category of its own. */
    const val COLOUR_ACTIVE = 0xFF6A1B9A.toInt()

    /** Brighter, because a simulator nothing here started is the worse of the two. */
    const val COLOUR_FOREIGN = 0xFFAD1457.toInt()

    /** Muted: we have asked and DJI has not answered, which is not yet a claim. */
    const val COLOUR_PENDING = 0xFF4A148C.toInt()
}
