package com.dimensional.mini4pro.light

import com.dimensional.mini4pro.command.Verdict

/**
 * **Every decision the bottom light makes.** The counterpart of `command/MsdkFlightActions` and
 * `gimbal/MsdkGimbalAim`, and the class a fake [LightPort] can drive in a plain JVM test.
 *
 * ## Why this is not behind the command interlock
 *
 * `command/CommandInterlock` gates the things that **move the aircraft** — Return, Land, the
 * emergency stop — because those are commands that a link fault, a stale QGC, or a misread packet
 * could turn into an unplanned flight. A lamp is not in that category. It cannot translate the
 * aircraft, cannot end a flight, and its worst failure is that the operator has to press the other
 * button.
 *
 * Holding it to the interlock anyway would have a real cost: the interlock starts **off** every
 * session by design, so a light behind it would be unavailable at exactly the moment it is wanted
 * — an operator setting up after dark, before arming anything. Making a harmless control share a
 * safety gate with dangerous ones also erodes the gate, because it gives a reason to leave the
 * gate open that has nothing to do with flying.
 *
 * ## What it refuses
 *
 * **A mode the wire does not name.** `AuxiliaryLight.fromParam` returns null for anything that is
 * not one of the four, and that becomes a [Verdict.DENIED] rather than a write. The reason is the
 * one this project keeps meeting: DJI's `AuxiliaryLightMode` has an `UNKNOWN` member, so an
 * unrecognised number *could* be mapped to something the SDK accepts, and then the aircraft would
 * be asked for a mode nobody chose.
 *
 * **[AuxiliaryLight.UNKNOWN] itself**, which is unreachable from the wire by construction and
 * refused here as well. It is a thing the aircraft can *tell* us, never a thing we can ask for,
 * and the two are different directions of the same word.
 *
 * ## What it does not promise
 *
 * [setMode] returns [Verdict.ACCEPTED] when DJI took the write, which is not the same as the lamp
 * being lit — the same distinction `MsdkGimbalAim` draws between a rotation asked for and a camera
 * that moved. The only honest answer to "is it on?" is [observed], which is what the aircraft last
 * reported through [LightPort.listenMode].
 */
class LightControl(
    private val port: LightPort,
    private val log: (String) -> Unit = {},
) {

    private val gate = Any()
    private var listening = false
    private var lastReported: AuxiliaryLight? = null

    /**
     * The mode the aircraft last reported, or null if it has never said.
     *
     * Null is *not* [AuxiliaryLight.UNKNOWN]: "the aircraft has not told us" and "the aircraft
     * told us a mode this SDK cannot name" are different facts and a caller may care which.
     */
    fun observed(): AuxiliaryLight? = synchronized(gate) { lastReported }

    /**
     * Commands the light.
     *
     * The subscription is planted on the first command rather than at construction, the way
     * `MsdkGimbalAim.ensureListening` does it and for the same reason: MSDK refuses a subscription
     * made before registration completes, silently and with no error, so subscribing lazily at the
     * first moment the operator has demonstrably got a working link is the shape that survives.
     */
    fun setMode(mode: AuxiliaryLight): Verdict {
        if (mode == AuxiliaryLight.UNKNOWN) {
            // Unreachable from the wire; see the class KDoc. Kept because this is a public method
            // and UNKNOWN is a member of the enum a caller could hand us.
            log("light refused: UNKNOWN is something the aircraft says, not something to ask for")
            return Verdict.DENIED
        }
        port.unavailableReason()?.let {
            log("light unavailable: $it")
            return Verdict.DENIED
        }
        ensureListeningLocked()
        port.setMode(
            mode,
            onSuccess = { log("light set to $mode, accepted by DJI") },
            onFailure = { error -> log("light set to $mode failed: $error") },
        )
        // DJI took the write. Whether the lamp is lit is answered by [observed], later.
        return Verdict.ACCEPTED
    }

    /** For `Bridge.stop()`, which calls it unconditionally so no listener outlives the link. */
    fun stop() {
        synchronized(gate) {
            listening = false
            lastReported = null
        }
        port.cancelListens()
    }

    private fun ensureListeningLocked() {
        synchronized(gate) {
            if (listening) return
            listening = true
        }
        port.listenMode { delivered ->
            synchronized(gate) { lastReported = delivered }
        }
    }
}
