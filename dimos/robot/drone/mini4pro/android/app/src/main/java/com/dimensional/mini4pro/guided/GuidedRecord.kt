package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.record.CommandSource
import com.dimensional.mini4pro.record.Setpoint
import com.dimensional.mini4pro.record.StickAxes
import com.dimensional.mini4pro.record.StickModes
import com.dimensional.mini4pro.record.StickRange

/**
 * The engine's line into the flight recorder, as an interface so the engine stays free of
 * `Recorder`'s Android and DJI imports and a test can assert exactly what would have been
 * recorded.
 *
 * The value types are `record/`'s own (plain Kotlin, no Android, no DJI — `LogEntry.kt`), so
 * nothing is translated on the way through: what the engine hands this hook is byte-for-byte
 * what `Recorder.stickCmd` writes, which is what `tools/flightlog --diagnose-axis` reads.
 * `Bridge` wires it to `Recorder` with the project's containment rule — an evidence problem
 * must never become a control problem, so the production adapter swallows recorder throws.
 */
interface GuidedRecord {

    /** One virtual-stick command, immediately after the SDK call — `Recorder.stickCmd`'s contract. */
    fun stickCmd(
        setpoint: Setpoint?,
        axes: StickAxes,
        modes: StickModes,
        source: CommandSource?,
        range: StickRange?,
        accepted: Boolean?,
        error: String?,
    )

    /** A discrete engagement-lifecycle event — `Recorder.event`'s contract. */
    fun event(code: String, message: String? = null, warn: Boolean = false)

    /** For construction without a recorder; every call is a no-op. */
    object None : GuidedRecord {
        override fun stickCmd(
            setpoint: Setpoint?,
            axes: StickAxes,
            modes: StickModes,
            source: CommandSource?,
            range: StickRange?,
            accepted: Boolean?,
            error: String?,
        ) = Unit

        override fun event(code: String, message: String?, warn: Boolean) = Unit
    }
}
