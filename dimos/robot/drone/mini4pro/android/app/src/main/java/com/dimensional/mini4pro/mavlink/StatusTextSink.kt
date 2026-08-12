package com.dimensional.mini4pro.mavlink

import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.Severity
import com.dimensional.mini4pro.command.StatusTexts
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.Statustext

/**
 * The MAVLink end of [Announcer]: one sentence becomes one `STATUSTEXT`.
 *
 * This is the code that used to be written out three times, once inside each of
 * `CommandDispatcher.announce`, `GimbalManager.announce` and `GuidedStickEngine.announce`, and it
 * is **relocated rather than rewritten** — the same builder, the same severity, the same
 * `StatusTexts.clamp`, handed to the same `send` lambda those classes were constructed with. The
 * three copies were already identical; having one is the only difference.
 *
 * ## Why the clamp is here and not in the announcer
 *
 * `STATUSTEXT.text` is a fixed 50-byte `char[50]` with no length prefix, so anything longer is
 * silently cut *on the wire* — an operator reading a truncated error name and searching for the
 * wrong string. That is a fact about this message, not about announcements, so the limit is
 * enforced at this message and a transport without the limit does not inherit it.
 *
 * Note the clamp runs *after* the de-duplication that each announcing class does on its own raw
 * text, which is exactly the order that held before: those classes compared the unclamped string
 * and clamped only when building the message.
 *
 * ## Why the severity mapping is written out
 *
 * `io.dronefleet.mavlink`'s `MavSeverity` exposes no accessor for its wire value (javap,
 * 2026-07-26: eight constants, `values()`, `valueOf()`, nothing else), so any table-driven lookup
 * would have to go through `ordinal` — right for MAV_SEVERITY today and exactly the coincidence
 * that turns a CRITICAL into an INFO the day a dialect is regenerated. `Bridge.healthStatusText`
 * writes its own out for the same reason. The `when` is exhaustive over [Severity] with no
 * `else`, so a third level cannot arrive here without a decision.
 */
class StatusTextSink(
    /** Where the message goes. `Bridge.sendOffMain` in the app; a list in the tests. */
    private val send: (Any) -> Unit,
) : Announcer.Sink {

    override fun say(severity: Severity, text: String) {
        send(
            Statustext.builder()
                .severity(mavSeverity(severity))
                .text(StatusTexts.clamp(text))
                .build()
        )
    }

    private fun mavSeverity(severity: Severity): MavSeverity = when (severity) {
        Severity.CRITICAL -> MavSeverity.MAV_SEVERITY_CRITICAL
        Severity.ERROR -> MavSeverity.MAV_SEVERITY_ERROR
    }
}
