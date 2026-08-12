package com.dimensional.mini4pro.command

import java.util.concurrent.CopyOnWriteArrayList

/**
 * The one way a decision in this project reaches a human — and, from here on, the one way it
 * reaches *every* interface a human might be watching.
 *
 * ## What it replaces, and why that was not good enough
 *
 * `CommandDispatcher`, `GuidedStickEngine` and `GimbalManager` each took a
 * `send: (Any) -> Unit` and built a `STATUSTEXT` into it. That is a MAVLink shape in three
 * classes whose decisions are otherwise transport-free, and it has a failure mode that only
 * appears when a second interface exists: a refusal composed here would go to whichever link the
 * lambda happened to point at, and a second transport would be told about a refusal only if
 * somebody remembered to wire it up — per sentence, forever.
 *
 * `docs/zenoh-dimos-transport.md` §4.4 wants the opposite property: *"a QGC operator must not be
 * blind to a DiMOS goto"*, and the reverse. Routing every sentence through one fan-out makes that
 * **structural** rather than a rule people follow. Every refusal, every `GOTO_STARTED`, every
 * disengage reason and every simulator notice already passes through these three classes, so all
 * of them reach a new transport the day it attaches a sink — and none of them can be forgotten,
 * because there is nowhere else for them to go.
 *
 * ## What it deliberately does not do
 *
 * **No de-duplication.** Each of the three classes suppresses its own identical text over its own
 * window, keyed on its own last announcement, and those windows are load-bearing in ways that
 * differ (`CommandDispatcher.ACTION_REPEAT_MS` collapses QGC's 3× `SET_MODE` burst;
 * `GimbalManager.ANNOUNCE_REPEAT_MS` collapses a 10 Hz drag; `GuidedStickEngine`'s collapses a
 * 25 Hz stream against a refusing gate). Hoisting them into one shared window here would merge
 * three independent suppressions into one and change which sentences an operator sees. They stay
 * exactly where they are.
 *
 * **No truncation.** `STATUSTEXT`'s 50-byte field is a MAVLink fact, not a fact about
 * announcements, so the clamp lives in the MAVLink sink where the field is. A transport with no
 * such limit must not inherit somebody else's.
 *
 * **No containment.** A throwing sink propagates, exactly as the `send` lambda did before this
 * class existed. That is the current behaviour preserved rather than a considered position: with
 * one sink it cannot matter, and with two it wants a decision (which transport's failure may cost
 * the other its sentence?) that belongs with the transport that raises the question.
 *
 * Sinks are held in a [CopyOnWriteArrayList] because [say] is called from the `mavlink-rx` thread,
 * the guided engine's 10 Hz thread and DJI's own callback threads, while [attach] happens on
 * whichever thread starts a transport.
 */
class Announcer(vararg sinks: Sink) {

    /**
     * One interface's way of saying a sentence. The MAVLink one builds a `STATUSTEXT`; a Zenoh one
     * would publish a `std_msgs.String` on `dimos/status`.
     */
    fun interface Sink {
        fun say(severity: Severity, text: String)
    }

    private val attached = CopyOnWriteArrayList<Sink>(sinks.toList())

    /** How many sinks are listening. For status display and tests; nothing branches on it. */
    val sinkCount: Int get() = attached.size

    /** Idempotent: attaching the same sink twice does not double every sentence. */
    fun attach(sink: Sink) {
        attached.addIfAbsent(sink)
    }

    fun detach(sink: Sink) {
        attached.remove(sink)
    }

    /**
     * Says one sentence on every attached interface.
     *
     * With no sinks this is a no-op, which is the same answer `Bridge.sendOffMain` already gave
     * for a `STATUSTEXT` composed while no link was running: a sentence with nowhere to go is
     * dropped with the decision it describes unaffected.
     */
    fun say(severity: Severity, text: String) {
        for (sink in attached) sink.say(severity, text)
    }
}

/**
 * How loudly to say it — the two levels this project's command layer actually uses.
 *
 * A **1:1 image of the `MAV_SEVERITY` values these three classes announce at**, on the same
 * principle as [Verdict]: copy what is returned, not what the enum offers. The full MAVLink
 * taxonomy has eight levels; the command layer uses two, and the mapping to the wire is an
 * exhaustive `when` at the MAVLink sink so a third cannot be added without a decision about what
 * it means there.
 *
 * Both of these are above QGC's display threshold, and that is the whole reason there are only
 * two. QGC 5.0.8 surfaces only EMERGENCY/ALERT/CRITICAL/ERROR to an operator
 * (`StatusTextHandler.cc:18-24`), so anything below [ERROR] is an announcement that never
 * happened — a fact recorded in three separate KDocs in this project and worth not re-learning.
 *
 * Note the *device-health* path is not this one: `WarningMonitor` has its own severity table, with
 * WARNING and INFO in it, because health items are forwarded on their own rules and are not
 * command verdicts. It is untouched here.
 */
enum class Severity {
    /** Reserved for the one sentence an operator may have to act on within seconds. */
    CRITICAL,

    /** Everything else this layer says. The taxonomy is abused for visibility, knowingly. */
    ERROR,
}
