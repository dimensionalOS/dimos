package com.dimensional.mini4pro.command

import org.junit.Assert.assertEquals
import org.junit.Assert.assertSame
import org.junit.Test

/**
 * The fan-out that makes "every interface hears every refusal" a property of the code rather than
 * of somebody's memory.
 *
 * [Announcer] replaced a `send: (Any) -> Unit` in three classes, and the whole reason it exists is
 * a property that **cannot be observed today**: with one sink attached it is indistinguishable
 * from the lambda it replaced, and the existing suites prove that half by continuing to pass
 * unchanged. What this file adds is the other half — that a second sink gets *the same sentence,
 * with the same severity, on the same call* — measured now, while there is nothing at stake, so
 * that the day a second transport attaches the question is already answered.
 *
 * The two things it pins hardest are the two ways a fan-out silently stops being one:
 *
 *  - **delivery to only the first sink**, which looks exactly like working software until the
 *    interface that was attached second is the one an operator is watching;
 *  - **double delivery**, which is what an idempotent `attach` prevents and which a transport
 *    that reconnects would otherwise produce — two identical red lines per refusal, the thing
 *    every announce window in this project exists to avoid.
 *
 * De-duplication and truncation are deliberately *not* tested here, because they are deliberately
 * not here: each announcing class keeps its own window keyed on its own last sentence, and the
 * 50-byte clamp lives at the MAVLink message (`StatusTextSinkTest`). See [Announcer]'s KDoc for
 * why hoisting either would have been a behaviour change wearing a refactor's clothes.
 *
 * Mutation-checked 2026-07-27, one breakage at a time, failing tests counted and the code
 * reverted after each — this file, and the whole suite:
 *
 *  | mutation | this file | whole suite |
 *  |---|---|---|
 *  | `say` reaches only the first attached sink | 4 | 4 |
 *  | `say` reaches only the last attached sink | 4 | 4 |
 *  | `attach` uses `add` instead of `addIfAbsent` (a re-attach doubles every sentence) | 1 | 1 |
 *  | `detach` is a no-op | 1 | 1 |
 *  | `say` passes [Severity.ERROR] regardless of what it was given | 2 | 4 |
 *  | `say` drops the sentence when more than one sink is attached | 4 | 4 |
 *
 * Five of the six rows read the same in both columns, and that is the honest shape of a fan-out
 * with one sink in production: the other 1046 tests **cannot see these mutations at all**, which
 * is exactly why this file has to. The one exception is the severity row, where two tests in
 * `CommandDispatcherTest` notice, because Emergency Stop is the one sentence this project says at
 * `CRITICAL`.
 */
class AnnouncerTest {

    /** One recorded sentence, as a sink saw it. */
    private data class Said(val severity: Severity, val text: String)

    private class Recorder : Announcer.Sink {
        val heard = mutableListOf<Said>()
        override fun say(severity: Severity, text: String) {
            heard += Said(severity, text)
        }
    }

    // ------------------------------------------------------- one sink: today's behaviour

    @Test
    fun `one sink hears the sentence it was given, verbatim`() {
        val sink = Recorder()
        val announcer = Announcer(sink)

        announcer.say(Severity.ERROR, "Return refused by DJI: FC_AUTH_STATE")

        assertEquals(listOf(Said(Severity.ERROR, "Return refused by DJI: FC_AUTH_STATE")), sink.heard)
    }

    @Test
    fun `severity is carried through, not flattened`() {
        val sink = Recorder()
        val announcer = Announcer(sink)

        announcer.say(Severity.CRITICAL, "No motor cut. Hold both RC sticks inner-down")
        announcer.say(Severity.ERROR, "Land sent to DJI")

        assertEquals(listOf(Severity.CRITICAL, Severity.ERROR), sink.heard.map { it.severity })
    }

    /**
     * A sentence composed with nothing listening is dropped, not thrown.
     *
     * The same answer `Bridge.sendOffMain` already gave for a `STATUSTEXT` composed while no link
     * was running — and the reason it matters is that a *decision* must never depend on whether
     * anyone was there to hear it.
     */
    @Test
    fun `no sinks is a no-op`() {
        Announcer().say(Severity.ERROR, "nobody is listening")
    }

    // ------------------------------------------------------------------- the fan-out

    @Test
    fun `every attached sink hears the same sentence on one call`() {
        val first = Recorder()
        val second = Recorder()
        val third = Recorder()
        val announcer = Announcer(first, second)
        announcer.attach(third)

        announcer.say(Severity.CRITICAL, "Goto refused: NO_RC_FEED")

        val expected = listOf(Said(Severity.CRITICAL, "Goto refused: NO_RC_FEED"))
        assertEquals("first sink", expected, first.heard)
        assertEquals("second sink", expected, second.heard)
        assertEquals("third sink", expected, third.heard)
    }

    /**
     * The order every sink sees is the order the announcing class said things in — a sink is a
     * transcript of the session, and a reordered transcript is a different session.
     */
    @Test
    fun `sinks see the sentences in the order they were said`() {
        val a = Recorder()
        val b = Recorder()
        val announcer = Announcer(a, b)

        announcer.say(Severity.ERROR, "Goto started")
        announcer.say(Severity.ERROR, "Goto capped at 60m")
        announcer.say(Severity.CRITICAL, "Guided off: link-lost")

        val expected = listOf("Goto started", "Goto capped at 60m", "Guided off: link-lost")
        assertEquals(expected, a.heard.map { it.text })
        assertEquals(expected, b.heard.map { it.text })
    }

    /**
     * A sink attached twice hears each sentence once.
     *
     * Not hypothetical: a transport that drops and reconnects is the ordinary case, and an
     * `attach` that appended blindly would give the operator two identical red lines per refusal
     * after the first reconnect — the exact failure the three announce windows exist to prevent,
     * reintroduced underneath them where no window can see it.
     */
    @Test
    fun `attaching the same sink twice does not double the sentence`() {
        val sink = Recorder()
        val announcer = Announcer(sink)
        announcer.attach(sink)

        announcer.say(Severity.ERROR, "Takeoff failed: NO_ALT_DATUM")

        assertEquals(1, sink.heard.size)
        assertEquals(1, announcer.sinkCount)
    }

    /**
     * A detached sink hears nothing more, and the others are unaffected — the shape a transport
     * stopping while another keeps running has to have.
     */
    @Test
    fun `a detached sink stops hearing and the rest carry on`() {
        val staying = Recorder()
        val leaving = Recorder()
        val announcer = Announcer(staying, leaving)

        announcer.say(Severity.ERROR, "before")
        announcer.detach(leaving)
        announcer.say(Severity.ERROR, "after")

        assertEquals(listOf("before", "after"), staying.heard.map { it.text })
        assertEquals(listOf("before"), leaving.heard.map { it.text })
        assertEquals(1, announcer.sinkCount)
    }

    /**
     * The sentence handed to each sink is the *same string instance* the announcing class
     * composed: no per-sink transformation happens in the fan-out.
     *
     * Pinned because a transformation here — a prefix naming the transport, a per-sink clamp —
     * would be the natural place to put one, and it would mean two operators reading two
     * different accounts of the same refusal.
     */
    @Test
    fun `no per-sink transformation happens in the fan-out`() {
        val a = Recorder()
        val b = Recorder()
        val announcer = Announcer(a, b)
        val text = "DJI: MOTOR_START_FAIL"

        announcer.say(Severity.ERROR, text)

        assertSame(text, a.heard.single().text)
        assertSame(text, b.heard.single().text)
    }
}
