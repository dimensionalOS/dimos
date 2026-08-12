package com.dimensional.mini4pro.mavlink

import com.dimensional.mini4pro.command.Severity
import com.dimensional.mini4pro.command.StatusTexts
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.Statustext
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The MAVLink end of the announcer, which is the half of the refactor that had to be a **rename
 * and nothing else**: three classes stopped building a `STATUSTEXT` and this one started, and an
 * operator must not be able to tell.
 *
 * The three copies it replaced were already identical — same builder, same `StatusTexts.clamp`,
 * same severity — so the risk was never that this file gets the message wrong in an interesting
 * way. It is that the *relocation* quietly drops something on the way: the clamp, most obviously,
 * which was inside a call that moved packages.
 *
 * That one is worth spelling out because of how it fails. `STATUSTEXT.text` is a fixed 50-byte
 * `char[50]` with no length prefix, so an over-long sentence is not rejected — it is **silently
 * cut on the wire**, and the operator reads a truncated DJI error name and searches for a string
 * that does not exist. A dropped clamp therefore passes every test that asserts on what we
 * *composed* and fails only on a ground station, in a session, at the moment somebody needed to
 * read the error.
 *
 * Mutation-checked 2026-07-27, one breakage at a time, failing tests counted and the code
 * reverted after each — this file, and the whole suite:
 *
 *  | mutation | this file | whole suite |
 *  |---|---|---|
 *  | the 50-byte clamp dropped (`.text(text)`) | 2 | **2 — see below** |
 *  | `CRITICAL` mapped to `MAV_SEVERITY_ERROR` | 1 | 3 |
 *  | `ERROR` mapped to `MAV_SEVERITY_INFO` (below QGC's display threshold) | 2 | 10 |
 *  | the clamp cuts on bytes without respecting UTF-8 characters | 1 | 1 |
 *  | the sink sends nothing at all | 5 | 115 |
 *
 * The clamp row is the reason this file exists. Dropping it breaks **nothing outside this file** —
 * 1046 tests, including every suite that asserts on what an operator reads, go on passing, because
 * they all assert on sentences that already fit. The cut happens on the wire, in a field with no
 * length prefix, in front of the one person who needed to read the error name. Contrast the last
 * row: a sink that sends nothing is caught 115 times over, because "the operator was told" is what
 * most of this project's suites are about. Loudness of failure is not proportional to danger, and
 * these two rows are the same fact from both ends.
 */
class StatusTextSinkTest {

    private val sent = mutableListOf<Any>()
    private val sink = StatusTextSink { sent.add(it) }

    private fun single(): Statustext = sent.single() as Statustext

    @Test
    fun `a sentence becomes one STATUSTEXT with the text intact`() {
        sink.say(Severity.ERROR, "Return refused by DJI: FC_AUTH_STATE")

        assertEquals(1, sent.size)
        assertEquals("Return refused by DJI: FC_AUTH_STATE", single().text())
    }

    /**
     * The severity table, both rows.
     *
     * `ERROR` is not a taxonomy choice, it is a visibility one: QGC 5.0.8 surfaces only
     * EMERGENCY/ALERT/CRITICAL/ERROR to an operator (`StatusTextHandler.cc:18-24`), so a level
     * below it is an announcement that never happened. `CRITICAL` is reserved for the one
     * sentence an operator may have to act on within seconds.
     */
    @Test
    fun `each severity reaches the wire at the level it was said at`() {
        sink.say(Severity.CRITICAL, "No motor cut. Hold both RC sticks inner-down")
        sink.say(Severity.ERROR, "Land sent to DJI")

        val levels = sent.map { (it as Statustext).severity().entry() }
        assertEquals(
            listOf(MavSeverity.MAV_SEVERITY_CRITICAL, MavSeverity.MAV_SEVERITY_ERROR),
            levels,
        )
    }

    /** Totality: a severity added without a wire level fails here rather than on a ground station. */
    @Test
    fun `every severity has a wire level, and none is below QGC's display threshold`() {
        val visible = setOf(
            MavSeverity.MAV_SEVERITY_EMERGENCY,
            MavSeverity.MAV_SEVERITY_ALERT,
            MavSeverity.MAV_SEVERITY_CRITICAL,
            MavSeverity.MAV_SEVERITY_ERROR,
        )
        for (severity in Severity.values()) {
            sent.clear()
            sink.say(severity, "x")
            assertTrue(
                "$severity would not be shown to an operator",
                single().severity().entry() in visible,
            )
        }
    }

    /**
     * The clamp, at the message where the 50 bytes are.
     *
     * The fixture is a DJI-shaped error name long enough to overflow, because that is the real
     * source of an over-long sentence: `StatusTexts` composes from a DJI string of unknown length
     * and `FCFlightMode` alone has 79 constants.
     */
    @Test
    fun `an over-long sentence is cut to the 50-byte field`() {
        val long = "Return refused by DJI: " + "NAV_SYS_EXCEPTION_WITH_A_VERY_LONG_DJI_NAME"

        sink.say(Severity.ERROR, long)

        val text = single().text()
        assertTrue("must not exceed the field", text.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES)
        assertTrue("must be a prefix of what was said", long.startsWith(text))
    }

    /**
     * Cutting on a byte count must not split a character in half: a lone surrogate encodes to a
     * replacement character on the ground station, which is a decoding failure rather than a short
     * string. DJI's names are ASCII, so this is the case nobody will hit and everybody will
     * assume works.
     */
    @Test
    fun `the cut never splits a UTF-8 character`() {
        val text = "Πτήση αρνήθηκε από το DJI: ΣΦΑΛΜΑ ΣΥΣΤΗΜΑΤΟΣ ΠΛΟΗΓΗΣΗΣ"

        sink.say(Severity.ERROR, text)

        val out = single().text()
        assertTrue(out.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES)
        // Round-tripping through UTF-8 is lossless exactly when no character was cut in half.
        assertEquals(out, String(out.toByteArray(Charsets.UTF_8), Charsets.UTF_8))
        assertTrue(text.startsWith(out))
    }
}
