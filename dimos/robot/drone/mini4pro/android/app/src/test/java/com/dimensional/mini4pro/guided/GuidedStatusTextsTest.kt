package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.StatusTexts
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Stage A's operator-facing sentences: exact texts pinned (they are the Q5 announcement
 * surface — the *only* sign the operator gets, since the heartbeat never claims a mode), and
 * every composition proven to fit `STATUSTEXT`'s hard 50-byte field.
 */
class GuidedStatusTextsTest {

    private fun bytes(s: String) = s.toByteArray(Charsets.UTF_8).size

    @Test
    fun `fixed texts are pinned and fit the field`() {
        assertEquals("Virtual stick engaged - GCS sticks live", GuidedStatusTexts.ENGAGED)
        assertEquals("Virtual stick resumed - GCS sticks live", GuidedStatusTexts.RESUMED)
        assertEquals("Sticks ignored: interlock off", GuidedStatusTexts.IGNORED_INTERLOCK)
        assertEquals("Sticks ignored: center sticks to engage", GuidedStatusTexts.CENTER_FIRST)
        assertEquals("Sticks ignored: no RC stick feed", GuidedStatusTexts.NO_RC_FEED)
        assertEquals("Sticks refused: unreadable axes", GuidedStatusTexts.BAD_AXES)
        assertEquals("Ceiling 100m - climb blocked", GuidedStatusTexts.CEILING)
        assertEquals("No altitude - climb blocked", GuidedStatusTexts.NO_ALTITUDE)
        for (t in listOf(
            GuidedStatusTexts.ENGAGED, GuidedStatusTexts.RESUMED,
            GuidedStatusTexts.IGNORED_INTERLOCK, GuidedStatusTexts.CENTER_FIRST,
            GuidedStatusTexts.NO_RC_FEED, GuidedStatusTexts.BAD_AXES,
            GuidedStatusTexts.CEILING, GuidedStatusTexts.NO_ALTITUDE,
        )) {
            assertTrue("'$t' is ${bytes(t)} bytes", bytes(t) <= StatusTexts.MAX_BYTES)
        }
    }

    @Test
    fun `every disengage reason produces a within-field sentence`() {
        for (reason in GuidedStickEngine.DisengageReason.entries) {
            val plain = GuidedStatusTexts.off(reason.wire)
            assertTrue("'$plain' is ${bytes(plain)} bytes", bytes(plain) <= StatusTexts.MAX_BYTES)
            assertTrue(plain.startsWith("Virtual stick off: "))
            assertTrue(plain.endsWith(reason.wire))
        }
    }

    @Test
    fun `authority disengage carries DJI's reason name verbatim`() {
        assertEquals(
            "Virtual stick off: authority NEAR_BOUNDARY",
            GuidedStatusTexts.off("authority", "NEAR_BOUNDARY"),
        )
        // DJI's longest reason name today still fits whole.
        val longest = GuidedStatusTexts.off("authority", "BATTERY_SUPER_LOW_LANDING")
        assertEquals("Virtual stick off: authority BATTERY_SUPER_LOW_LANDING".take(50), longest)
        assertTrue(bytes(longest) <= StatusTexts.MAX_BYTES)
    }

    @Test
    fun `an oversized detail is clamped with the reason category intact`() {
        val text = GuidedStatusTexts.off("authority", "A".repeat(80))
        assertTrue(bytes(text) <= StatusTexts.MAX_BYTES)
        assertTrue("the category must survive the clamp", text.startsWith("Virtual stick off: authority"))
    }

    @Test
    fun `stopping announces the wind-down with its reason`() {
        assertEquals("Virtual stick stopping: link-lost", GuidedStatusTexts.stopping("link-lost"))
        for (reason in GuidedStickEngine.DisengageReason.entries) {
            assertTrue(bytes(GuidedStatusTexts.stopping(reason.wire)) <= StatusTexts.MAX_BYTES)
        }
    }

    @Test
    fun `refused prefers the full sentence and falls back to DJI's word alone`() {
        assertEquals("Virtual stick refused: NO_CONFIRM", GuidedStatusTexts.refused("NO_CONFIRM"))
        // Long DJI error: the framing is dropped, DJI's word survives (clamped).
        val longError = "SOME_EXTREMELY_LONG_DJI_ERROR_NAME_THAT_KEEPS_GOING_AND_GOING"
        val text = GuidedStatusTexts.refused(longError)
        assertTrue(bytes(text) <= StatusTexts.MAX_BYTES)
        assertTrue("DJI's word has first claim on the bytes", text.startsWith("SOME_EXTREMELY_LONG"))
    }

    @Test
    fun `ignored composes the port's reason verbatim`() {
        assertEquals("Sticks ignored: NO_PRODUCT", GuidedStatusTexts.ignored("NO_PRODUCT"))
        assertTrue(bytes(GuidedStatusTexts.ignored("SDK_NOT_REGISTERED")) <= StatusTexts.MAX_BYTES)
    }

    @Test
    fun `the Q5 reason words are exactly the decided set`() {
        // Q5's six, plus Stage A's judgement calls (timeout, stopped — `docs/m3-stage-a.md`
        // JC-4/JC-10), Stage B's no-fix (a position feed dead past its grace mid-reposition,
        // `docs/m3-stage-b.md`), Stage D's tag-lost (the descent's sensor dead past its own
        // grace, `docs/m3-stage-d-tag-descent.md` — the no-fix argument applied to the tag
        // pipeline) and Stage C's touchdown (motors-off during a full autoland — the one
        // *success* in the set, 2026-07-28). Growing this set is a documented decision, never
        // a drive-by.
        val words = GuidedStickEngine.DisengageReason.entries.map { it.wire }.toSet()
        assertEquals(
            setOf(
                "released", "sticks", "interlock", "authority", "link-lost", "idle",
                "timeout", "no-fix", "tag-lost", "touchdown", "stopped",
            ),
            words,
        )
    }

    @Test
    fun `the stage C sentences are pinned and fit the field`() {
        assertEquals("Tag autoland armed - to touchdown", GuidedStatusTexts.DESCENT_ARMED_AUTOLAND)
        assertEquals("DJI landing on tag - sticks cancel", GuidedStatusTexts.DESCENT_DJI_LANDING)
        assertEquals("Touchdown - autoland complete", GuidedStatusTexts.DESCENT_TOUCHDOWN)
        assertEquals("Stop landing sent to DJI", GuidedStatusTexts.DESCENT_STOP_SENT)
        assertEquals("DJI never took the landing - holding", GuidedStatusTexts.DESCENT_DJI_TIMEOUT)
        for (t in listOf(
            GuidedStatusTexts.DESCENT_ARMED_AUTOLAND,
            GuidedStatusTexts.DESCENT_DJI_LANDING,
            GuidedStatusTexts.DESCENT_TOUCHDOWN,
            GuidedStatusTexts.DESCENT_STOP_SENT,
            GuidedStatusTexts.DESCENT_DJI_TIMEOUT,
        )) {
            assertTrue("'$t' is ${bytes(t)} bytes", bytes(t) <= StatusTexts.MAX_BYTES)
        }
        // The commit-refusal sentence carries DJI's word and stays inside the field.
        val refused = GuidedStatusTexts.autolandCommitFailed("GPS_DISCONNECT")
        assertEquals("Autoland commit failed: GPS_DISCONNECT", refused)
        assertTrue(bytes(refused) <= StatusTexts.MAX_BYTES)
    }

    @Test
    fun `REFUSAL BY NAME - every distance refusal quotes the bound it was refused against`() {
        // Q1's rule, and the reason `big1.plan`'s refusal became a decision instead of a mystery:
        // the operator was told *which* leg and *what* the limit was, so they could argue with the
        // number. A refusal that says only "too far" cannot be acted on and cannot be argued with.
        //
        // Asserted on the **rendered** sentence against the constant, not by comparing the constant
        // with itself: drop the interpolation from either string and this goes red.
        val limit = GuidedEnvelope.MAX_REPOSITION_DISTANCE_M.toInt().toString()
        assertEquals("beyond 2000m limit", GuidedStatusTexts.REASON_TOO_FAR)
        assertEquals("circle beyond 2000m", GuidedStatusTexts.REASON_ORBIT_TOO_FAR)
        for (text in listOf(GuidedStatusTexts.REASON_TOO_FAR, GuidedStatusTexts.REASON_ORBIT_TOO_FAR)) {
            assertTrue("'$text' must name $limit", text.contains(limit))
            // Still inside the field once the framing is wrapped round it — four digits where
            // there used to be three, which is the kind of growth that silently truncates.
            assertTrue(bytes(GuidedStatusTexts.refused(text)) <= StatusTexts.MAX_BYTES)
            assertTrue(bytes(GuidedStatusTexts.orbitRefused(text)) <= StatusTexts.MAX_BYTES)
        }
        // The radius band names both of its numbers for the same reason.
        assertEquals("radius 5-50m only", GuidedStatusTexts.REASON_ORBIT_RADIUS)
    }
}
