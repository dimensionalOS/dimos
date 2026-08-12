package com.dimensional.mini4pro.warn

import com.dimensional.mini4pro.command.StatusTexts
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The operator-facing sentences: the 50-byte field, and the rule that **DJI's word survives and
 * ours does not**.
 *
 * `STATUSTEXT.text` is a fixed-width `char[50]` with no length prefix and no continuation, so
 * anything longer is silently cut on the wire — the operator reads a truncated warning name and
 * searches for a string that does not exist. Every sentence here is generated at runtime from a
 * DJI title of entirely unknown length, so the limit has to be enforced rather than counted by
 * hand.
 *
 * Mutation-checked 2026-07-26, one breakage at a time, failing tests counted across the three
 * health suites and the code reverted after each — measured counts, not estimates. The sentence
 * mutants:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | clamp removed (`essential` returned unclamped; long titles overflow the field) | 3 |
 *  | framing kept in preference to DJI's title when both do not fit | 2 |
 *  | the level word dropped from `changed` (an escalation reads as a duplicate) | 3 |
 *  | UTF-8 clamped by characters rather than bytes (in the shared `StatusTexts.clamp`) | 2 |
 *
 * The blank-title fallback lives on `Warning.name` and is counted in `WarningMonitorTest`'s
 * table; [a_blank_title_falls_back_to_djis_code] is the test that kills it.
 */
class WarnStatusTextsTest {

    private companion object {
        const val CODE = "0x1600A0"

        fun item(level: WarnLevel, title: String?, measurement: String? = null) = Warning(
            source = WarnSource.DEVICE_HEALTH,
            code = CODE,
            state = level.name,
            level = level,
            title = title,
            measurement = measurement,
        )

        fun bytes(s: String) = s.toByteArray(Charsets.UTF_8).size
    }

    @Test
    fun an_appearance_names_djis_title() {
        assertEquals(
            "DJI: Aircraft overheating",
            WarnStatusTexts.appeared(item(WarnLevel.WARNING, "Aircraft overheating")),
        )
    }

    /**
     * The level word is in the sentence **only** for a change, because there it is the news.
     * QGroundControl prints its own severity label in front of every `STATUSTEXT`, so anywhere
     * else it would be printed twice at the cost of DJI's title.
     */
    @Test
    fun a_change_names_the_new_level() {
        assertEquals(
            "DJI SERIOUS_WARNING: Overheat",
            WarnStatusTexts.changed(item(WarnLevel.SERIOUS_WARNING, "Overheat")),
        )
    }

    @Test
    fun a_clear_says_so() {
        assertEquals(
            "DJI cleared: Overheat",
            WarnStatusTexts.cleared(item(WarnLevel.WARNING, "Overheat")),
        )
    }

    /**
     * DJI failed to map its own code, so there is no title. The raw code is still searchable and
     * is a far better thing to send than a friendly sentence we invented — or than the empty
     * string, which is what a missing fallback produces.
     */
    @Test
    fun a_blank_title_falls_back_to_djis_code() {
        for (title in listOf(null, "", "   ")) {
            assertEquals(
                "title=${title?.let { "'$it'" }}",
                "DJI: $CODE",
                WarnStatusTexts.appeared(item(WarnLevel.WARNING, title)),
            )
        }
    }

    /**
     * When the framing plus the title will not fit, **the framing goes**.
     *
     * This is the counter-intuitive half of `command/StatusTexts`' rule and the one a well-meaning
     * edit would reverse: our prefix is what explains where the sentence came from, but the
     * operator can only act on DJI's half.
     */
    @Test
    fun the_framing_is_dropped_before_djis_words_are() {
        // 44 chars: fits alone, does not fit behind "DJI SERIOUS_WARNING: " (21 bytes).
        val title = "Aircraft overheating - land as soon as safe"
        assertTrue(bytes(title) <= StatusTexts.MAX_BYTES)

        val text = WarnStatusTexts.changed(item(WarnLevel.SERIOUS_WARNING, title))

        assertEquals("DJI's whole title, and none of our framing", title, text)
    }

    /** Only when DJI's own title exceeds the field is anything of DJI's cut. */
    @Test
    fun a_title_longer_than_the_field_is_clamped_to_it() {
        val title = "A".repeat(120)

        val text = WarnStatusTexts.appeared(item(WarnLevel.WARNING, title))

        assertEquals(StatusTexts.MAX_BYTES, bytes(text))
        assertTrue(text.all { it == 'A' })
    }

    /**
     * The field is specified in **bytes**, so the clamp counts bytes — and never leaves half a
     * character behind, which a ground station decodes as a replacement glyph rather than as a
     * short string.
     */
    @Test
    fun clamping_never_splits_a_utf8_character() {
        // 3 bytes each in UTF-8; 20 of them is 60 bytes, so the cut lands mid-sequence if the
        // clamp counts characters.
        val title = "温".repeat(20)

        for (text in listOf(
            WarnStatusTexts.appeared(item(WarnLevel.WARNING, title)),
            WarnStatusTexts.changed(item(WarnLevel.WARNING, title)),
            WarnStatusTexts.cleared(item(WarnLevel.WARNING, title)),
        )) {
            assertTrue("'$text' is ${bytes(text)} bytes", bytes(text) <= StatusTexts.MAX_BYTES)
            assertTrue("no replacement characters", text.none { it == '�' })
            assertEquals("whole characters only", text, String(text.toByteArray(Charsets.UTF_8), Charsets.UTF_8))
        }
    }

    /** No composed sentence, for any level and any DJI title, may exceed the field. */
    @Test
    fun no_sentence_ever_exceeds_the_field() {
        val titles = listOf(null, "", "short", "A".repeat(49), "A".repeat(50), "A".repeat(400), "温".repeat(60))
        for (level in WarnLevel.entries) {
            for (title in titles) {
                val i = item(level, title)
                for (text in listOf(
                    WarnStatusTexts.appeared(i),
                    WarnStatusTexts.changed(i),
                    WarnStatusTexts.cleared(i),
                )) {
                    assertTrue(
                        "level=$level title=${title?.length} produced ${bytes(text)} bytes",
                        bytes(text) <= StatusTexts.MAX_BYTES,
                    )
                }
            }
        }
    }

    // ── The description fallback, from the first real messages ───────────────

    /**
     * **DJI sends a description and no title, and this is what makes the feature work.**
     *
     * Measured on hardware 2026-07-27, the first three health messages this project ever received.
     * Every one had a blank `title` and a populated `description`; without the fallback the
     * operator reads a hex code for a message whose entire purpose is to be understood at a glance
     * while an aircraft is misbehaving. The package exists because a warning went unnoticed for an
     * hour, and a warning rendered as `0x16100013` would go unnoticed just as long.
     *
     * The strings below are verbatim from the flight record, not invented.
     *
     * | mutation | tests that fail | measured |
     * |---|---|---|
     * | description fallback removed (`title ?: code`) | this test, and the two below | 3 |
     * | description preferred over a present title | `a title still wins when DJI sends one` | 1 |
     * | first-sentence trim removed (whole description used) | this test (the two-clause cases) | 1 |
     * | `MIN_SENTENCE_CHARS` set to 0 | `an early full stop does not truncate to nothing` | 1 |
     */
    @Test
    fun djis_description_is_used_when_the_title_is_blank() {
        // "Running Flight Simulator. Restart aircraft to take off"
        val sim = Warning(
            source = WarnSource.DEVICE_HEALTH,
            state = "WARNING",
            code = "0x16100013",
            level = WarnLevel.WARNING,
            title = "",
            description = "Running Flight Simulator. Restart aircraft to take off",
        )
        assertEquals("Running Flight Simulator.", sim.name)

        // "Obstacle sensing not available at night. Adjust RTH altitude to above tallest ..."
        val night = Warning(
            source = WarnSource.DEVICE_HEALTH,
            state = "WARNING",
            code = "0x1B030010",
            level = WarnLevel.WARNING,
            description = "Obstacle sensing not available at night. Adjust RTH altitude to " +
                "above tallest surrounding building before flying",
        )
        assertEquals("Obstacle sensing not available at night.", night.name)

        // No sentence break at all — the whole thing, for the clamp to shorten.
        val unbound = Warning(
            source = WarnSource.DEVICE_HEALTH,
            state = "WARNING",
            code = "0x16100089",
            level = WarnLevel.WARNING,
            description = "Remote controller not bound/still confirming binding status",
        )
        assertEquals("Remote controller not bound/still confirming binding status", unbound.name)

        // And every one of them still fits the wire once framed.
        for (item in listOf(sim, night, unbound)) {
            val text = WarnStatusTexts.appeared(item)
            assertTrue(
                "must fit the field: '$text'",
                text.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES,
            )
            assertFalse("the code must not surface when DJI gave words", text.contains("0x"))
        }
    }

    /** DJI's title, when there is one, still outranks the description. */
    @Test
    fun a_title_still_wins_when_dji_sends_one() {
        val item = Warning(
            source = WarnSource.DEVICE_HEALTH,
            state = "WARNING",
            code = "0x1600A0",
            level = WarnLevel.WARNING,
            title = "Aircraft overheating",
            description = "Aircraft overheating. Land as soon as possible",
        )
        assertEquals("Aircraft overheating", item.name)
    }

    /**
     * An early full stop must not reduce a warning to a fragment.
     *
     * An abbreviation or a version number ("v1.2 compass fault…") would otherwise cut the sentence
     * to nothing useful, which is a worse failure than a long line the clamp shortens from the
     * front.
     */
    @Test
    fun an_early_full_stop_does_not_truncate_to_nothing() {
        val item = Warning(
            source = WarnSource.DEVICE_HEALTH,
            state = "WARNING",
            code = "0xDEAD",
            level = WarnLevel.WARNING,
            description = "No. 2 motor temperature high",
        )
        assertEquals("No. 2 motor temperature high", item.name)
    }

    /** With nothing at all from DJI, the searchable code is still the honest answer. */
    @Test
    fun the_code_remains_the_last_resort() {
        val item = Warning(
            source = WarnSource.DEVICE_HEALTH,
            state = "WARNING",
            code = "0xBEEF", level = WarnLevel.WARNING, title = "  ")
        assertEquals("0xBEEF", item.name)
    }
}
