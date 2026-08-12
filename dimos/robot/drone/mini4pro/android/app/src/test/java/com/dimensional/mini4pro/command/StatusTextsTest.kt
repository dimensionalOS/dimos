package com.dimensional.mini4pro.command

import org.junit.Assert.assertFalse
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The 50-byte field, and the rule about whose words get cut when it overflows.
 *
 * `STATUSTEXT.text` is a fixed `char[50]`. Overflow is not an error — it is a silent truncation
 * on the wire, so the operator reads half an error name and searches for the wrong string.
 */
class StatusTextsTest {

    /**
     * The three DJI refusals the flight recorder actually captured, in order, during the 19:16
     * session on 2026-07-25 (`docs/decisions/2026-07-25-m2-command-safety.md` §Q3).
     */
    private val measuredDjiErrors = listOf("FC_AUTH_STATE", "GPS_DISCONNECT", "NAV_SYS_EXCEPTION")

    @Test
    fun `every measured DJI refusal fits the long form, for every action`() {
        FlightAction.entries.forEach { action ->
            measuredDjiErrors.forEach { error ->
                val text = StatusTexts.refusal(action, error)
                assertTrue("$action/$error: $text", text.toByteArray().size <= 50)
                // The long form: framing *and* DJI's word. If a future action label pushed one
                // of these over, this fails rather than quietly dropping the framing.
                assertTrue("$action/$error: $text", text.contains(action.label))
                assertTrue("$action/$error: $text", text.contains(error))
                assertTrue("$action/$error: $text", text.contains("DJI"))
            }
        }
    }

    // ------------------------------------------------------------------ takeoff

    @Test
    fun `the takeoff height notice names both numbers and fits`() {
        // The measured frame: a 3 m request from a vehicle reporting 103.2 m, i.e. param7 = 106.2.
        val text = StatusTexts.takeoffHeight(3.0, CommandDispatcher.DJI_TAKEOFF_HEIGHT_M)

        assertEquals("Takeoff: DJI goes to 1.2m, not 3.0m", text)
        assertEquals(35, text.toByteArray().size)
    }

    @Test
    fun `the height notice fits across QGC's whole takeoff slider`() {
        // PX4 does not override minimumTakeoffAltitudeMeters, so the slider runs from
        // FirmwarePlugin.h:204's 3.048 m to FlyView.SettingsGroup.json's 121.92 m default max.
        // Every value in between must produce a sentence the operator receives whole; a silent
        // truncation here would cut the requested height off the end, which is the one number
        // this sentence exists to deliver.
        listOf(3.048, 10.0, 99.9, 100.0, 121.92).forEach { requested ->
            val text = StatusTexts.takeoffHeight(requested, CommandDispatcher.DJI_TAKEOFF_HEIGHT_M)
            assertTrue("$requested: $text", text.toByteArray().size <= 50)
            assertTrue("$requested: $text", text.contains("1.2m"))
        }
    }

    @Test
    fun `the height notice uses a decimal point in every locale`() {
        // The app runs in Greece. `String.format` without an explicit Locale renders 3,0 there,
        // and "not 3,0m" is a sentence an operator has to stop and parse at the moment they are
        // watching an aircraft leave the ground.
        val previous = java.util.Locale.getDefault()
        try {
            java.util.Locale.setDefault(java.util.Locale.forLanguageTag("el-GR"))
            assertEquals(
                "Takeoff: DJI goes to 1.2m, not 3.0m",
                StatusTexts.takeoffHeight(3.0, CommandDispatcher.DJI_TAKEOFF_HEIGHT_M),
            )
        } finally {
            java.util.Locale.setDefault(previous)
        }
    }

    @Test
    fun `the out-of-range refusal names the offending height and fits`() {
        val text = StatusTexts.takeoffAltOutOfRange(300.0, CommandDispatcher.MAX_TAKEOFF_HEIGHT_M)

        assertEquals("Takeoff failed: 300.0m not in 0-121m", text)
        assertTrue(text.toByteArray().size <= 50)
    }

    /**
     * **A negative height is a datum disagreement, and the sentence says so.**
     *
     * This is the refusal Ivan actually met in the air on 2026-07-28 at 09:46:26, where the old
     * text read "Takeoff failed: -49.5m not in 0-121m" — true, and useless. A negative result can
     * only mean the GCS composed `param7` against an altitude datum below the one we published,
     * so the operator's move is to make the GCS re-read home, and that is what the number now
     * points at.
     */
    @Test
    fun `a negative height blames the datum rather than printing a bare number`() {
        val text = StatusTexts.takeoffAltOutOfRange(-49.5, CommandDispatcher.MAX_TAKEOFF_HEIGHT_M)

        assertEquals("Takeoff failed: GCS datum 49.5m below ours", text)
        assertTrue(text.toByteArray().size <= 50)
        // The magnitude is the operator's only clue to how far off the datum is; a sign-mangled
        // "-49.5m below ours" would read as the opposite of the truth.
        assertFalse("the sentence must not carry a minus sign", text.contains("-"))
        // And the too-high branch must keep saying what it said.
        assertTrue(
            StatusTexts.takeoffAltOutOfRange(300.0, CommandDispatcher.MAX_TAKEOFF_HEIGHT_M)
                .contains("not in 0-")
        )
    }

    @Test
    fun `the arm explanation fits, and says what DJI does instead`() {
        assertEquals(42, StatusTexts.NO_SEPARATE_ARM.toByteArray().size)
        assertTrue(StatusTexts.NO_SEPARATE_ARM.toByteArray().size <= StatusTexts.MAX_BYTES)
        // The operator's question after a refused arm is "so how do the motors start?" — and the
        // answer has to be in this sentence, because it is the only one they get.
        assertTrue(StatusTexts.NO_SEPARATE_ARM.contains("takeoff"))
    }

    @Test
    fun `the takeoff reason codes fit inside the unavailable form`() {
        listOf(
            CommandDispatcher.NO_ALT_DATUM,
            CommandDispatcher.ALT_NOT_A_NUMBER,
            MsdkFlightActions.SIMULATOR_REQUIRED,
            MsdkFlightActions.CANNOT_PERFORM_ACTION,
        ).forEach { reason ->
            val text = StatusTexts.unavailable(FlightAction.TAKEOFF, reason)
            assertTrue("$reason: $text", text.toByteArray().size <= 50)
            // The long form, framing included: these reasons are ours rather than DJI's, so
            // losing "Takeoff failed:" would leave a bare word with no subject.
            assertTrue("$reason: $text", text.startsWith("Takeoff failed: "))
            assertTrue("$reason: $text", text.contains(reason))
        }
    }

    @Test
    fun `the worst measured case is the longest one, and it still fits`() {
        // "Return" is the longer of the two labels and NAV_SYS_EXCEPTION the longest measured
        // name, so this is the tightest real combination. Counted, not estimated.
        val text = StatusTexts.refusal(FlightAction.RETURN_TO_HOME, "NAV_SYS_EXCEPTION")
        assertEquals("Return refused by DJI: NAV_SYS_EXCEPTION", text)
        assertEquals(40, text.toByteArray().size)
    }

    @Test
    fun `our framing is dropped before DJI's word is cut`() {
        // The rule from §Q3: the operator should read DJI's own name for the refusal. When the
        // whole sentence will not fit, our wording is the expendable half.
        val long = "SOME_VERY_LONG_DJI_ERROR_NAME_THAT_WILL_NOT_FIT"
        val text = StatusTexts.refusal(FlightAction.RETURN_TO_HOME, long)

        assertTrue(text.toByteArray().size <= 50)
        assertEquals("DJI's name survives intact", long, text)
    }

    @Test
    fun `a DJI name too long for the field alone is cut, never dropped`() {
        val absurd = "A".repeat(80)
        val text = StatusTexts.refusal(FlightAction.LAND, absurd)

        assertEquals(50, text.toByteArray().size)
        // A prefix of the name, so it is still recognisable and still searchable. Silence would
        // be the alternative and it is worse.
        assertTrue(absurd.startsWith(text))
    }

    @Test
    fun `refused and failed are different verbs on purpose`() {
        // "Refused" attributes a decision to the flight controller. If we never reached the
        // flight controller there was no decision, and claiming one would invent it.
        assertTrue(StatusTexts.refusal(FlightAction.LAND, "GPS_DISCONNECT").contains("refused"))
        assertTrue(StatusTexts.unavailable(FlightAction.LAND, "NO_AIRCRAFT_LINK").contains("failed"))
        assertTrue(
            StatusTexts.threw(FlightAction.LAND, IllegalStateException("boom")).contains("failed"),
        )
    }

    @Test
    fun `a thrown exception with no message still names something`() {
        // A TODO() or a bare throw is what an unwritten DJI layer looks like. "Land failed:"
        // with nothing after it would tell the operator less than the class name does.
        val text = StatusTexts.threw(FlightAction.LAND, NullPointerException())
        assertTrue(text, text.contains("NullPointerException"))
        assertTrue(text.toByteArray().size <= 50)
    }

    @Test
    fun `nothing this object can produce exceeds the field, for any input`() {
        val nasty = listOf(
            "",
            " ",
            "X",
            "A".repeat(49),
            "A".repeat(50),
            "A".repeat(51),
            "A".repeat(500),
            "ошибка_полёта_" + "я".repeat(60), // multi-byte, in case DJI ever localises
            "🚀".repeat(40),          // surrogate pairs, 4 bytes each
        )
        FlightAction.entries.forEach { action ->
            // Composed from the action label alone, so it has no runtime input to overflow — but
            // a long label added later would, and this catches that.
            assertTrue(
                StatusTexts.dispatched(action),
                StatusTexts.dispatched(action).toByteArray().size <= StatusTexts.MAX_BYTES,
            )
            nasty.forEach { error ->
                listOf(
                    StatusTexts.refusal(action, error),
                    StatusTexts.unavailable(action, error),
                    StatusTexts.djiError(error),
                    StatusTexts.clamp(error),
                ).forEach { text ->
                    assertTrue(
                        "$action + ${error.length} chars → ${text.toByteArray().size} bytes",
                        text.toByteArray().size <= StatusTexts.MAX_BYTES,
                    )
                }
            }
        }
    }

    @Test
    fun `clamping never splits a UTF-8 character in half`() {
        // A half character at the end of a char[50] is a decoding failure on the ground station,
        // which is a worse outcome than a shorter string.
        val rockets = "🚀".repeat(20) // 4 bytes each: 12 fit, 13 do not
        val clamped = StatusTexts.clamp(rockets)

        assertEquals(48, clamped.toByteArray().size)
        // Round-trips: no replacement characters, so nothing was cut mid-character.
        assertEquals(clamped, String(clamped.toByteArray(Charsets.UTF_8), Charsets.UTF_8))
        assertTrue(rockets.startsWith(clamped))
    }

    @Test
    fun `text that already fits is returned untouched`() {
        assertEquals("FC_AUTH_STATE", StatusTexts.clamp("FC_AUTH_STATE"))
        assertEquals(50, StatusTexts.clamp("A".repeat(50)).length)
    }

    @Test
    fun `the dispatch announcement is pinned, for both actions`() {
        // Pinned like MODE_REFUSAL_TEXT and LANDING_CONFIRMED. Added 2026-07-26 after a real
        // press produced no operator feedback whatsoever
        // (docs/measurements/2026-07-26-m2-first-command.md).
        assertEquals("Return sent to DJI", StatusTexts.dispatched(FlightAction.RETURN_TO_HOME))
        assertEquals("Land sent to DJI", StatusTexts.dispatched(FlightAction.LAND))
        assertEquals(18, StatusTexts.dispatched(FlightAction.RETURN_TO_HOME).toByteArray().size)
        assertEquals(16, StatusTexts.dispatched(FlightAction.LAND).toByteArray().size)
    }

    @Test
    fun `the dispatch announcement never claims the aircraft is complying`() {
        // The whole reason this sentence is allowed to exist. It reports what the bridge did with
        // the request; nothing on this side of the seam knows what the aircraft decided, and
        // ActionOutcome's KDoc forbids naming an outcome after a state we cannot observe. A
        // future edit to something friendlier — "Returning home", "RTH engaged" — is the exact
        // lie the project is built to avoid, so it fails here.
        val claims = listOf(
            "returning", "landing", "engaged", "started", "complying", "success", "ok",
            "accepted", "confirmed", "underway", "en route", "coming home", "descending",
        )
        FlightAction.entries.forEach { action ->
            val text = StatusTexts.dispatched(action).lowercase()
            claims.forEach { claim ->
                assertTrue("'$text' must not claim '$claim'", !text.contains(claim))
            }
            // What it must say instead: who did it, and that it was only a send.
            assertTrue(text, text.contains("sent"))
            assertTrue(text, text.contains("dji"))
            assertTrue(text, StatusTexts.dispatched(action).contains(action.label))
        }
    }

    @Test
    fun `dispatch and refusal name who acted, and disagree about it`() {
        // "sent to DJI" against "refused by DJI": read together the operator always knows which
        // side of the seam the sentence is about. If both ever became "by DJI" the announcement
        // would start reading as DJI's own acknowledgement, which is what it must never be.
        assertTrue(StatusTexts.dispatched(FlightAction.RETURN_TO_HOME).contains("sent to DJI"))
        assertTrue(
            StatusTexts.refusal(FlightAction.RETURN_TO_HOME, "FC_AUTH_STATE")
                .contains("refused by DJI"),
        )
    }

    @Test
    fun `the landing-confirmed sentence is pinned against the 50-byte field`() {
        // Pinned like MODE_REFUSAL_TEXT: the KDoc claims 25 bytes, counted, and this fails if
        // anyone reworsens it past the field.
        assertEquals("Landing confirmed at 0.5m", StatusTexts.LANDING_CONFIRMED)
        assertEquals(25, StatusTexts.LANDING_CONFIRMED.toByteArray().size)
    }
}
