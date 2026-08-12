package com.dimensional.mini4pro.handshake

import com.dimensional.mini4pro.telemetry.AircraftState
import io.dronefleet.mavlink.common.MavParamType
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.ParamRequestList
import io.dronefleet.mavlink.common.ParamRequestRead
import io.dronefleet.mavlink.common.ParamSet
import io.dronefleet.mavlink.common.ParamValue
import io.dronefleet.mavlink.common.Statustext
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The parameter table once it stopped being only about the bridge.
 *
 * `RTL_RETURN_ALT` is the first PX4 name this project publishes, and it is publishable for
 * exactly one reason: DJI states the value (`FlightControllerKey.KeyGoHomeHeight`). Everything
 * here defends the two halves of that sentence — that we never publish it without the value, and
 * that the value we publish is DJI's rather than ours.
 *
 * The suite is written to fail loudly for the mistakes that would put a wrong number in front of
 * an operator, or let one be entered:
 *
 *  - publishing `RTL_RETURN_ALT` with a placeholder when DJI has not delivered the key (the
 *    fabrication that got the ArduPilot identity reversed, in miniature)
 *  - a table that changes size after it is built (QGC's parameter download never completes,
 *    because `_checkInitialLoadComplete` removes wait-list entries by index)
 *  - freezing the value at connect, so an RTH altitude an operator changed in DJI Fly reads as
 *    the old one
 *  - a later `null` from DJI collapsing the value to 0 — an aircraft that returns at ground level
 *  - accepting a parameter write, or refusing one silently: QGC lets an operator type over any
 *    value it can read, and a refusal nobody sees is how they come to believe a return altitude
 *    the aircraft does not have
 *
 * ## The jar, not the docs
 *
 * `KeyGoHomeHeight` was read out of `dji-sdk-v5-aircraft-provided-5.18.0.jar` rather than out of
 * the reference, per the house rule (the docs have been wrong today —
 * `SimulatorManager.isSimulatorEnabled()` is stubbed to `return false`). From
 * `javap -p -c dji.sdk.keyvalue.key.DJIFlightControllerKey`, offsets 4557-4602:
 *
 * ```
 * 4573: ldc_w   // String GoHomeHeight
 * 4576: getstatic SingleValueConverter.IntegerConverter
 * 4583: canGet(true)  4587: canSet(true)  4591: canListen(true)
 * 4595: canPerformAction(false)  4599: setIsEvent(false)
 * ```
 *
 * The offline doc agrees, and adds the two behavioural caveats worth knowing: the unit is metres
 * and the datum is the **takeoff** point ("related to the altitude when taking off"), where
 * PX4's `RTL_RETURN_ALT` is measured above the **destination/home** point — identical unless home
 * is moved after takeoff. And "if the horizontal distance between aircraft and home point is
 * within 50 meters, the aircraft will ignore the set return-to-home altitude and return at
 * current altitude", which PX4 mirrors with `RTL_MIN_DIST`. Neither makes the number untrue;
 * both are recorded so nobody re-derives them.
 *
 * PX4's side: `PX4ParameterFactMetaData.json` gives `RTL_RETURN_ALT` as `Float`, units `m`,
 * min 0, described as the *minimum* altitude to climb to on return "unless it currently is
 * flying higher already" — the same concept DJI's key configures.
 *
 * ## Mutation-checked 2026-07-26
 *
 * Each breakage was made deliberately, the failing tests counted, and the code reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `forAircraft` publishes `0f` instead of omitting an undelivered parameter | 3 |
 *  | `refreshFromAircraft` short-circuited (value frozen at build time) | 1 |
 *  | `refreshFromAircraft` writes `0f` instead of keeping the last value on a null read | 1 |
 *  | `RTL_RETURN_ALT` declared `writable = true` | 2 |
 *  | `RTL_RETURN_ALT` declared `MAV_PARAM_TYPE_INT32` | 4 |
 *  | reader-name validation dropped from `ParameterStore.init` | 1 |
 *  | `handleParamSet` echoes the *requested* value instead of the current one | 4 |
 *  | the refusal `STATUSTEXT` removed from `handleParamSet` | 4 |
 *  | refusal severity lowered to `MAV_SEVERITY_WARNING` | 2 |
 *  | refusal de-duplication removed | 1 |
 *  | de-duplication keyed on nothing (every refusal after the first suppressed) | 1 |
 *
 * The three mutations that cost only one test each are the ones with a single dedicated test by
 * design — freezing the value, collapsing a null to zero, and losing the de-duplication key are
 * each a specific claim rather than a property several tests happen to touch.
 */
class ParametersTest {

    private val gcsSysId = 255
    private val gcsCompId = 190

    /** DJI's own number, as it would arrive from `KeyGoHomeHeight`: metres, integral. */
    private val rthMetres = 100

    private fun stateWithRth(metres: Int?) = AircraftState(goHomeHeightM = metres)

    // ─────────────────────────────────────────────── publishing from the aircraft

    @Test
    fun `RTL_RETURN_ALT is published with DJI's own value once the key has arrived`() {
        val store = ParameterStore.forAircraft(state = { stateWithRth(rthMetres) })

        val p = store.byName("RTL_RETURN_ALT")!!
        assertEquals(100f, p.value, 0f)
        // PX4's metadata calls this a Float; a mismatch would have QGC decode our bytes with the
        // wrong union member and render a plausible-looking wrong altitude.
        assertEquals(MavParamType.MAV_PARAM_TYPE_REAL32, p.type)
        // REAL32 is the one type the bytewise codec passes through unchanged. Asserted through
        // ParamCodec rather than by hand so this still holds if the type ever moves.
        assertEquals(ParamCodec.encode(p.type, 100f), p.wireValue(), 0f)
        assertEquals(100f, ParamCodec.decode(p.type, p.wireValue()), 0f)
    }

    @Test
    fun `an undelivered key is omitted for the session, never filled in with a default`() {
        val log = mutableListOf<String>()
        val store = ParameterStore.forAircraft(state = { stateWithRth(null) }, log = { log.add(it) })

        // Not present at all. Not 0, not 30 (PX4's default), not 100 (DJI Fly's).
        assertNull(store.byName("RTL_RETURN_ALT"))
        assertEquals(-1, store.indexOf("RTL_RETURN_ALT"))
        assertEquals(
            ParameterStore.DEFAULT_PARAMETERS.map { it.name },
            store.snapshot().map { it.name },
        )
        // And it says so, naming the key, because an absent parameter is otherwise
        // indistinguishable from a bug in this file.
        assertTrue(
            log.toString(),
            log.any {
                it.contains("RTL_RETURN_ALT") &&
                    it.contains("FlightControllerKey.KeyGoHomeHeight") &&
                    it.contains("omitted")
            },
        )
    }

    @Test
    fun `a key that arrives after the store is built does not grow the table`() {
        // The rule QGC's download depends on. `_checkInitialLoadComplete` removes wait-list
        // entries by index against the param_count it was given first; a table that gains an
        // entry mid-session leaves that count one short forever and QGC re-requests until it
        // gives up.
        var live: Int? = null
        val store = ParameterStore.forAircraft(state = { stateWithRth(live) })
        val countAtBuild = store.count

        live = 120 // DJI finally delivers, one second after the link opened

        assertEquals(countAtBuild, store.count)
        assertNull(store.byName("RTL_RETURN_ALT"))
        assertEquals(countAtBuild, store.snapshot().size)
    }

    @Test
    fun `the value tracks the aircraft, so a setting changed in DJI Fly is what QGC re-reads`() {
        var live = 100
        val store = ParameterStore.forAircraft(state = { stateWithRth(live) })
        assertEquals(100f, store.byName("RTL_RETURN_ALT")!!.value, 0f)

        live = 45 // the operator drags the RTH slider on the other screen

        assertEquals(45f, store.byName("RTL_RETURN_ALT")!!.value, 0f)
        assertEquals(45f, store.at(store.indexOf("RTL_RETURN_ALT"))!!.value, 0f)
        assertEquals(45f, store.snapshot().single { it.name == "RTL_RETURN_ALT" }.value, 0f)
    }

    @Test
    fun `DJI going quiet keeps the last stated value rather than collapsing it to zero`() {
        // A null from a KeyManager listener is "the component is gone", not "the setting is now
        // zero". Publishing 0 here would tell an operator this aircraft returns at ground level.
        // Removing the parameter is not available either: param_count cannot shrink mid-session.
        var live: Int? = 100
        val store = ParameterStore.forAircraft(state = { stateWithRth(live) })
        assertEquals(100f, store.byName("RTL_RETURN_ALT")!!.value, 0f)

        live = null

        assertEquals(100f, store.byName("RTL_RETURN_ALT")!!.value, 0f)
    }

    @Test
    fun `every published parameter has a dense unique index and one agreed count`() {
        for (state in listOf(stateWithRth(rthMetres), stateWithRth(null))) {
            val store = ParameterStore.forAircraft(state = { state })
            val names = store.snapshot().map { it.name }
            assertEquals(store.count, names.size)
            assertEquals(names.size, names.distinct().size)
            assertEquals(
                (0 until store.count).toList(),
                names.map { store.indexOf(it) }.sorted(),
            )
            // The BRG_ block keeps its indices whether or not the aircraft one is there, so a
            // reconnect does not renumber what an operator was reading.
            ParameterStore.DEFAULT_PARAMETERS.forEachIndexed { i, p ->
                assertEquals(p.name, store.at(i)!!.name)
            }
        }
    }

    // ─────────────────────────────────────────────────────── what may be borrowed

    @Test
    fun `no aircraft parameter is one we could never state truthfully`() {
        for (p in ParameterStore.AIRCRAFT_PARAMETERS) {
            assertFalse(p.name, p.name in ParameterStore.FORBIDDEN_PARAMETERS)
            assertFalse(p.name, p.name in ParameterStore.PX4_CALIBRATION_PARAMETERS)
            assertTrue(p.name, p.name.length <= ParameterStore.MAX_NAME_LENGTH)
            assertTrue(p.name, ParamCodec.isCarryable(p.type))
            // Publishing a name QGC never asks for buys nothing and borrows a firmware's
            // semantics for free. Every entry must be one QGC actually reported missing.
            assertTrue(p.name, p.name in ParameterStore.PX4_MISSING_PARAMETERS)
        }
    }

    @Test
    fun `the rejected candidates stay rejected`() {
        // Each of these was surveyed against a DJI key on 2026-07-26 and rejected on *meaning*:
        //  - BAT1_SOURCE     PX4 names which hardware measures pack voltage; DJI has no analogue
        //  - RTL_DESCEND_ALT PX4's descend target vs DJI's operator-confirmation height
        //  - RTL_LAND_DELAY  no equivalent key
        //  - NAV_RCL_ACT / NAV_DLL_ACT / COM_LOW_BAT_ACT   failsafe *action enums* whose
        //    encodings differ: a wrong value says the aircraft will Land when it will Return
        //  - COM_RC_LOSS_T   no key
        // If one of these ever appears, it is because someone matched on the name.
        val published = ParameterStore.forAircraft(state = { stateWithRth(rthMetres) })
            .snapshot().map { it.name }
        for (name in listOf(
            "BAT1_SOURCE", "RTL_DESCEND_ALT", "RTL_LAND_DELAY",
            "NAV_RCL_ACT", "NAV_DLL_ACT", "COM_LOW_BAT_ACT", "COM_RC_LOSS_T",
            "GF_MAX_HOR_DIST", "SYS_AUTOSTART", "CAL_GYRO0_ID", "RC_MAP_ROLL",
        )) {
            assertFalse(name, name in published)
        }
    }

    @Test
    fun `the bridge-only table needs no aircraft and carries no aircraft parameter`() {
        val store = ParameterStore.default()
        assertEquals(ParameterStore.DEFAULT_PARAMETERS.map { it.name }, store.snapshot().map { it.name })
        assertTrue(store.snapshot().all { it.name.startsWith("BRG_") })
    }

    @Test(expected = IllegalArgumentException::class)
    fun `a reader for a parameter that is not in the table is rejected`() {
        // Otherwise it is a value nobody can ever see, and a silent typo in a name.
        ParameterStore(
            entries = ParameterStore.DEFAULT_PARAMETERS,
            readers = mapOf<String, (AircraftState) -> Float?>("RTL_RETURN_ALT" to { 1f }),
            state = { AircraftState() },
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun `an aircraft parameter with no state supplier is rejected`() {
        ParameterStore(
            entries = ParameterStore.DEFAULT_PARAMETERS +
                Parameter("BRG_PROTO_VER2", MavParamType.MAV_PARAM_TYPE_INT32, 1f, false, "x"),
            readers = mapOf<String, (AircraftState) -> Float?>("BRG_PROTO_VER2" to { 1f }),
            state = null,
        )
    }

    // ───────────────────────────────────────────────────── the responder's answers

    private class Rig(state: () -> AircraftState, var now: Long = 0L) {
        val sent = mutableListOf<Any>()
        val responder = HandshakeResponder(
            send = { sent.add(it) },
            parameters = ParameterStore.forAircraft(state),
            nowMs = { now },
        )

        fun statusTexts() = sent.filterIsInstance<Statustext>()
        fun paramValues() = sent.filterIsInstance<ParamValue>()
    }

    private fun paramSet(name: String, value: Float, type: MavParamType) =
        ParamSet.builder()
            .targetSystem(1).targetComponent(1)
            .paramId(name)
            .paramValue(ParamCodec.encode(type, value))
            .paramType(type)
            .build()

    private fun paramRequestRead(name: String) =
        ParamRequestRead.builder()
            .targetSystem(1).targetComponent(1)
            .paramId(name)
            .paramIndex(-1)
            .build()

    @Test
    fun `RTL_RETURN_ALT reaches the GCS in the parameter download`() {
        val rig = Rig({ stateWithRth(rthMetres) })
        rig.responder.onMessage(
            ParamRequestList.builder().targetSystem(1).targetComponent(0).build(),
            gcsSysId, gcsCompId,
        )

        val values = rig.paramValues()
        assertEquals(rig.responder.parameters.count, values.size)
        assertTrue(values.all { it.paramCount() == rig.responder.parameters.count })
        val rth = values.single { it.paramId() == "RTL_RETURN_ALT" }
        assertEquals(100f, rth.paramValue(), 0f)
        assertEquals(
            MavParamType.MAV_PARAM_TYPE_REAL32.ordinal + 1,
            rth.paramType().value(),
        )
    }

    @Test
    fun `a write to RTL_RETURN_ALT is refused, and the operator is told in words`() {
        val rig = Rig({ stateWithRth(rthMetres) })

        // QGC's parameter editor lets an operator type any value over a parameter it can read.
        rig.responder.onMessage(
            paramSet("RTL_RETURN_ALT", 250f, MavParamType.MAV_PARAM_TYPE_REAL32),
            gcsSysId, gcsCompId,
        )

        // The echo carries DJI's value, not the requested one. QGC master's ack predicate
        // (`ParameterManager.cc:315-349`) compares the two and discards a mismatch, so this
        // does not complete the write; what it does do is put the true value back in QGC's Fact.
        val echo = rig.paramValues().single()
        assertEquals("RTL_RETURN_ALT", echo.paramId())
        assertEquals(100f, echo.paramValue(), 0f)
        assertEquals(100f, rig.responder.parameters.byName("RTL_RETURN_ALT")!!.value, 0f)

        // And the echo is not the whole answer. On QGC 5.0.8 it satisfied the write wait — that
        // build matched on name only — and the revert was silent. So a sentence goes with it.
        val warning = rig.statusTexts().single()
        assertEquals("Param refused, read-only: RTL_RETURN_ALT", warning.text())
        // ERROR, for the reason measured on the mode-refusal path: only EMERGENCY / ALERT /
        // CRITICAL / ERROR reach the operator (`StatusTextHandler.cc:18-30`, popup at `:241`).
        // At WARNING this is filed in a list nobody opens, which is the same as silence.
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, warning.severity().entry())
    }

    @Test
    fun `re-reading after a refused write returns the aircraft's value, so the revert is visible`() {
        val rig = Rig({ stateWithRth(rthMetres) })
        rig.responder.onMessage(
            paramSet("RTL_RETURN_ALT", 250f, MavParamType.MAV_PARAM_TYPE_REAL32),
            gcsSysId, gcsCompId,
        )
        rig.sent.clear()

        // QGC master re-reads the parameter as the last step of its failed-write state machine
        // (`ParameterManager.cc:429-437`, paramRefreshState).
        rig.responder.onMessage(paramRequestRead("RTL_RETURN_ALT"), gcsSysId, gcsCompId)

        assertEquals(100f, rig.paramValues().single().paramValue(), 0f)
    }

    @Test
    fun `QGC's three-shot PARAM_SET retry produces one warning, not three`() {
        val rig = Rig({ stateWithRth(rthMetres) })
        // kParamSetRetryCount = 2 plus the initial attempt, kWaitForParamValueAckMs = 1000
        // (`ParameterManager.h:109-111`): three transmissions about a second apart.
        repeat(3) {
            rig.responder.onMessage(
                paramSet("RTL_RETURN_ALT", 250f, MavParamType.MAV_PARAM_TYPE_REAL32),
                gcsSysId, gcsCompId,
            )
            rig.now += 1_000
        }

        assertEquals(1, rig.statusTexts().size)
        // Every attempt is still answered on the wire; only the operator-facing sentence is
        // de-duplicated.
        assertEquals(3, rig.paramValues().size)
    }

    @Test
    fun `a different parameter, or the same one later, is a new edit and is announced again`() {
        val rig = Rig({ stateWithRth(rthMetres) })
        rig.responder.onMessage(
            paramSet("RTL_RETURN_ALT", 250f, MavParamType.MAV_PARAM_TYPE_REAL32),
            gcsSysId, gcsCompId,
        )
        rig.responder.onMessage(
            paramSet("BRG_TLM_HZ", 5f, MavParamType.MAV_PARAM_TYPE_INT32),
            gcsSysId, gcsCompId,
        )
        assertEquals(
            listOf("Param refused, read-only: RTL_RETURN_ALT", "Param refused, read-only: BRG_TLM_HZ"),
            rig.statusTexts().map { it.text() },
        )

        rig.now += HandshakeResponder.PARAM_REFUSAL_REPEAT_MS + 1
        rig.responder.onMessage(
            paramSet("BRG_TLM_HZ", 5f, MavParamType.MAV_PARAM_TYPE_INT32),
            gcsSysId, gcsCompId,
        )
        assertEquals(3, rig.statusTexts().size)
    }

    @Test
    fun `a writable parameter whose write is not applied is refused out loud too`() {
        // Unreachable today — nothing is writable and no writer is registered — and asserted
        // anyway, so "a refused write is never silent" survives the first writable parameter.
        val store = ParameterStore(
            listOf(
                Parameter(
                    name = "BRG_TEST_RW",
                    type = MavParamType.MAV_PARAM_TYPE_INT32,
                    value = 1f,
                    writable = true,
                    doc = "writable fixture",
                )
            )
        )
        val out = mutableListOf<Any>()
        val subject = HandshakeResponder(send = { out.add(it) }, parameters = store)
        subject.registerParameterWriter { _, _ -> false } // the writer declines

        subject.onMessage(paramSet("BRG_TEST_RW", 4f, MavParamType.MAV_PARAM_TYPE_INT32), gcsSysId, gcsCompId)

        assertEquals(1f, store.byName("BRG_TEST_RW")!!.value, 0f)
        val warning = out.filterIsInstance<Statustext>().single()
        assertEquals("Param write not applied: BRG_TEST_RW", warning.text())
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, warning.severity().entry())
    }

    @Test
    fun `an accepted write says nothing, because nothing was refused`() {
        val store = ParameterStore(
            listOf(
                Parameter(
                    name = "BRG_TEST_RW",
                    type = MavParamType.MAV_PARAM_TYPE_INT32,
                    value = 1f,
                    writable = true,
                    doc = "writable fixture",
                )
            )
        )
        val out = mutableListOf<Any>()
        val subject = HandshakeResponder(send = { out.add(it) }, parameters = store)
        subject.registerParameterWriter { _, _ -> true }

        subject.onMessage(paramSet("BRG_TEST_RW", 4f, MavParamType.MAV_PARAM_TYPE_INT32), gcsSysId, gcsCompId)

        assertEquals(4f, store.byName("BRG_TEST_RW")!!.value, 0f)
        assertTrue(out.filterIsInstance<Statustext>().isEmpty())
    }

    @Test
    fun `a write to a parameter we do not publish stays silent`() {
        // Unchanged behaviour, restated because the refusal above must not have widened it: an
        // unknown name gets no PARAM_VALUE (that would assert the setting exists) and no
        // STATUSTEXT (we have nothing to refuse — it is not ours).
        val rig = Rig({ stateWithRth(rthMetres) })
        rig.responder.onMessage(
            paramSet("SYS_AUTOSTART", 4001f, MavParamType.MAV_PARAM_TYPE_INT32),
            gcsSysId, gcsCompId,
        )
        assertTrue(rig.sent.isEmpty())
    }

    @Test
    fun `every refusal fits the STATUSTEXT field, for the longest name we could ever declare`() {
        // 50 bytes is a fixed-width char[50] with no length prefix: anything longer is silently
        // cut on the wire, and the operator reads a truncated parameter name.
        val longest = "X".repeat(ParameterStore.MAX_NAME_LENGTH)
        for (prefix in listOf(
            HandshakeResponder.PARAM_REFUSAL_READ_ONLY,
            HandshakeResponder.PARAM_REFUSAL_NOT_APPLIED,
        )) {
            val text = HandshakeResponder.paramRefusalText(prefix, longest)
            assertTrue(text, text.toByteArray().size <= 50)
        }
        // 40 bytes for the one that will actually be sent.
        assertEquals(
            40,
            HandshakeResponder
                .paramRefusalText(HandshakeResponder.PARAM_REFUSAL_READ_ONLY, "RTL_RETURN_ALT")
                .toByteArray().size,
        )
    }

    // ────────────────────────────────────────────────── rebinding at link open

    @Test
    fun `rebinding swaps the table a GCS will download`() {
        // Bridge builds the aircraft table when a link opens, because that is the first moment
        // the MSDK has had a chance to deliver and no GCS is listening yet.
        val responder = HandshakeResponder(send = {})
        val before = responder.parameters.count
        assertNull(responder.parameters.byName("RTL_RETURN_ALT"))

        responder.rebindParameters(ParameterStore.forAircraft({ stateWithRth(rthMetres) }))

        assertEquals(before + 1, responder.parameters.count)
        assertEquals(100f, responder.parameters.byName("RTL_RETURN_ALT")!!.value, 0f)
    }

    @Test
    fun `a rebind with nothing from the aircraft leaves the bridge table intact`() {
        val responder = HandshakeResponder(send = {})
        val before = responder.parameters.snapshot().map { it.name }

        responder.rebindParameters(ParameterStore.forAircraft({ AircraftState() }))

        assertEquals(before, responder.parameters.snapshot().map { it.name })
    }
}
