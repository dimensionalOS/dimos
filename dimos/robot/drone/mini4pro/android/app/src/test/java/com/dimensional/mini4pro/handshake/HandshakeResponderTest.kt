package com.dimensional.mini4pro.handshake

import io.dronefleet.mavlink.MavlinkConnection
import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.AutopilotVersion
import io.dronefleet.mavlink.common.CommandAck
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.Statustext
import io.dronefleet.mavlink.common.CommandInt
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.ExtendedSysState
import io.dronefleet.mavlink.common.FileTransferProtocol
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavFrame
import io.dronefleet.mavlink.common.MavLandedState
import io.dronefleet.mavlink.common.MavMissionResult
import io.dronefleet.mavlink.common.MavMissionType
import io.dronefleet.mavlink.common.MavMode
import io.dronefleet.mavlink.common.MavParamType
import io.dronefleet.mavlink.common.MavProtocolCapability
import io.dronefleet.mavlink.common.MavResult
import io.dronefleet.mavlink.common.MavVtolState
import io.dronefleet.mavlink.common.MissionAck
import io.dronefleet.mavlink.common.MissionClearAll
import io.dronefleet.mavlink.common.MissionCount
import io.dronefleet.mavlink.common.MissionRequest
import io.dronefleet.mavlink.common.MissionRequestInt
import io.dronefleet.mavlink.common.MissionRequestList
import io.dronefleet.mavlink.common.ParamRequestList
import io.dronefleet.mavlink.common.ParamRequestRead
import io.dronefleet.mavlink.common.ParamSet
import io.dronefleet.mavlink.common.ParamValue
import io.dronefleet.mavlink.common.RequestDataStream
import io.dronefleet.mavlink.common.SetMode
import io.dronefleet.mavlink.util.EnumValue
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.ByteArrayInputStream
import java.io.ByteArrayOutputStream

/**
 * Drives [HandshakeResponder] with the requests QGroundControl actually sends (sysid 255,
 * compid 190, addressed to system 1) and asserts on what goes back on the wire.
 *
 * The PX4 sections are grounded in a measured capture rather than only in QGC's source: a
 * PX4-identifying fake vehicle was put on the wire against the QGC on this laptop (5.0.8) and
 * every inbound frame logged. That is where the `SET_MODE` base/custom values, the retry counts
 * and the absence of a `_HASH_CHECK` request in 5.0.8 come from.
 */
class HandshakeResponderTest {

    private val sent = mutableListOf<Any>()
    private val responder = HandshakeResponder(send = { sent.add(it) })

    private val gcsSysId = 255
    private val gcsCompId = 190

    // ------------------------------------------------------------------ params

    @Test
    fun `param request list enumerates every parameter with a consistent count and index`() {
        responder.onMessage(paramRequestList(), gcsSysId, gcsCompId)

        val values = sent.filterIsInstance<ParamValue>()
        val expectedCount = responder.parameters.count
        assertEquals(expectedCount, values.size)
        // Every message must agree on the total, or QGC's wait list never empties.
        assertTrue(values.all { it.paramCount() == expectedCount })
        // Indices are 0-based, dense and unique.
        assertEquals((0 until expectedCount).toList(), values.map { it.paramIndex() }.sorted())
        assertEquals(expectedCount, values.map { it.paramId() }.distinct().size)
        // The name/index pairing matches the store's own view.
        values.forEach { assertEquals(it.paramIndex(), responder.parameters.indexOf(it.paramId())) }
    }

    @Test
    fun `param request list can be answered twice`() {
        responder.onMessage(paramRequestList(), gcsSysId, gcsCompId)
        val first = sent.filterIsInstance<ParamValue>().map { it.paramId() to it.paramIndex() }
        sent.clear()

        // QGC re-requests the whole list on retry and on a manual parameter refresh.
        responder.onMessage(paramRequestList(), gcsSysId, gcsCompId)
        val second = sent.filterIsInstance<ParamValue>().map { it.paramId() to it.paramIndex() }

        assertEquals(first, second)
        assertEquals(responder.parameters.count, second.size)
    }

    @Test
    fun `param request list addressed to another vehicle is ignored`() {
        responder.onMessage(
            ParamRequestList.builder().targetSystem(2).targetComponent(1).build(),
            gcsSysId,
            gcsCompId,
        )
        assertTrue(sent.isEmpty())
    }

    @Test
    fun `arming require is absent from the parameter set`() {
        val names = responder.parameters.snapshot().map { it.name }
        assertFalse("ARMING_REQUIRE" in names)
        assertNull(responder.parameters.byName("ARMING_REQUIRE"))
    }

    @Test(expected = IllegalArgumentException::class)
    fun `arming require cannot be added to a parameter store`() {
        ParameterStore(
            listOf(
                Parameter(
                    name = "ARMING_REQUIRE",
                    type = MavParamType.MAV_PARAM_TYPE_INT32,
                    value = 1f,
                    doc = "must be rejected",
                )
            )
        )
    }

    // --------------------------------------------------- PX4's _HASH_CHECK

    @Test
    fun `a hash check request is met with silence, never an invented hash`() {
        // QGC master asks for this by name with param_index -1 (ParameterManager.cc:899-921) as
        // the first step of its PX4 parameter download. Answering with any value sends QGC into
        // _tryCacheHashLoad, which CRC32s ParamCache/1_1.v2 — a file any real PX4 vehicle
        // previously connected as system 1 component 1 also wrote. On a match QGC displays that
        // file's parameters as ours and tells us to stop sending. Silence costs one second
        // (kHashCheckTimeoutMs) and fabricates nothing.
        responder.onMessage(
            ParamRequestRead.builder()
                .targetSystem(1).targetComponent(1)
                .paramId("_HASH_CHECK")
                .paramIndex(-1)
                .build(),
            gcsSysId, gcsCompId,
        )

        assertTrue("a _HASH_CHECK answer claims a parameter set we do not have", sent.isEmpty())
    }

    @Test
    fun `hash check is absent from the parameter set`() {
        val names = responder.parameters.snapshot().map { it.name }
        assertFalse("_HASH_CHECK" in names)
        assertNull(responder.parameters.byName("_HASH_CHECK"))
    }

    @Test(expected = IllegalArgumentException::class)
    fun `hash check cannot be added to a parameter store`() {
        // Publishing it would also wedge the download outright: QGC's _HASH_CHECK branch returns
        // before the wait-list bookkeeping, so the index we gave it is never retired and
        // _checkInitialLoadComplete never fires.
        ParameterStore(
            listOf(
                Parameter(
                    name = "_HASH_CHECK",
                    type = MavParamType.MAV_PARAM_TYPE_UINT32,
                    value = 0f,
                    doc = "must be rejected",
                )
            )
        )
    }

    // ------------------------------- the PX4 parameters QGC asks for and we refuse

    @Test
    fun `not one parameter QGC reports missing is published`() {
        // The measured 2026-07-25 dialog against QGC 5.0.8. Publishing any of these to silence
        // it is the exact failure that got the ArduPilot identity reversed — with the difference
        // that under PX4 nothing is gated on them, so there is no functional pressure at all.
        val published = responder.parameters.snapshot().map { it.name }.toSet()
        val leaked = ParameterStore.PX4_MISSING_PARAMETERS.filter { it in published }
        assertEquals("PX4 setup parameters must not be invented to quiet a dialog", emptyList<String>(), leaked)
        assertEquals(30, ParameterStore.PX4_MISSING_PARAMETERS.size)
        assertEquals(
            ParameterStore.PX4_MISSING_PARAMETERS.size,
            ParameterStore.PX4_MISSING_PARAMETERS.distinct().size,
        )
        // The one name a bare connect reports before any setup page loads must be in the list
        // and must be forbidden — it is the head of the cascade.
        assertTrue(ParameterStore.PX4_MISSING_ON_BARE_CONNECT in ParameterStore.PX4_MISSING_PARAMETERS)
        assertTrue(ParameterStore.PX4_MISSING_ON_BARE_CONNECT in ParameterStore.FORBIDDEN_PARAMETERS)
    }

    @Test
    fun `calibration and RC mapping parameters cannot be published even deliberately`() {
        // These are the subset no future SDK reading could make true: sensor calibration ids,
        // RC channel mappings, the PX4 airframe id and the flight-mode switch assignment.
        assertTrue(ParameterStore.PX4_CALIBRATION_PARAMETERS.isNotEmpty())
        ParameterStore.PX4_CALIBRATION_PARAMETERS.forEach { name ->
            assertTrue("$name must be forbidden", name in ParameterStore.FORBIDDEN_PARAMETERS)
            try {
                ParameterStore(
                    listOf(
                        Parameter(
                            name = name,
                            type = MavParamType.MAV_PARAM_TYPE_INT32,
                            value = 1f,
                            doc = "must be rejected",
                        )
                    )
                )
                throw AssertionError("$name was accepted into a ParameterStore")
            } catch (expected: IllegalArgumentException) {
                // correct
            }
        }
    }

    @Test
    fun `the aircraft settings QGC asks for are merely absent, not banned`() {
        // RTH altitude and friends describe behaviour a DJI aircraft genuinely has. They are
        // legitimate to publish once KeyManager actually reports them — and only then. Banning
        // them would block honest work later; seeding them with plausible defaults would be the
        // same fabrication as the calibration ids.
        listOf(
            "RTL_RETURN_ALT", "RTL_DESCEND_ALT", "RTL_LAND_DELAY", "BAT1_SOURCE",
            "NAV_RCL_ACT", "NAV_DLL_ACT", "COM_RC_LOSS_T", "COM_LOW_BAT_ACT", "MAV_SYS_ID",
        ).forEach { name ->
            assertTrue("$name should be in the measured list", name in ParameterStore.PX4_MISSING_PARAMETERS)
            assertFalse("$name must not be banned outright", name in ParameterStore.FORBIDDEN_PARAMETERS)
            assertNull("$name must not be published today", responder.parameters.byName(name))
        }
    }

    @Test
    fun `param request read by index answers that parameter`() {
        val expected = responder.parameters.at(1)!!

        responder.onMessage(
            ParamRequestRead.builder()
                .targetSystem(1).targetComponent(1)
                .paramId("")
                .paramIndex(1)
                .build(),
            gcsSysId, gcsCompId,
        )

        val value = sent.filterIsInstance<ParamValue>().single()
        assertEquals(expected.name, value.paramId())
        assertEquals(1, value.paramIndex())
        assertEquals(responder.parameters.count, value.paramCount())
    }

    @Test
    fun `param request read by name answers that parameter`() {
        val expected = responder.parameters.at(0)!!

        responder.onMessage(
            ParamRequestRead.builder()
                .targetSystem(1).targetComponent(1)
                .paramId(expected.name)
                .paramIndex(-1) // MAVLink's "look me up by name"
                .build(),
            gcsSysId, gcsCompId,
        )

        val value = sent.filterIsInstance<ParamValue>().single()
        assertEquals(expected.name, value.paramId())
        assertEquals(0, value.paramIndex())
    }

    @Test
    fun `param request read for an unknown parameter is not answered with an invented value`() {
        responder.onMessage(
            ParamRequestRead.builder()
                .targetSystem(1).targetComponent(1)
                .paramId("WPNAV_SPEED")
                .paramIndex(-1)
                .build(),
            gcsSysId, gcsCompId,
        )
        assertTrue(sent.isEmpty())

        responder.onMessage(
            ParamRequestRead.builder()
                .targetSystem(1).targetComponent(1)
                .paramId("")
                .paramIndex(responder.parameters.count + 5)
                .build(),
            gcsSysId, gcsCompId,
        )
        assertTrue(sent.isEmpty())
    }

    @Test
    fun `param set on a read only parameter echoes the unchanged value`() {
        val target = responder.parameters.snapshot().first { !it.writable }

        responder.onMessage(paramSet(target.name, target.value + 7f, target.type), gcsSysId, gcsCompId)

        val echo = sent.filterIsInstance<ParamValue>().single()
        assertEquals(target.name, echo.paramId())
        assertEquals(target.wireValue(), echo.paramValue(), 0f)
        assertEquals(target.value, responder.parameters.byName(target.name)!!.value, 0f)
    }

    @Test
    fun `param set naming the wrong type is refused rather than decoded as garbage`() {
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

        // A GCS that thinks this is a float would have us reinterpret 4.0f's bits as 1082130432.
        subject.onMessage(
            paramSet("BRG_TEST_RW", 4f, MavParamType.MAV_PARAM_TYPE_REAL32),
            gcsSysId, gcsCompId,
        )

        assertEquals(1f, store.byName("BRG_TEST_RW")!!.value, 0f)
        val echo = out.filterIsInstance<ParamValue>().single()
        assertEquals(ParamCodec.encode(MavParamType.MAV_PARAM_TYPE_INT32, 1f), echo.paramValue(), 0f)
    }

    @Test
    fun `param set is applied only when a writer accepts it`() {
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

        val int32 = MavParamType.MAV_PARAM_TYPE_INT32

        // No writer registered yet: the write must not take effect.
        subject.onMessage(paramSet("BRG_TEST_RW", 4f, int32), gcsSysId, gcsCompId)
        assertEquals(ParamCodec.encode(int32, 1f), out.filterIsInstance<ParamValue>().single().paramValue(), 0f)
        assertEquals(1f, store.byName("BRG_TEST_RW")!!.value, 0f)

        out.clear()
        val seen = mutableListOf<Pair<String, Float>>()
        subject.registerParameterWriter { name, value -> seen.add(name to value); true }

        subject.onMessage(paramSet("BRG_TEST_RW", 4f, int32), gcsSysId, gcsCompId)
        // The writer sees the decoded numeric value, not the wire bit pattern.
        assertEquals(listOf("BRG_TEST_RW" to 4f), seen)
        assertEquals(ParamCodec.encode(int32, 4f), out.filterIsInstance<ParamValue>().single().paramValue(), 0f)
        assertEquals(4f, store.byName("BRG_TEST_RW")!!.value, 0f)
    }

    @Test
    fun `param set for an unknown parameter is ignored`() {
        responder.onMessage(
            paramSet("NOT_A_PARAM", 1f, MavParamType.MAV_PARAM_TYPE_INT32),
            gcsSysId, gcsCompId,
        )
        assertTrue(sent.isEmpty())
    }

    // --------------------------------------------------------- param encoding

    @Test
    fun `parameter values go out byte-wise encoded`() {
        // Bytewise is what QGC reads natively now that no APM plugin translates for us:
        // integer 1 must travel as the float whose bit pattern is 0x00000001.
        responder.onMessage(paramRequestList(), gcsSysId, gcsCompId)

        val values = sent.filterIsInstance<ParamValue>()
        val declared = responder.parameters.snapshot().associateBy { it.name }
        values.forEach { wire ->
            val parameter = declared[wire.paramId()]!!
            assertEquals(MavParamType.MAV_PARAM_TYPE_INT32, parameter.type)
            assertEquals(parameter.value.toInt(), wire.paramValue().toRawBits())
        }
    }

    @Test
    fun `a byte-wise value decodes the way QGroundControl decodes it`() {
        // Mirrors ParameterManager::_mavlinkParamUnionToVariant: copy param_value into the
        // union, then read the member for the declared type — a bit reinterpretation.
        fun qgcReadsInt32(wire: Float): Int = wire.toRawBits()

        responder.onMessage(paramRequestList(), gcsSysId, gcsCompId)

        val byName = sent.filterIsInstance<ParamValue>().associateBy { it.paramId() }
        assertEquals(1, qgcReadsInt32(byName["BRG_PROTO_VER"]!!.paramValue()))
        assertEquals(1, qgcReadsInt32(byName["BRG_MAV_SYSID"]!!.paramValue()))
        assertEquals(1, qgcReadsInt32(byName["BRG_TLM_HZ"]!!.paramValue()))
        assertEquals(0, qgcReadsInt32(byName["BRG_GUIDED_OK"]!!.paramValue()))
    }

    @Test
    fun `the codec round trips every carryable type`() {
        val cases = listOf(
            MavParamType.MAV_PARAM_TYPE_REAL32 to 1.5f,
            MavParamType.MAV_PARAM_TYPE_INT32 to -12345f,
            MavParamType.MAV_PARAM_TYPE_UINT32 to 3000000f,
            MavParamType.MAV_PARAM_TYPE_INT16 to -300f,
            MavParamType.MAV_PARAM_TYPE_UINT16 to 60000f,
            MavParamType.MAV_PARAM_TYPE_INT8 to -7f,
            MavParamType.MAV_PARAM_TYPE_UINT8 to 200f,
        )
        cases.forEach { (type, value) ->
            assertEquals("$type", value, ParamCodec.decode(type, ParamCodec.encode(type, value)), 0f)
        }
    }

    @Test(expected = IllegalArgumentException::class)
    fun `a 64 bit parameter type is rejected because it cannot fit param_value`() {
        ParameterStore(
            listOf(
                Parameter(
                    name = "BRG_TOO_WIDE",
                    type = MavParamType.MAV_PARAM_TYPE_INT64,
                    value = 1f,
                    doc = "cannot be carried by the parameter protocol",
                )
            )
        )
    }

    // ---------------------------------------------------------------- commands

    @Test
    fun `an unimplemented command is acknowledged as unsupported`() {
        responder.onMessage(commandLong(MavCmd.MAV_CMD_NAV_TAKEOFF, param7 = 10f), gcsSysId, gcsCompId)

        val ack = sent.filterIsInstance<CommandAck>().single()
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, ack.result().entry())
        assertEquals(
            EnumValue.of(MavCmd.MAV_CMD_NAV_TAKEOFF).value(),
            ack.command().value(),
        )
        assertEquals(gcsSysId, ack.targetSystem())
        assertEquals(gcsCompId, ack.targetComponent())
        // Nothing else: an unsupported command must not produce side effects.
        assertEquals(1, sent.size)
    }

    @Test
    fun `an unimplemented command int is acknowledged as unsupported`() {
        responder.onMessage(
            CommandInt.builder()
                .targetSystem(1).targetComponent(1)
                .command(MavCmd.MAV_CMD_DO_REPOSITION)
                .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                .x(377000000).y(238000000).z(30f)
                .build(),
            gcsSysId, gcsCompId,
        )

        val ack = sent.filterIsInstance<CommandAck>().single()
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, ack.result().entry())
        assertEquals(EnumValue.of(MavCmd.MAV_CMD_DO_REPOSITION).value(), ack.command().value())
    }

    @Test
    fun `a registered command handler decides the result and still gets exactly one ack`() {
        val seen = mutableListOf<HandshakeResponder.CommandRequest>()
        responder.registerCommandHandler(EnumValue.of(MavCmd.MAV_CMD_NAV_TAKEOFF).value()) { request ->
            seen.add(request)
            MavResult.MAV_RESULT_TEMPORARILY_REJECTED
        }

        responder.onMessage(commandLong(MavCmd.MAV_CMD_NAV_TAKEOFF, param7 = 12f), gcsSysId, gcsCompId)

        assertEquals(1, seen.size)
        assertEquals(12f, seen[0].param7, 0f)
        val ack = sent.filterIsInstance<CommandAck>().single()
        assertEquals(MavResult.MAV_RESULT_TEMPORARILY_REJECTED, ack.result().entry())
    }

    @Test
    fun `a handler that throws is reported as failed and does not escape`() {
        responder.registerCommandHandler(EnumValue.of(MavCmd.MAV_CMD_NAV_TAKEOFF).value()) {
            throw IllegalStateException("no aircraft")
        }

        responder.onMessage(commandLong(MavCmd.MAV_CMD_NAV_TAKEOFF), gcsSysId, gcsCompId)

        assertEquals(
            MavResult.MAV_RESULT_FAILED,
            sent.filterIsInstance<CommandAck>().single().result().entry(),
        )
    }

    @Test
    fun `a message provider that throws leaves the request unsupported`() {
        responder.registerMessageProvider(242) { throw IllegalStateException("no home position") }

        responder.onMessage(commandLong(MavCmd.MAV_CMD_REQUEST_MESSAGE, param1 = 242f), gcsSysId, gcsCompId)

        assertEquals(
            MavResult.MAV_RESULT_UNSUPPORTED,
            sent.filterIsInstance<CommandAck>().single().result().entry(),
        )
        assertEquals(1, sent.size)
    }

    // ------------------------------------------------- the PX4 button surface

    /**
     * Every command QGroundControl's PX4 Fly view can send us, with the numeric `MAV_CMD` id
     * spelled out. Ids are asserted against the dialect below, so a library that renumbers or
     * renames one of these fails loudly instead of silently changing which button we answer.
     *
     * These buttons did not exist while we identified as `MAV_AUTOPILOT_GENERIC`
     * (`FirmwarePlugin::isCapable` was false for everything). `PX4FirmwarePlugin::isCapable`
     * grants them from the vehicle type alone, so this is the surface the identity change opened.
     */
    private val px4ButtonCommands: List<Triple<Int, MavCmd, String>> = listOf(
        Triple(22, MavCmd.MAV_CMD_NAV_TAKEOFF, "Takeoff (param7 = AMSL alt)"),
        Triple(400, MavCmd.MAV_CMD_COMPONENT_ARM_DISARM, "Arm/Disarm, and Emergency Stop"),
        Triple(192, MavCmd.MAV_CMD_DO_REPOSITION, "Pause / Go To / Change Alt / Change Heading"),
        Triple(178, MavCmd.MAV_CMD_DO_CHANGE_SPEED, "Change Speed"),
        Triple(34, MavCmd.MAV_CMD_DO_ORBIT, "Orbit"),
        Triple(195, MavCmd.MAV_CMD_DO_SET_ROI_LOCATION, "ROI"),
        Triple(197, MavCmd.MAV_CMD_DO_SET_ROI_NONE, "ROI off"),
        Triple(179, MavCmd.MAV_CMD_DO_SET_HOME, "Set Home from the map"),
        Triple(191, MavCmd.MAV_CMD_DO_GO_AROUND, "Abort Landing"),
        Triple(2510, MavCmd.MAV_CMD_LOGGING_START, "PX4 log page Start"),
        Triple(2511, MavCmd.MAV_CMD_LOGGING_STOP, "PX4 log page Stop"),
        // Not sent by QGC's PX4 plugin (it uses SET_MODE), but sent by other ground stations
        // and by ArduPilot-flavoured clients. Must be refusable, not silently ignored.
        Triple(176, MavCmd.MAV_CMD_DO_SET_MODE, "mode change, command form"),
        Triple(21, MavCmd.MAV_CMD_NAV_LAND, "Land, command form"),
        Triple(20, MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH, "RTL, command form"),
        Triple(300, MavCmd.MAV_CMD_MISSION_START, "Start Mission, command form"),
    )

    @Test
    fun `the MAV_CMD ids we enumerate are the ids the dialect defines`() {
        px4ButtonCommands.forEach { (id, command, what) ->
            assertEquals("$what: $command", id, EnumValue.of(command).value())
        }
        assertEquals(176, HandshakeResponder.MAV_CMD_DO_SET_MODE)
    }

    @Test
    fun `every PX4 fly-view command is refused as unsupported, once, with no side effects`() {
        px4ButtonCommands.forEach { (id, command, what) ->
            sent.clear()
            responder.onMessage(
                CommandLong.builder()
                    .targetSystem(1).targetComponent(1)
                    .command(command)
                    .confirmation(0)
                    .param1(1f)
                    .build(),
                gcsSysId, gcsCompId,
            )

            // Exactly one message back, and it is the refusal. Never ACCEPTED: telling QGC a
            // command succeeded when nothing happened is the failure that can hurt someone.
            assertEquals("$what: one ack only", 1, sent.size)
            val ack = sent.filterIsInstance<CommandAck>().single()
            assertEquals("$what", MavResult.MAV_RESULT_UNSUPPORTED, ack.result().entry())
            assertEquals("$what", id, ack.command().value())
            assertEquals("$what", gcsSysId, ack.targetSystem())
            assertEquals("$what", gcsCompId, ack.targetComponent())
        }
    }

    @Test
    fun `the measured map-click commands are refused with their real wire arguments`() {
        // Captured from QGC 5.0.8 on 2026-07-25 against a vehicle reporting 103.2 m AMSL ground
        // and 30 m altitude. The altitudes are the point: every one is AMSL, so an M2/M3
        // implementation must subtract our own AMSL rather than treat z as height above ground.
        val goTo = CommandInt.builder()
            .targetSystem(1).targetComponent(1)
            .command(MavCmd.MAV_CMD_DO_REPOSITION)
            .frame(MavFrame.MAV_FRAME_GLOBAL)
            .param1(-1f).param2(1f).param3(0f).param4(Float.NaN)
            .x(379975294).y(237272504).z(133.2f)
            .build()
        responder.onMessage(goTo, gcsSysId, gcsCompId)
        assertEquals(1, sent.size)
        assertEquals(
            MavResult.MAV_RESULT_UNSUPPORTED,
            sent.filterIsInstance<CommandAck>().single().result().entry(),
        )

        // The decoded request keeps the frame and the raw 1e7 integers, so a future handler can
        // tell MAV_FRAME_GLOBAL (AMSL) from MAV_FRAME_GLOBAL_RELATIVE_ALT without guessing.
        val seen = mutableListOf<HandshakeResponder.CommandRequest>()
        responder.registerCommandHandler(EnumValue.of(MavCmd.MAV_CMD_DO_REPOSITION).value()) {
            seen.add(it)
            MavResult.MAV_RESULT_UNSUPPORTED
        }
        sent.clear()
        responder.onMessage(goTo, gcsSysId, gcsCompId)
        val request = seen.single()
        assertTrue(request.isCommandInt)
        assertEquals(EnumValue.of(MavFrame.MAV_FRAME_GLOBAL).value(), request.frame)
        assertEquals(379975294, request.x)
        assertEquals(237272504, request.y)
        assertEquals(133.2f, request.param7, 0.001f) // COMMAND_INT.z is param7 by definition
        assertEquals(1f, request.param2, 0f)         // MAV_DO_REPOSITION_FLAGS_CHANGE_MODE
    }

    @Test
    fun `set home arrives as COMMAND_LONG with float degrees, not COMMAND_INT`() {
        // Measured: QGC's terrain-backed Set Home uses sendMavCommand, so the coordinate is
        // degraded to float in param5/param6 rather than carried as 1e7 integers. Worth pinning
        // because it is the one positional command that does NOT benefit from our COMMAND_INT
        // capability claim, and a handler must not assume x/y are populated.
        val seen = mutableListOf<HandshakeResponder.CommandRequest>()
        responder.registerCommandHandler(EnumValue.of(MavCmd.MAV_CMD_DO_SET_HOME).value()) {
            seen.add(it)
            MavResult.MAV_RESULT_UNSUPPORTED
        }

        responder.onMessage(
            CommandLong.builder()
                .targetSystem(1).targetComponent(1)
                .command(MavCmd.MAV_CMD_DO_SET_HOME)
                .param1(0f).param2(0f).param3(0f).param4(Float.NaN)
                .param5(37.994545f).param6(23.728001f).param7(78.0f)
                .build(),
            gcsSysId, gcsCompId,
        )

        val request = seen.single()
        assertFalse(request.isCommandInt)
        assertEquals(0, request.x)
        assertEquals(0, request.y)
        assertEquals(37.994545f, request.param5, 1e-5f)
        assertEquals(23.728001f, request.param6, 1e-5f)
        assertEquals(78.0f, request.param7, 0f)
        assertEquals(
            MavResult.MAV_RESULT_UNSUPPORTED,
            sent.filterIsInstance<CommandAck>().single().result().entry(),
        )
    }

    @Test
    fun `the same refusal applies in COMMAND_INT form, which PX4 uses for positional commands`() {
        // guidedModeGotoLocation / guidedModeOrbit / stopGuidedModeROI / sendROICommand all send
        // COMMAND_INT once MAV_PROTOCOL_CAPABILITY_COMMAND_INT is claimed, which it is.
        listOf(
            MavCmd.MAV_CMD_DO_REPOSITION,
            MavCmd.MAV_CMD_DO_ORBIT,
            MavCmd.MAV_CMD_DO_SET_ROI_LOCATION,
            MavCmd.MAV_CMD_DO_SET_ROI_NONE,
            MavCmd.MAV_CMD_DO_SET_HOME,
        ).forEach { command ->
            sent.clear()
            responder.onMessage(
                CommandInt.builder()
                    .targetSystem(1).targetComponent(1)
                    .command(command)
                    .frame(MavFrame.MAV_FRAME_GLOBAL)
                    .x(379938600).y(237253300).z(133.2f)
                    .build(),
                gcsSysId, gcsCompId,
            )

            assertEquals("$command", 1, sent.size)
            val ack = sent.filterIsInstance<CommandAck>().single()
            assertEquals("$command", MavResult.MAV_RESULT_UNSUPPORTED, ack.result().entry())
            assertEquals("$command", EnumValue.of(command).value(), ack.command().value())
        }
    }

    @Test
    fun `a takeoff refusal is what stops QGC from following up with an arm`() {
        // PX4FirmwarePlugin::_mavCommandResult arms the vehicle only on
        // MAV_CMD_NAV_TAKEOFF + MAV_RESULT_ACCEPTED. Answering ACCEPTED to a takeoff we cannot
        // perform would make QGC send MAV_CMD_COMPONENT_ARM_DISARM next, unprompted.
        responder.onMessage(
            CommandLong.builder()
                .targetSystem(1).targetComponent(1)
                .command(MavCmd.MAV_CMD_NAV_TAKEOFF)
                .param1(Float.NaN).param2(Float.NaN).param3(0f)
                .param4(Float.NaN).param5(Float.NaN).param6(Float.NaN)
                .param7(133.2f) // AMSL, not relative: PX4FirmwarePlugin.cc:317-327
                .build(),
            gcsSysId, gcsCompId,
        )

        val ack = sent.filterIsInstance<CommandAck>().single()
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, ack.result().entry())
        assertFalse(
            "MAV_RESULT_ACCEPTED for an unperformed takeoff triggers QGC's auto-arm",
            ack.result().entry() == MavResult.MAV_RESULT_ACCEPTED,
        )
    }

    @Test
    fun `an emergency stop is refused rather than acknowledged`() {
        // Vehicle::emergencyStop: ARM_DISARM with the 21196 force-disarm magic. An ACCEPTED here
        // would tell an operator the motors were cut.
        responder.onMessage(
            CommandLong.builder()
                .targetSystem(1).targetComponent(1)
                .command(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM)
                .param1(0f).param2(21196f)
                .build(),
            gcsSysId, gcsCompId,
        )

        val ack = sent.filterIsInstance<CommandAck>().single()
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, ack.result().entry())
        assertEquals(1, sent.size)
    }

    @Test
    fun `no refusal is ever reported as in progress`() {
        // MAV_RESULT_IN_PROGRESS with no terminal ack leaves QGC's queue entry pending forever
        // and blocks that command from ever being re-sent (MavCommandQueue.cc:497-506).
        px4ButtonCommands.forEach { (_, command, what) ->
            sent.clear()
            responder.onMessage(
                CommandLong.builder()
                    .targetSystem(1).targetComponent(1)
                    .command(command)
                    .build(),
                gcsSysId, gcsCompId,
            )
            assertFalse(
                "$what",
                sent.filterIsInstance<CommandAck>().single().result().entry() ==
                    MavResult.MAV_RESULT_IN_PROGRESS,
            )
        }
    }

    // ----------------------------------------------------------- SET_MODE (#11)

    @Test
    fun `a PX4 mode change is recorded and deliberately not answered`() {
        // QGC's PX4 route for RTL / Land / Hold / Mission / the mode dropdown. Not a command:
        // MAV_CMD_DO_SET_MODE_is_supported() is false for PX4, so Vehicle::setFlightMode sends
        // the SET_MODE message, which has no acknowledgement.
        responder.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)

        // No acknowledgement: there is no negative ack for SET_MODE, and a COMMAND_ACK for a
        // message QGC never queued is discarded anyway ("Ack not in list").
        assertTrue("SET_MODE must never be acked", sent.none { it is CommandAck })
        // But not silence either. The mode dropdown never re-reads and reports nothing, so
        // without this the control just springs back and the operator is told nothing at all.
        val warning = sent.filterIsInstance<Statustext>().single()
        assertEquals(HandshakeResponder.MODE_REFUSAL_TEXT, warning.text())
        // ERROR because QGroundControl only surfaces EMERGENCY/ALERT/CRITICAL/ERROR
        // (`StatusTextHandler.cc:18-24`, `:241`). Measured: at WARNING, pressing Return on a
        // real QGC 5.0.8 showed the operator nothing at all — the refusal was filed in a list
        // nobody opens, which made this whole path a no-op. Do not lower it back.
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, warning.severity().entry())
        // 50 bytes is the STATUSTEXT field width; longer text is truncated on the wire.
        assertTrue(HandshakeResponder.MODE_REFUSAL_TEXT.toByteArray().size <= 50)

        val recorded = responder.requestedModes.single()
        assertEquals(PX4_AUTO_RTL, recorded.customMode)
        assertEquals(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, recorded.baseMode)
        assertEquals(gcsSysId, recorded.senderSystemId)
        assertEquals(gcsCompId, recorded.senderComponentId)
    }

    @Test
    fun `each PX4 guided mode QGC can ask for is recorded verbatim`() {
        // Hand-computed from px4_custom_mode.h: main mode in bits 16-23, sub mode in bits 24-31.
        val modes = listOf(
            "Return" to PX4_AUTO_RTL,        // main AUTO(4), sub RTL(5)
            "Land" to 0x06_04_0000L,         // main AUTO(4), sub LAND(6)
            "Hold/pause" to 0x03_04_0000L,   // main AUTO(4), sub LOITER(3)
            "Mission" to 0x04_04_0000L,      // main AUTO(4), sub MISSION(4)
            "Takeoff" to 0x02_04_0000L,      // main AUTO(4), sub TAKEOFF(2)
            "Manual" to 0x00_01_0000L,       // main MANUAL(1), no sub mode
        )

        modes.forEach { (_, customMode) ->
            responder.onMessage(setMode(customMode), gcsSysId, gcsCompId)
        }

        // Never acked, and each distinct mode is announced to the operator once.
        assertTrue(sent.none { it is CommandAck })
        assertEquals(modes.size, sent.filterIsInstance<Statustext>().size)
        assertEquals(modes.map { it.second }, responder.requestedModes.map { it.customMode })
        // QGC retries three times per press; every attempt is kept, none coalesced.
        responder.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)
        responder.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)
        assertEquals(modes.size + 2, responder.requestedModes.size)
        // ...but the operator is not warned three times for one click. The last announced
        // mode was Manual, so the first RTL here is a new intent and is announced; the
        // identical one behind it is the retry and is suppressed.
        assertEquals(modes.size + 1, sent.filterIsInstance<Statustext>().size)
    }

    @Test
    fun `a repeated mode request is announced again once the retry window has passed`() {
        // The window exists to collapse QGC's 3-shot retry burst (~1.34 s apart) into one
        // warning. It must not swallow a second, deliberate press by the operator — someone
        // who clicks Return again after nothing happened is owed the same answer again.
        var clock = 10_000L
        val fresh = mutableListOf<Any>()
        val r = HandshakeResponder(send = { fresh.add(it) }, nowMs = { clock })

        r.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)
        assertEquals(1, fresh.filterIsInstance<Statustext>().size)

        // The rest of the retry burst.
        clock += 1_340
        r.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)
        clock += 1_340
        r.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)
        assertEquals("one click must not warn three times", 1, fresh.filterIsInstance<Statustext>().size)

        // A fresh press, well after the burst.
        clock += HandshakeResponder.MODE_REFUSAL_REPEAT_MS
        r.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)
        assertEquals(2, fresh.filterIsInstance<Statustext>().size)

        // Every attempt is still recorded, regardless of what was announced.
        assertEquals(4, r.requestedModes.size)
    }

    @Test
    fun `a different mode inside the window is a new intent and is announced`() {
        var clock = 10_000L
        val fresh = mutableListOf<Any>()
        val r = HandshakeResponder(send = { fresh.add(it) }, nowMs = { clock })

        r.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)
        clock += 100
        r.onMessage(setMode(0x06_04_0000L), gcsSysId, gcsCompId) // Land
        assertEquals(2, fresh.filterIsInstance<Statustext>().size)
    }

    @Test
    fun `a mode change for another vehicle is ignored`() {
        // SET_MODE has no target_component, so only the system id can be checked.
        responder.onMessage(
            SetMode.builder()
                .targetSystem(2)
                .baseMode(EnumValue.create<MavMode>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))
                .customMode(PX4_AUTO_RTL)
                .build(),
            gcsSysId, gcsCompId,
        )
        assertTrue(sent.isEmpty())
        assertTrue(responder.requestedModes.isEmpty())
    }

    @Test
    fun `a mode handler that takes the request replaces the refusal, and only the refusal`() {
        // M2's command layer registers here. Taking a request means "acted on, and the operator
        // has been told whatever there is to tell", so the generic warning would be both
        // redundant and less true than DJI's own word for what happened.
        val seen = mutableListOf<HandshakeResponder.ModeRequest>()
        responder.registerModeHandler { seen.add(it); true }

        responder.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)

        assertTrue("still never acked", sent.none { it is CommandAck })
        assertTrue("the handler owns the operator's news now", sent.isEmpty())
        assertEquals(PX4_AUTO_RTL, seen.single().customMode)
        // Recorded regardless: requestedModes is what QGC asked for, which is true whoever
        // handles it.
        assertEquals(PX4_AUTO_RTL, responder.requestedModes.single().customMode)
    }

    @Test
    fun `a mode handler that declines leaves the pre-M2 refusal exactly as it was`() {
        // This is the safe state of the interlock, and the reason it is expressed as "decline"
        // rather than as a second refusal branch: with commands off, the bridge is byte for byte
        // the one that existed before the command layer did.
        responder.registerModeHandler { false }

        responder.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)

        val warning = sent.filterIsInstance<Statustext>().single()
        assertEquals(HandshakeResponder.MODE_REFUSAL_TEXT, warning.text())
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, warning.severity().entry())
        assertTrue(sent.none { it is CommandAck })
        assertEquals(1, responder.requestedModes.size)
    }

    @Test
    fun `a mode handler that throws is treated as declining, and does not escape`() {
        // A half-written command layer must not kill the receive thread, and must not silently
        // swallow the operator's only feedback either.
        responder.registerModeHandler { throw IllegalStateException("half-written") }

        responder.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)

        assertEquals(
            HandshakeResponder.MODE_REFUSAL_TEXT,
            sent.filterIsInstance<Statustext>().single().text(),
        )
        assertEquals(1, responder.requestedModes.size)
    }

    @Test
    fun `a taken request does not consume the refusal window of a later declined one`() {
        // The two are independent: taking requests must not leave the de-duplication state of
        // announceModeRefusal in a position where a genuinely refused mode goes unannounced.
        var take = true
        var clock = 10_000L
        val fresh = mutableListOf<Any>()
        val r = HandshakeResponder(send = { fresh.add(it) }, nowMs = { clock })
        r.registerModeHandler { take }

        r.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)
        assertTrue(fresh.isEmpty())

        take = false
        clock += 100
        r.onMessage(setMode(PX4_AUTO_RTL), gcsSysId, gcsCompId)
        assertEquals(1, fresh.filterIsInstance<Statustext>().size)
    }

    @Test
    fun `the command form of a mode change is refusable, unlike the message form`() {
        // MAV_CMD_DO_SET_MODE (176) is what ArduPilot-flavoured clients send. It is a command,
        // so it does get a COMMAND_ACK — and that ack must be UNSUPPORTED.
        responder.onMessage(
            CommandLong.builder()
                .targetSystem(1).targetComponent(1)
                .command(MavCmd.MAV_CMD_DO_SET_MODE)
                .param1(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED.toFloat())
                .param2(PX4_AUTO_RTL.toFloat())
                .build(),
            gcsSysId, gcsCompId,
        )

        val ack = sent.filterIsInstance<CommandAck>().single()
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, ack.result().entry())
        assertEquals(HandshakeResponder.MAV_CMD_DO_SET_MODE, ack.command().value())
        // The command form is not recorded as a mode request: requestedModes is specifically the
        // unackable channel, and conflating them would hide which one QGC actually used.
        assertTrue(responder.requestedModes.isEmpty())
    }

    @Test
    fun `a mode change decoded off the wire is recorded the same way`() {
        responder.onMessage(roundTrip(setMode(PX4_AUTO_RTL)))

        assertTrue(sent.none { it is CommandAck })
        assertEquals(1, sent.filterIsInstance<Statustext>().size)
        val recorded = responder.requestedModes.single()
        assertEquals(PX4_AUTO_RTL, recorded.customMode)
        assertEquals(gcsSysId, recorded.senderSystemId)
        assertEquals(gcsCompId, recorded.senderComponentId)
    }

    @Test
    fun `a command aimed at another vehicle is neither handled nor acknowledged`() {
        responder.onMessage(
            CommandLong.builder()
                .targetSystem(2).targetComponent(1)
                .command(MavCmd.MAV_CMD_NAV_TAKEOFF)
                .build(),
            gcsSysId, gcsCompId,
        )
        assertTrue(sent.isEmpty())
    }

    // ------------------------------------------------------- autopilot version

    @Test
    fun `autopilot version is served through MAV_CMD_REQUEST_MESSAGE`() {
        responder.onMessage(
            commandLong(
                MavCmd.MAV_CMD_REQUEST_MESSAGE,
                param1 = AutopilotIdentity.MESSAGE_ID_AUTOPILOT_VERSION.toFloat(),
            ),
            gcsSysId, gcsCompId,
        )

        val ack = sent.filterIsInstance<CommandAck>().single()
        assertEquals(MavResult.MAV_RESULT_ACCEPTED, ack.result().entry())
        assertEquals(EnumValue.of(MavCmd.MAV_CMD_REQUEST_MESSAGE).value(), ack.command().value())
        assertNotNull(sent.filterIsInstance<AutopilotVersion>().single())
    }

    @Test
    fun `autopilot version is served through the legacy capabilities command`() {
        responder.onMessage(
            commandLong(MavCmd.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, param1 = 1f),
            gcsSysId, gcsCompId,
        )

        val ack = sent.filterIsInstance<CommandAck>().single()
        assertEquals(MavResult.MAV_RESULT_ACCEPTED, ack.result().entry())
        assertEquals(
            EnumValue.of(MavCmd.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES).value(),
            ack.command().value(),
        )
        assertNotNull(sent.filterIsInstance<AutopilotVersion>().single())
    }

    @Test
    fun `a request for a message we cannot produce is unsupported, not a silent accept`() {
        responder.onMessage(
            commandLong(MavCmd.MAV_CMD_REQUEST_MESSAGE, param1 = 259f), // CAMERA_INFORMATION
            gcsSysId, gcsCompId,
        )

        val ack = sent.filterIsInstance<CommandAck>().single()
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, ack.result().entry())
        assertEquals(1, sent.size)
    }

    @Test
    fun `a registered message provider makes that request supported`() {
        val extendedSysState = ExtendedSysState.builder()
            .vtolState(MavVtolState.MAV_VTOL_STATE_UNDEFINED)
            .landedState(MavLandedState.MAV_LANDED_STATE_ON_GROUND)
            .build()
        responder.registerMessageProvider(245) { extendedSysState }

        responder.onMessage(commandLong(MavCmd.MAV_CMD_REQUEST_MESSAGE, param1 = 245f), gcsSysId, gcsCompId)

        assertEquals(MavResult.MAV_RESULT_ACCEPTED, sent.filterIsInstance<CommandAck>().single().result().entry())
        assertEquals(extendedSysState, sent.filterIsInstance<ExtendedSysState>().single())
    }

    @Test
    fun `declared capabilities exclude what we cannot honour`() {
        val bits = AutopilotIdentity().capabilityBits
        fun has(capability: MavProtocolCapability) =
            bits and EnumValue.of(capability).value() != 0

        assertTrue(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MAVLINK2))
        assertTrue(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_COMMAND_INT))
        // Bytewise. PX4 reintroduces no parameter translation: its plugin's
        // adjustIncomingMavlinkMessage handles AUTOPILOT_VERSION only, and it does not override
        // adjustOutgoingMavlinkMessageThreadSafe at all — APM is the sole override in the tree.
        assertTrue(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE))
        assertFalse(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST))
        // Deprecated in favour of the two encoding flags, and says nothing they do not.
        assertFalse(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT))

        // M4 turned MISSION_INT on: we hold a real mission store, take MISSION_ITEM_INT uploads
        // and serve them back verbatim (`mission/MissionTransaction`). MISSION_FLOAT stays off —
        // the float form is refused rather than converted, because a float latitude is ~1 m of
        // precision lost.
        assertTrue(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MISSION_INT))

        // We implement no FTP, no fence, no rally, no guided setpoints yet.
        assertFalse(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_FTP))
        assertFalse(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT))
        assertFalse(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE))
        assertFalse(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY))
        assertFalse(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_TERRAIN))
        assertFalse(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED))
        assertFalse(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT))
        assertFalse(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET))
        assertFalse(has(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION))
    }

    @Test
    fun `answering at all is what stops QGC inventing fence and rally support for a PX4 vehicle`() {
        // InitialConnectStateMachine::_handleAutopilotVersionFailure assumes
        // MAVLINK2 | MISSION_INT | COMMAND_INT | MISSION_FENCE | MISSION_RALLY for px4Firmware()
        // or apmFirmware(), and MAVLINK2 alone otherwise. Under GENERIC, silence was merely
        // uninformative; under PX4 it would make GeoFenceManager::supported() and
        // RallyPointManager::supported() true for a vehicle that has neither.
        //
        // MISSION_INT is deliberately *not* in this list any more: M4 turned it on, so it is a
        // bit we now genuinely claim rather than one QGC would have guessed. The two that still
        // matter are fence and rally, whose absence is what makes QGC skip those connect states.
        val bits = AutopilotIdentity().capabilityBits
        val inventedOnSilence = listOf(
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE,
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY,
        )
        inventedOnSilence.forEach { capability ->
            assertEquals(
                "$capability must be absent, so answering removes QGC's PX4 guess",
                0,
                bits and EnumValue.of(capability).value(),
            )
        }
    }

    @Test
    fun `autopilot version claims no flight stack version and no git hash`() {
        val version = AutopilotIdentity().autopilotVersion()
        // 0 keeps QGC from calling setFirmwareVersion(); we are not a flight stack. It is also
        // what makes PX4FirmwarePlugin show its "upgrade your firmware" modal — a deliberate
        // trade, see AutopilotIdentity.
        assertEquals(0L, version.flightSwVersion())
        assertEquals(0, version.vendorId())
        assertEquals(0, version.productId())

        // flight_custom_version must be all zero, not ASCII. QGC reads it as reversed binary for
        // PX4 (git hash "0000000000000000", custom version 0.0.0). An ASCII "mini4pro" would be
        // rendered as the hash 6f7270346e696d00 and the version 110.105.109 — accidental digits
        // that look like a real firmware revision.
        val custom = version.flightCustomVersion()
        assertEquals(8, custom.size)
        assertTrue(
            "an ASCII build tag is misread as a git hash under PX4",
            custom.all { it.toInt() == 0 },
        )
    }

    // ---------------------------------------------------------------- missions

    /**
     * The three tests that used to live here — the empty-list download, the refused upload and
     * the refused clear-all — **moved to `MissionTransactionTest` with the code they pinned**
     * (JC-10). They were assertions about `HandshakeResponder` answering mission messages, and it
     * no longer does; re-pointing them at `MissionTransaction` is what keeps the behaviour
     * covered rather than merely deleted.
     *
     * What stays here is the *absence*: this class must answer nothing at all to a mission
     * message, because it runs **first** in `Bridge.onInbound` and a stale answer would win.
     */
    @Test
    fun `no mission message is answered here any more, because a second answer would win`() {
        val missionMessages = listOf(
            MissionRequestList.builder()
                .targetSystem(1).targetComponent(1)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION).build(),
            MissionCount.builder()
                .targetSystem(1).targetComponent(1).count(4)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION).build(),
            MissionClearAll.builder()
                .targetSystem(1).targetComponent(1)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION).build(),
            MissionRequestInt.builder()
                .targetSystem(1).targetComponent(1).seq(0)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION).build(),
            MissionRequest.builder()
                .targetSystem(1).targetComponent(1).seq(0)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION).build(),
        )

        missionMessages.forEach { message ->
            sent.clear()
            responder.onMessage(message, gcsSysId, gcsCompId)
            assertEquals(
                "${message.javaClass.simpleName} must fall through to MissionTransaction",
                emptyList<Any>(),
                sent.toList(),
            )
        }
    }

    // -------------------------------------------------------------- rate asks

    @Test
    fun `set message interval is recorded and reported unsupported without a sink`() {
        responder.onMessage(
            commandLong(MavCmd.MAV_CMD_SET_MESSAGE_INTERVAL, param1 = 242f, param2 = 1000000f),
            gcsSysId, gcsCompId,
        )

        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, sent.filterIsInstance<CommandAck>().single().result().entry())
        assertEquals(1000000L, responder.requestedIntervals[242] ?: -1L)
    }

    @Test
    fun `set message interval is accepted when a sink honours it`() {
        val honoured = mutableListOf<Pair<Int, Long>>()
        responder.registerIntervalSink { messageId, intervalUs ->
            honoured.add(messageId to intervalUs)
            messageId == 245
        }

        responder.onMessage(
            commandLong(MavCmd.MAV_CMD_SET_MESSAGE_INTERVAL, param1 = 245f, param2 = 500000f),
            gcsSysId, gcsCompId,
        )
        assertEquals(MavResult.MAV_RESULT_ACCEPTED, sent.filterIsInstance<CommandAck>().single().result().entry())

        sent.clear()
        responder.onMessage(
            commandLong(MavCmd.MAV_CMD_SET_MESSAGE_INTERVAL, param1 = 242f, param2 = 500000f),
            gcsSysId, gcsCompId,
        )
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, sent.filterIsInstance<CommandAck>().single().result().entry())
        assertEquals(listOf(245 to 500000L, 242 to 500000L), honoured)
    }

    @Test
    fun `request data stream is recorded and not answered`() {
        responder.onMessage(
            RequestDataStream.builder()
                .targetSystem(1).targetComponent(1)
                .reqStreamId(6) // MAV_DATA_STREAM_POSITION
                .reqMessageRate(4)
                .startStop(1)
                .build(),
            gcsSysId, gcsCompId,
        )

        // No reply exists for REQUEST_DATA_STREAM; QGC judges it by whether data arrives.
        assertTrue(sent.isEmpty())
        val recorded = responder.requestedStreams[6]!!
        assertEquals(4, recorded.rateHz)
        assertTrue(recorded.start)
    }

    // -------------------------------------------------------------------- ftp

    @Test
    fun `an ftp request is answered with a file not found nak`() {
        // Defensive: QGC only tries FTP for parameters on APM firmware (ParameterManager's
        // _tryftp is apmFirmware()), so PX4 keeps this off the connect path. But any client may
        // ask, and QGC itself would use FTP for onboard logs if we ever claimed the FTP
        // capability bit. Shape matches QGC's FTPManager opening
        // @PARAM/param.pck with kCmdOpenFileRO and sequence number 1.
        val request = ByteArray(MavlinkFtp.PAYLOAD_LENGTH)
        request[0] = 1                     // seqNumber lo
        request[3] = 4                     // opcode: kCmdOpenFileRO
        request[4] = 22                    // size: path length

        responder.onMessage(
            FileTransferProtocol.builder()
                .targetNetwork(0)
                .targetSystem(1)
                .targetComponent(1)
                .payload(request)
                .build(),
            gcsSysId, gcsCompId,
        )

        val nak = sent.filterIsInstance<FileTransferProtocol>().single()
        val payload = nak.payload()
        assertEquals(gcsSysId, nak.targetSystem())
        assertEquals(MavlinkFtp.PAYLOAD_LENGTH, payload.size)
        assertEquals(2, payload[0].toInt())                              // seqNumber = request + 1
        assertEquals(0, payload[1].toInt())
        assertEquals(MavlinkFtp.RSP_NAK, payload[3].toInt() and 0xFF)
        assertEquals(1, payload[4].toInt())                              // one byte of error code
        assertEquals(4, payload[5].toInt())                              // req_opcode echoed
        assertEquals(MavlinkFtp.ERR_FILE_NOT_FOUND, payload[12].toInt() and 0xFF)
    }

    @Test
    fun `session scoped ftp requests are naked as invalid session`() {
        val request = ByteArray(MavlinkFtp.PAYLOAD_LENGTH)
        request[0] = 7
        request[3] = MavlinkFtp.CMD_BURST_READ_FILE.toByte()

        responder.onMessage(
            FileTransferProtocol.builder()
                .targetNetwork(0).targetSystem(1).targetComponent(1)
                .payload(request)
                .build(),
            gcsSysId, gcsCompId,
        )

        val payload = sent.filterIsInstance<FileTransferProtocol>().single().payload()
        assertEquals(8, payload[0].toInt())
        assertEquals(MavlinkFtp.ERR_INVALID_SESSION, payload[12].toInt() and 0xFF)
    }

    @Test
    fun `our own ftp nak is not answered again`() {
        val ourNak = ByteArray(MavlinkFtp.PAYLOAD_LENGTH)
        ourNak[3] = MavlinkFtp.RSP_NAK.toByte()

        responder.onMessage(
            FileTransferProtocol.builder()
                .targetNetwork(0).targetSystem(1).targetComponent(1)
                .payload(ourNak)
                .build(),
            gcsSysId, gcsCompId,
        )
        assertTrue(sent.isEmpty())
    }

    // --------------------------------------------------------- the real path

    @Test
    fun `a message decoded off the wire is handled the same way`() {
        // Proves the MavlinkMessage entry point the Bridge will use, including that QGC's
        // origin ids come back in the ack.
        val message = roundTrip(
            commandLong(
                MavCmd.MAV_CMD_REQUEST_MESSAGE,
                param1 = AutopilotIdentity.MESSAGE_ID_AUTOPILOT_VERSION.toFloat(),
            )
        )

        responder.onMessage(message)

        val ack = sent.filterIsInstance<CommandAck>().single()
        assertEquals(MavResult.MAV_RESULT_ACCEPTED, ack.result().entry())
        assertEquals(gcsSysId, ack.targetSystem())
        assertEquals(gcsCompId, ack.targetComponent())
        assertNotNull(sent.filterIsInstance<AutopilotVersion>().single())
    }

    @Test
    fun `param values survive a round trip through the wire format`() {
        responder.onMessage(paramRequestList(), gcsSysId, gcsCompId)
        val original = sent.filterIsInstance<ParamValue>()

        val decoded = original.map { roundTrip(it, sysId = 1, compId = 1).payload as ParamValue }

        assertEquals(original.map { it.paramId() }, decoded.map { it.paramId() })
        assertEquals(original.map { it.paramIndex() }, decoded.map { it.paramIndex() })
        assertEquals(original.map { it.paramCount() }, decoded.map { it.paramCount() })
        assertEquals(original.map { it.paramValue() }, decoded.map { it.paramValue() })
    }

    // --------------------------------------------------------------- fixtures

    /** `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`; QGC forces this bit on in `SET_MODE.base_mode`. */
    private val MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

    /** PX4 `AUTO_RTL`: main mode AUTO (4) in bits 16-23, sub mode RTL (5) in bits 24-31. */
    private val PX4_AUTO_RTL = 0x05_04_0000L

    private fun setMode(customMode: Long) =
        SetMode.builder()
            .targetSystem(1) // SET_MODE has no target_component field
            .baseMode(EnumValue.create<MavMode>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))
            .customMode(customMode)
            .build()

    private fun paramRequestList() =
        ParamRequestList.builder()
            .targetSystem(1)
            .targetComponent(0) // MAV_COMP_ID_ALL, which is what QGC uses
            .build()

    /** [value] is the numeric value; it is encoded the way a GCS would encode it. */
    private fun paramSet(name: String, value: Float, type: MavParamType) =
        ParamSet.builder()
            .targetSystem(1).targetComponent(1)
            .paramId(name)
            .paramValue(ParamCodec.encode(type, value))
            .paramType(type)
            .build()

    private fun commandLong(
        command: MavCmd,
        param1: Float = 0f,
        param2: Float = 0f,
        param7: Float = 0f,
    ) = CommandLong.builder()
        .targetSystem(1)
        .targetComponent(1)
        .command(command)
        .confirmation(0)
        .param1(param1)
        .param2(param2)
        .param7(param7)
        .build()

    /** Encodes [payload] as MAVLink 2 and decodes it again, as the UDP link would. */
    private fun roundTrip(
        payload: Any,
        sysId: Int = 255,
        compId: Int = 190,
    ): MavlinkMessage<*> {
        val buffer = ByteArrayOutputStream()
        MavlinkConnection.create(ByteArrayInputStream(ByteArray(0)), buffer)
            .send2(sysId, compId, payload)
        return MavlinkConnection
            .create(ByteArrayInputStream(buffer.toByteArray()), ByteArrayOutputStream())
            .next()
    }
}
