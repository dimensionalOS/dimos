package com.dimensional.mini4pro.record

import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.GlobalPositionInt
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.SetPositionTargetLocalNed
import io.dronefleet.mavlink.common.VfrHud
import io.dronefleet.mavlink.minimal.Heartbeat
import io.dronefleet.mavlink.minimal.MavAutopilot
import io.dronefleet.mavlink.minimal.MavState
import io.dronefleet.mavlink.minimal.MavType
import io.dronefleet.mavlink.protocol.MavlinkPacket
import io.dronefleet.mavlink.util.EnumValue
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The MAVLink tap.
 *
 * The load-bearing property is that a `mav_out` entry's hex is a **real MAVLink 2
 * frame**: it is the copy of the evidence that survives a bug in our own field
 * extraction, so if it does not parse, the redundancy is imaginary. That is checked
 * by parsing it back with `MavlinkPacket.fromV2Bytes` and validating the CRC against
 * the message's own `crcExtra` — which is a weaker check than pymavlink's (same
 * library both ways) but does catch a framing or length mistake.
 */
class MavlinkTapTest {

    private fun heartbeat() = Heartbeat.builder()
        .type(EnumValue.of(MavType.MAV_TYPE_QUADROTOR))
        .autopilot(EnumValue.of(MavAutopilot.MAV_AUTOPILOT_ARDUPILOTMEGA))
        .customMode(4L)
        .systemStatus(EnumValue.of(MavState.MAV_STATE_STANDBY))
        .mavlinkVersion(3)
        .build()

    @Test
    fun `message names match what pymavlink and QGC call them`() {
        assertEquals("HEARTBEAT", MavlinkTap.nameOf(Heartbeat::class.java))
        assertEquals("GLOBAL_POSITION_INT", MavlinkTap.nameOf(GlobalPositionInt::class.java))
        assertEquals("VFR_HUD", MavlinkTap.nameOf(VfrHud::class.java))
        assertEquals("COMMAND_LONG", MavlinkTap.nameOf(CommandLong::class.java))
        assertEquals(
            "SET_POSITION_TARGET_LOCAL_NED",
            MavlinkTap.nameOf(SetPositionTargetLocalNed::class.java),
        )
    }

    @Test
    fun `field names are snake_case like pymavlink's`() {
        assertEquals("time_boot_ms", MavlinkTap.snake("timeBootMs"))
        assertEquals("lat", MavlinkTap.snake("lat"))
        assertEquals("relative_alt", MavlinkTap.snake("relativeAlt"))
        assertEquals("satellites_visible", MavlinkTap.snake("satellitesVisible"))
    }

    @Test
    fun `decoded fields are the raw wire values, not prettified`() {
        // Ground-probe values, already converted by TelemetryEncoder: degE7 and mm.
        // Logging them in wire units is the whole point — a prettified log cannot be
        // used to check the conversion that produced it.
        val msg = GlobalPositionInt.builder()
            .timeBootMs(12345L)
            .lat(379938612)
            .lon(237253298)
            .alt(103169)
            .relativeAlt(0)
            .vx(0).vy(0).vz(0)
            .hdg(23890)
            .build()
        val json = MavlinkTap.fieldsJson(msg)!!
        assertTrue(json, json.contains("\"lat\":379938612"))
        assertTrue(json, json.contains("\"lon\":237253298"))
        assertTrue(json, json.contains("\"alt\":103169"))
        assertTrue(json, json.contains("\"hdg\":23890"))
        assertTrue(json, json.contains("\"time_boot_ms\":12345"))
    }

    @Test
    fun `enums are recorded as both the wire integer and the name`() {
        val json = MavlinkTap.fieldsJson(heartbeat())!!
        // 3 = MAV_AUTOPILOT_ARDUPILOTMEGA. The integer is what went on the wire; the
        // name is what a human reads. A bad enum mapping shows up as an integer with
        // no name beside it.
        assertTrue(json, json.contains("\"autopilot\":3"))
        assertTrue(json, json.contains("\"autopilot_s\":\"MAV_AUTOPILOT_ARDUPILOTMEGA\""))
        assertTrue(json, json.contains("\"type\":2"))
        assertTrue(json, json.contains("\"type_s\":\"MAV_TYPE_QUADROTOR\""))
        assertTrue(json, json.contains("\"custom_mode\":4"))
    }

    @Test
    fun `a command carries its parameters, so what the GCS asked for is on the record`() {
        val msg = CommandLong.builder()
            .targetSystem(1).targetComponent(1)
            .command(EnumValue.of(MavCmd.MAV_CMD_NAV_TAKEOFF))
            .param7(10.0f)
            .build()
        val json = MavlinkTap.fieldsJson(msg)!!
        assertTrue(json, json.contains("\"command\":22"))
        assertTrue(json, json.contains("\"command_s\":\"MAV_CMD_NAV_TAKEOFF\""))
        assertTrue(json, json.contains("\"param7\":10"))
    }

    @Test
    fun `a reserialized outbound frame is a real MAVLink 2 frame with a valid CRC`() {
        val hex = MavlinkTap.reserialize(heartbeat(), systemId = 1, componentId = 1, sequence = 42)
        assertNotNull(hex)
        val bytes = ByteArray(hex!!.length / 2) {
            hex.substring(it * 2, it * 2 + 2).toInt(16).toByte()
        }
        val packet = MavlinkPacket.fromV2Bytes(bytes)
        assertTrue("must be MAVLink 2", packet.isMavlink2)
        assertEquals(0, packet.incompatibleFlags)
        assertEquals(42, packet.sequence)
        assertEquals(1, packet.systemId)
        assertEquals(1, packet.componentId)
        assertEquals(0, packet.messageId)                 // HEARTBEAT
        assertTrue("CRC must validate against crcExtra 50", packet.validateCrc(50))
    }

    @Test
    fun `outbound entries are honest that their hex was reserialized`() {
        val entry = MavlinkTap.outbound(0, heartbeat(), 1, 1)
        assertEquals(LogEntry.KIND_MAV_OUT, entry.kind)
        assertEquals(0, entry.messageId)
        assertEquals("HEARTBEAT", entry.name)
        val line = JsonObject.render { o -> o.put("k", entry.kind); entry.writeBody(o) }
        // The label exists so nobody reasons about sequence continuity from a frame
        // whose sequence number we invented.
        assertTrue(line, line.contains("\"hexsrc\":\"reserialized\""))
        assertTrue(line, line.contains("\"hex\":\"fd"))
        assertTrue(line, line.contains("\"len\":21"))
    }

    @Test
    fun `wire bytes, when supplied, are used verbatim and labelled as such`() {
        val bytes = byteArrayOf(0xfd.toByte(), 0x09, 0x00, 0x00, 0x2a)
        val entry = MavlinkTap.outbound(0, heartbeat(), 1, 1, wireBytes = bytes)
        val line = JsonObject.render { o -> entry.writeBody(o) }
        assertTrue(line, line.contains("\"hex\":\"fd0900002a\""))
        assertTrue(line, line.contains("\"hexsrc\":\"wire\""))
    }

    @Test
    fun `a payload with no MAVLink annotation degrades instead of throwing`() {
        val entry = MavlinkTap.outbound(0, "not a mavlink message", 1, 1)
        assertEquals(-1, entry.messageId)
        // No hex is available, so the entry must not claim a source for it.
        val line = JsonObject.render { o -> entry.writeBody(o) }
        assertTrue(line, !line.contains("hexsrc"))
        assertTrue(line, !line.contains("\"hex\""))
    }

    @Test
    fun `hex is lowercase and greppable`() {
        assertEquals("00fd7f80ff", MavlinkTap.hex(
            byteArrayOf(0, 0xfd.toByte(), 0x7f, 0x80.toByte(), 0xff.toByte())
        ))
    }
}
