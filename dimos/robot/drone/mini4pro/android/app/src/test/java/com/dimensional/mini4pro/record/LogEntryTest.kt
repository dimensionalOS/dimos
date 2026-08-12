package com.dimensional.mini4pro.record

import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Pins the wire schema with hand-written literals.
 *
 * `tools/flightlog` reads these field names, and its self-test generates synthetic
 * sessions from the same names in Python. Nothing links the two automatically, so
 * this test is the pin: if a field is renamed here it fails, which is the prompt to
 * change the tool and the doc in the same commit. An assertion that re-derived the
 * expected JSON from the entry object would pass under any rename and prove nothing
 * — the same trap `docs/verification.md` calls out for the encoder tests.
 */
class LogEntryTest {

    /** Renders an entry the way [FlightRecorder.render] does, with t = 0. */
    private fun line(e: LogEntry): String = JsonObject.render { o ->
        o.put("t", 0.0, 6)
        o.put("k", e.kind)
        e.writeBody(o)
    }

    @Test
    fun `dji_state carries DJI-native units, unconverted`() {
        val state = AircraftState(
            latitude = 37.9938612,
            longitude = 23.7253298,
            relativeAltitude = 0.0,
            rollDeg = -1.0,
            pitchDeg = 0.0,
            yawDeg = -121.1,
            velocityNorth = 0.0,
            velocityEast = 0.0,
            velocityDown = 0.0,
        )
        assertEquals(
            """{"t":0,"k":"dji_state","lat":37.9938612,"lon":23.7253298,"relalt":0,""" +
                """"roll":-1,"pitch":0,"yaw":-121.1,"vn":0,"ve":0,"vd":0}""",
            line(LogEntry.DjiState.of(0, state)),
        )
    }

    @Test
    fun `dji_state omits fields the aircraft did not report`() {
        val line = line(LogEntry.DjiState.of(0, AircraftState(latitude = 1.0)))
        assertEquals("""{"t":0,"k":"dji_state","lat":1}""", line)
        assertFalse("a missing reading must not appear as 0", line.contains("\"vn\""))
    }

    @Test
    fun `dji_state carries how old each reading is, beside the reading`() {
        // The ground-probe case, written down: position arriving at ~50 Hz while
        // KeyAircraftVelocity has not fired for 34 s. The velocity trace is flat
        // in both the "not moving" and the "not updating" case; only `age.vel`
        // separates them, and only if it is on the same line.
        val state = AircraftState(
            latitude = 37.9938612,
            longitude = 23.7253298,
            velocityNorth = 0.0,
            velocityEast = 0.0,
            velocityDown = 0.0,
            ages = SampleAges.of(
                Signal.POSITION to 20L,
                Signal.ALTITUDE to 20L,
                Signal.ATTITUDE to 520L,
                Signal.VELOCITY to 34_120L,
            ),
        )
        assertEquals(
            """{"t":0,"k":"dji_state","lat":37.9938612,"lon":23.7253298,"vn":0,"ve":0,"vd":0,""" +
                """"age":{"pos":20,"relalt":20,"att":520,"vel":34120}}""",
            line(LogEntry.DjiState.of(0, state)),
        )
    }

    @Test
    fun `an age that is unknown is omitted, exactly like an unknown reading`() {
        // Absent means never delivered. Writing 0 would claim the opposite — that
        // it arrived in this very millisecond.
        val line = line(LogEntry.DjiState.of(0, AircraftState(latitude = 1.0)))
        assertEquals("""{"t":0,"k":"dji_state","lat":1}""", line)
        assertFalse("no age at all means the age object disappears", line.contains("\"age\""))

        val partial = line(
            LogEntry.DjiState.of(
                0,
                AircraftState(latitude = 1.0, ages = SampleAges.of(Signal.POSITION to 7L)),
            )
        )
        assertEquals("""{"t":0,"k":"dji_state","lat":1,"age":{"pos":7}}""", partial)
    }

    @Test
    fun `stick_cmd keeps setpoint, DJI axes and control modes in one entry`() {
        val entry = LogEntry.StickCmd(
            monoNanos = 0,
            sequence = 7,
            setpoint = Setpoint(SetpointFrame.NED_VELOCITY, north = 0.0, east = 2.0, down = 0.0),
            axes = StickAxes(pitch = 0.0, roll = 2.0, yaw = 0.0, verticalThrottle = 0.0),
            modes = StickModes("VELOCITY", "ANGULAR_VELOCITY", "VELOCITY", "GROUND", advanced = true),
            source = CommandSource("SET_POSITION_TARGET_LOCAL_NED", 118, 0),
            range = StickRange(rollPitchMax = 15.0, verticalMax = 5.0, yawMax = 100.0),
            path = StickPath.ADVANCED_PARAM,
            accepted = true,
        )
        assertEquals(
            """{"t":0,"k":"stick_cmd","seq":7,""" +
                """"src":{"name":"SET_POSITION_TARGET_LOCAL_NED","seq":118,"t":0},""" +
                """"sp":{"frame":"NED_VELOCITY","vn":0,"ve":2,"vd":0,"units":"m/s;deg"},""" +
                """"dji":{"pitch":0,"roll":2,"yaw":0,"throttle":0},""" +
                """"modes":{"rp":"VELOCITY","yaw":"ANGULAR_VELOCITY","vert":"VELOCITY","coord":"GROUND","adv":true,"code":1100},""" +
                """"range":{"rp_max":15,"vert_max":5,"yaw_max":100},""" +
                """"path":"advanced_param","accepted":true}""",
            line(entry),
        )
    }

    @Test
    fun `control mode code packs the four DJI enums into one plottable integer`() {
        // rp*1000 + yaw*100 + vert*10 + coord, each digit the DJI enum's own value.
        assertEquals(0, StickModes("ANGLE", "ANGLE", "VELOCITY", "GROUND", true).code())
        assertEquals(1100, StickModes("VELOCITY", "ANGULAR_VELOCITY", "VELOCITY", "GROUND", true).code())
        assertEquals(1111, StickModes("VELOCITY", "ANGULAR_VELOCITY", "POSITION", "BODY", true).code())
        assertEquals(2011, StickModes("POSITION", "ANGLE", "POSITION", "BODY", true).code())
        // The failure mode this exists for: a horizontal command sent in ANGLE mode
        // must not be confusable with the same numbers in VELOCITY mode.
        assertTrue(
            StickModes("ANGLE", "ANGULAR_VELOCITY", "VELOCITY", "GROUND", true).code() !=
                StickModes("VELOCITY", "ANGULAR_VELOCITY", "VELOCITY", "GROUND", true).code()
        )
        // unknown / absent is 9 in every position, so it can never be mistaken for a mode
        assertEquals(9999, StickModes.UNKNOWN.code())
    }

    @Test
    fun `rc_stick carries raw SDK counts and a normalised copy`() {
        assertEquals(
            """{"t":0,"k":"rc_stick","lh":0,"lv":0,"rh":660,"rv":-330,""" +
                """"n":{"lh":0,"lv":0,"rh":1,"rv":-0.5}}""",
            line(LogEntry.RcStick(0, 0, 0, 660, -330)),
        )
    }

    @Test
    fun `vs_state records engagement, advanced mode, authority and DJI's own reason`() {
        assertEquals(
            """{"t":0,"k":"vs_state","on":true,"adv":true,"auth":"RC","reason":"NEAR_BOUNDARY"}""",
            line(LogEntry.VsState(0, enabled = true, advanced = true, authority = "RC", changeReason = "NEAR_BOUNDARY")),
        )
    }

    @Test
    fun `drop records where entries were lost, not just how many`() {
        assertEquals(
            """{"t":0,"k":"drop","n":12,"from":83412000000,"to":83605000000,"total":12,""" +
                """"kinds":{"dji_state":12}}""",
            line(
                LogEntry.Drop(
                    monoNanos = 0, count = 12,
                    firstMonoNanos = 83_412_000_000, lastMonoNanos = 83_605_000_000,
                    total = 12, kindCountsJson = """{"dji_state":12}""",
                )
            ),
        )
    }

    @Test
    fun `the entries a post-mortem is built from are marked urgent`() {
        // Urgent means fsync immediately: these must be on the flash before the next
        // one arrives, because they are the discrete moments a timeline hangs on.
        assertTrue(LogEntry.Event(0, EventCode.MODE_CHANGE).urgent)
        assertTrue(LogEntry.VsState(0, true, true, "MSDK").urgent)
        assertTrue(LogEntry.Field(0, "flightMode", "APAS", null).urgent)
        assertTrue(LogEntry.Drop(0, 1, 0, 0, 1).urgent)
        // ... and the high-rate ones are not, or every sample would cost an fsync.
        assertFalse(LogEntry.DjiState.of(0, AircraftState()).urgent)
        assertFalse(LogEntry.RcStick(0, 0, 0, 0, 0).urgent)
        assertFalse(
            LogEntry.StickCmd(
                0, 1, null, StickAxes(0.0, 0.0, 0.0, 0.0), StickModes.UNKNOWN,
                path = StickPath.ADVANCED_PARAM,
            ).urgent
        )
    }

    @Test
    fun `every kind constant is distinct`() {
        val kinds = listOf(
            LogEntry.KIND_HEADER, LogEntry.KIND_MAV_IN, LogEntry.KIND_MAV_OUT,
            LogEntry.KIND_DJI_STATE, LogEntry.KIND_DJI_FIELD, LogEntry.KIND_STICK_CMD,
            LogEntry.KIND_RC_STICK, LogEntry.KIND_VS_STATE, LogEntry.KIND_EVENT,
            LogEntry.KIND_DROP, LogEntry.KIND_STATS,
        )
        assertEquals(kinds.size, kinds.toSet().size)
    }
}
