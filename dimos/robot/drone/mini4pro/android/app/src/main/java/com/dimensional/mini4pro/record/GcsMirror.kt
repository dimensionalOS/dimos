package com.dimensional.mini4pro.record

import io.dronefleet.mavlink.common.DebugVect
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.NamedValueFloat
import io.dronefleet.mavlink.common.Statustext
import io.dronefleet.mavlink.util.EnumValue
import java.math.BigInteger

/**
 * Mirrors a small, deliberate set of our internals to the GCS, so they land in
 * QGroundControl's `.tlog` automatically — timestamped on QGC's clock, plotted in
 * its analyse view, and replayable there with no tooling of ours.
 *
 * ## This is a cross-check, not the record
 *
 * The phone-local JSONL file is the forensic record. This exists because the tlog is
 * free, is written by software that is not ours, and is already aligned with
 * everything else QGC saw. Two independent records disagreeing is itself a finding.
 *
 * ## Bandwidth, honestly
 *
 * The RC-N2 downlink shares its bandwidth with video, so this must not be a flood.
 * Costs, computed from the field sizes in `ref/mavlink/src/main/java-gen`
 * (MAVLink 2 framing is 12 bytes, and we do not sign):
 *
 * | message | payload | frame | count |
 * |---|---|---|---|
 * | `DEBUG_VECT` | 10 name + 8 time_usec + 12 xyz = 30 B | 42 B | 4 |
 * | `NAMED_VALUE_FLOAT` | 4 time_boot_ms + 10 name + 4 value = 18 B | 30 B | 5 |
 *
 * 4 × 42 + 5 × 30 = **318 bytes per cycle**, plus 28 bytes of UDP/IP header per
 * datagram (9 datagrams = 252 B), so **570 B per cycle on the wire**:
 *
 * | rate | bytes/s | bits/s |
 * |---|---|---|
 * | 1 Hz | 570 | 4.6 kbit/s |
 * | 2 Hz (default) | 1140 | 9.1 kbit/s |
 * | 5 Hz | 2850 | 22.8 kbit/s |
 *
 * For comparison, the existing telemetry set is roughly 1.4 kB/s at 5 Hz. So the
 * default doubles our MAVLink volume — which is why [MirrorConfig.onlyWhenEngaged]
 * defaults to true: the mirror is silent until virtual stick is actually engaged,
 * i.e. exactly during the seconds it is meant to explain. It is switchable in full
 * via [MirrorConfig.enabled].
 *
 * ## The names, and what each one means
 *
 * `NAMED_VALUE_FLOAT.name` and `DEBUG_VECT.name` are both 10-character fields, so
 * the names are terse. Every one is documented here and in
 * `docs/flight-recording.md`; a name whose meaning is not written down is a name
 * nobody will trust in six months.
 *
 * | name | type | meaning |
 * |---|---|---|
 * | `cmdvel` | DEBUG_VECT | **commanded** velocity, m/s, x=north y=east z=**down** |
 * | `achvel` | DEBUG_VECT | **achieved** velocity from `KeyAircraftVelocity`, same frame |
 * | `vstick` | DEBUG_VECT | what we handed DJI: x=pitch y=roll z=verticalThrottle |
 * | `rcstick` | DEBUG_VECT | human's sticks, normalised ±1: x=rightH y=rightV z=leftV |
 * | `vs_yaw` | NAMED_VALUE_FLOAT | DJI yaw axis value as sent (the 4th stick axis) |
 * | `cmd_yr` | NAMED_VALUE_FLOAT | commanded yaw rate, deg/s |
 * | `rc_lh` | NAMED_VALUE_FLOAT | human's left-horizontal (yaw) stick, normalised ±1 |
 * | `vs_mode` | NAMED_VALUE_FLOAT | packed control modes — see [StickModes.code] |
 * | `vs_auth` | NAMED_VALUE_FLOAT | authority + engagement — see [authorityCode] |
 *
 * `cmdvel` against `achvel` is the high-value pair: commanded versus achieved per
 * axis, on one plot, in one file QGC wrote itself. The scenario "I told it to go
 * right and it went up" is visible in those two vectors alone.
 */
object GcsMirror {

    data class MirrorConfig(
        /** Master switch. Off means not one extra byte on the link. */
        val enabled: Boolean = true,
        /** Cycles per second. See the bandwidth table before raising this. */
        val hz: Double = 2.0,
        /**
         * Send only while virtual stick is engaged. On by default: outside guided
         * control there is no commanded setpoint to compare against, so the mirror
         * would be spending downlink to say "zero".
         */
        val onlyWhenEngaged: Boolean = true,
        /** Mirror recorder events as `STATUSTEXT` so the tlog carries the timeline. */
        val events: Boolean = true,
    )

    /**
     * Everything one mirror cycle needs, as plain numbers. No DJI types, no
     * Android, so this whole file is unit-testable.
     */
    data class Sample(
        val timeBootMs: Long,
        val commandedNorth: Double? = null,
        val commandedEast: Double? = null,
        val commandedDown: Double? = null,
        val commandedYawRate: Double? = null,
        val achievedNorth: Double? = null,
        val achievedEast: Double? = null,
        val achievedDown: Double? = null,
        val axes: StickAxes? = null,
        val modes: StickModes? = null,
        val vsEnabled: Boolean? = null,
        val vsAdvanced: Boolean? = null,
        /** `RC` | `MSDK` | … as reported by `VirtualStickState`. */
        val authority: String? = null,
        val rcLeftHorizontal: Int? = null,
        val rcLeftVertical: Int? = null,
        val rcRightHorizontal: Int? = null,
        val rcRightVertical: Int? = null,
    )

    const val NAME_CMD_VEL = "cmdvel"
    const val NAME_ACH_VEL = "achvel"
    const val NAME_V_STICK = "vstick"
    const val NAME_RC_STICK = "rcstick"
    const val NAME_VS_YAW = "vs_yaw"
    const val NAME_CMD_YAW_RATE = "cmd_yr"
    const val NAME_RC_LEFT_H = "rc_lh"
    const val NAME_VS_MODE = "vs_mode"
    const val NAME_VS_AUTH = "vs_auth"

    /**
     * Builds one cycle's messages. Returns an empty list when [MirrorConfig] says
     * to stay quiet, so the caller needs no conditional of its own.
     *
     * A missing value is sent as `NaN` rather than 0. A commanded velocity of 0 and
     * "we have no commanded velocity" are completely different claims, and on a plot
     * a fabricated zero is indistinguishable from a real command to hold still —
     * which is precisely the ambiguity this whole exercise exists to remove. QGC
     * draws NaN as a gap.
     */
    fun cycle(sample: Sample, config: MirrorConfig = MirrorConfig()): List<Any> {
        if (!config.enabled) return emptyList()
        if (config.onlyWhenEngaged && sample.vsEnabled != true) return emptyList()

        val usec = BigInteger.valueOf(sample.timeBootMs * 1000L)
        val out = ArrayList<Any>(9)

        out.add(vect(NAME_CMD_VEL, usec, sample.commandedNorth, sample.commandedEast, sample.commandedDown))
        out.add(vect(NAME_ACH_VEL, usec, sample.achievedNorth, sample.achievedEast, sample.achievedDown))
        out.add(vect(NAME_V_STICK, usec, sample.axes?.pitch, sample.axes?.roll, sample.axes?.verticalThrottle))
        out.add(
            vect(
                NAME_RC_STICK, usec,
                LogEntry.RcStick.norm(sample.rcRightHorizontal),
                LogEntry.RcStick.norm(sample.rcRightVertical),
                LogEntry.RcStick.norm(sample.rcLeftVertical),
            )
        )
        out.add(named(NAME_VS_YAW, sample.timeBootMs, sample.axes?.yaw))
        out.add(named(NAME_CMD_YAW_RATE, sample.timeBootMs, sample.commandedYawRate))
        out.add(named(NAME_RC_LEFT_H, sample.timeBootMs, LogEntry.RcStick.norm(sample.rcLeftHorizontal)))
        out.add(named(NAME_VS_MODE, sample.timeBootMs, sample.modes?.code()?.toDouble()))
        out.add(named(NAME_VS_AUTH, sample.timeBootMs, authorityCode(sample).toDouble()))
        return out
    }

    /**
     * Engagement and authority packed into one plottable number:
     * `enabled*100 + advanced*10 + authority`, where authority is DJI's own
     * `FlightControlAuthority` integer (`RC=0 MSDK=1 AUTO_TEST=2 OSDK=3 AIRPORT=4`,
     * recovered by javap; `9` for unknown or absent).
     *
     * So `111` is "virtual stick on, advanced mode, we hold authority" — the only
     * value under which a stick command means anything — and `100` is the shape of a
     * takeback: still enabled, but the RC has the aircraft.
     */
    fun authorityCode(sample: Sample): Int {
        val auth = when (sample.authority) {
            "RC" -> 0
            "MSDK" -> 1
            "AUTO_TEST" -> 2
            "OSDK" -> 3
            "AIRPORT" -> 4
            else -> 9
        }
        val enabled = when (sample.vsEnabled) { true -> 1; false -> 0; null -> 9 }
        val advanced = when (sample.vsAdvanced) { true -> 1; false -> 0; null -> 9 }
        return enabled * 100 + advanced * 10 + auth
    }

    /**
     * A recorder event as `STATUSTEXT`, so the tlog carries the same timeline
     * anchors the JSONL does. Truncated to the 50-character field rather than
     * silently overflowing.
     */
    fun statusText(event: LogEntry.Event): Statustext {
        val severity = when (event.severity) {
            LogEntry.SEV_ERROR -> MavSeverity.MAV_SEVERITY_ERROR
            LogEntry.SEV_WARN -> MavSeverity.MAV_SEVERITY_WARNING
            else -> MavSeverity.MAV_SEVERITY_INFO
        }
        val text = (event.message?.let { "${event.code}: $it" } ?: event.code).take(STATUSTEXT_CHARS)
        return Statustext.builder()
            .severity(EnumValue.of(severity))
            .text(text)
            .build()
    }

    private fun vect(name: String, usec: BigInteger, x: Double?, y: Double?, z: Double?) =
        DebugVect.builder()
            .name(name)
            .timeUsec(usec)
            .x(f(x))
            .y(f(y))
            .z(f(z))
            .build()

    private fun named(name: String, timeBootMs: Long, value: Double?) =
        NamedValueFloat.builder()
            .timeBootMs(timeBootMs)
            .name(name)
            .value(f(value))
            .build()

    private fun f(v: Double?): Float = v?.toFloat() ?: Float.NaN

    /** `STATUSTEXT.text` is a 50-byte field (`ref/mavlink/.../Statustext.java`). */
    const val STATUSTEXT_CHARS = 50

    /** Bytes per mirror cycle on the wire, including UDP/IP headers. See the class doc. */
    const val CYCLE_WIRE_BYTES = 4 * (42 + 28) + 5 * (30 + 28)
}
