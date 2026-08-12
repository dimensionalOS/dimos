package com.dimensional.mini4pro.handshake

import com.dimensional.mini4pro.command.Verdict
import io.dronefleet.mavlink.common.MavResult

/**
 * The MAVLink edge of [Verdict] — the one place a transport-neutral decision becomes a
 * `COMMAND_ACK` result, and the only thing this refactor added to `handshake/`.
 *
 * It lives here rather than beside [Verdict] because this is the side that knows about MAVLink:
 * `handshake/` is where `registerCommandHandler`'s `(CommandRequest) -> MavResult` contract lives
 * and where the ack is actually built. `command/` stays free of the dialect jar, which is the
 * whole point of the enum.
 *
 * ## Written as an exhaustive `when`, on purpose
 *
 * No `else`, and no default. Every value of [Verdict] is listed, so adding a sixth is a
 * compilation failure here rather than a silent fall-through to whatever the default happened to
 * be. That is the same discipline `CommandDispatcher.invoke` uses for `FlightAction` and for the
 * same reason: on this path a lost case is an ack that says something untrue about an aircraft.
 *
 * The mapping is an identity, not a policy. Each verdict names the `MAV_RESULT` the deciding code
 * used to return literally, so this refactor is provably a rename — see `VerdictTest` for the
 * table pinned in both directions.
 */
fun Verdict.toMavResult(): MavResult = when (this) {
    Verdict.ACCEPTED -> MavResult.MAV_RESULT_ACCEPTED
    Verdict.DENIED -> MavResult.MAV_RESULT_DENIED
    Verdict.UNSUPPORTED -> MavResult.MAV_RESULT_UNSUPPORTED
    Verdict.TEMPORARILY_REJECTED -> MavResult.MAV_RESULT_TEMPORARILY_REJECTED
    Verdict.FAILED -> MavResult.MAV_RESULT_FAILED
}
