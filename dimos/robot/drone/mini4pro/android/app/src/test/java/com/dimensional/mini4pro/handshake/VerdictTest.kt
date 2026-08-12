package com.dimensional.mini4pro.handshake

import com.dimensional.mini4pro.command.Verdict
import io.dronefleet.mavlink.common.MavResult
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The one seam this project's decisions cross to reach QGroundControl: [Verdict] to `MAV_RESULT`.
 *
 * The refactor that introduced [Verdict] claimed to be a **rename with no behaviour change** — the
 * engine, the dispatcher and the gimbal manager return a transport-neutral value where they used
 * to return a MAVLink one, and this function puts the MAVLink one back on at the edge. That claim
 * is only true if the mapping is the identity on the five values this project returns, and the
 * whole suite of ack assertions elsewhere (`CommandDispatcherTest`, `GimbalManagerTest`,
 * `HandshakeResponderTest`) rests on it silently. This file makes it rest on it out loud.
 *
 * Three properties, and they are deliberately not the same property said three times:
 *
 *  1. **The table**, verdict by verdict. Catches a swapped pair, which is the mistake that would
 *     turn a refusal into the `ACCEPTED` that makes QGC arm.
 *  2. **Injectivity.** Two verdicts collapsing onto one `MAV_RESULT` would silently delete a
 *     distinction the deciding code makes — `UNSUPPORTED` (the interlock is off) reading as
 *     `DENIED` (commands are on and this one was refused) is exactly the pair `onTakeoff`'s KDoc
 *     says an operator must be able to tell apart without looking at the app.
 *  3. **Coverage**, i.e. the reverse direction: every `MAV_RESULT` this project ever put on the
 *     wire from a decision is still reachable. A verdict quietly dropped from the enum would
 *     otherwise show up only as a result QGC never receives again.
 *
 * Exhaustiveness is enforced by the compiler rather than here — `toMavResult` is a `when` over
 * [Verdict] with no `else`, so a sixth value fails the build. What this file adds is that the
 * `when` cannot be *satisfied wrongly*: [everyVerdictIsMapped] iterates `Verdict.values()`, so a
 * new value has to be given a row in the table below before this test can pass.
 *
 * Mutation-checked 2026-07-27, one breakage at a time, failing tests counted and the code
 * reverted after each — measured counts, not estimates. Two columns because both numbers say
 * something: this file alone, and the whole suite, which is what the mapping is silently holding
 * up:
 *
 *  | mutation | this file | whole suite |
 *  |---|---|---|
 *  | `DENIED` mapped to `MAV_RESULT_ACCEPTED` | 3 | 15 |
 *  | `UNSUPPORTED` mapped to `MAV_RESULT_DENIED` | 3 | 12 |
 *  | `TEMPORARILY_REJECTED` and `FAILED` swapped | 2 | 7 |
 *  | `FAILED` collapsed onto `MAV_RESULT_DENIED` | 3 | **4 — see below** |
 *  | `ACCEPTED` mapped to `MAV_RESULT_TEMPORARILY_REJECTED` | 3 | 13 |
 *  | `TEMPORARILY_REJECTED` mapped to `MAV_RESULT_IN_PROGRESS` | 4 | 8 |
 *
 * The collapse row is why the injectivity test earns its place. Only **one** test outside this
 * file noticed `FAILED` and `DENIED` becoming the same wire value, because almost nothing asserts
 * on a `FAILED` ack — a throwing DJI layer is rare on the bench. It is not rare in the failure
 * mode this project exists to prevent, where "the aircraft refused" and "the bridge broke" are
 * different things an operator must be able to tell apart.
 */
class VerdictTest {

    /**
     * The mapping, spelled out once so the assertions below share one table rather than three
     * independent hand-copies of it.
     */
    private val expected = mapOf(
        Verdict.ACCEPTED to MavResult.MAV_RESULT_ACCEPTED,
        Verdict.DENIED to MavResult.MAV_RESULT_DENIED,
        Verdict.UNSUPPORTED to MavResult.MAV_RESULT_UNSUPPORTED,
        Verdict.TEMPORARILY_REJECTED to MavResult.MAV_RESULT_TEMPORARILY_REJECTED,
        Verdict.FAILED to MavResult.MAV_RESULT_FAILED,
    )

    @Test
    fun `each verdict maps to the MAV_RESULT the deciding code used to return`() {
        for ((verdict, result) in expected) {
            assertEquals("Verdict.$verdict", result, verdict.toMavResult())
        }
    }

    /**
     * Forward direction, totality: a value added to [Verdict] without a decision about what it
     * means on the wire fails here, not in flight.
     */
    @Test
    fun everyVerdictIsMapped() {
        assertEquals(
            "every Verdict needs a row in this test's table",
            Verdict.values().toSet(),
            expected.keys,
        )
        for (verdict in Verdict.values()) {
            assertEquals(expected[verdict], verdict.toMavResult())
        }
    }

    /**
     * Reverse direction: the five `MAV_RESULT`s this project puts on the wire from a decision are
     * each produced by exactly one verdict.
     *
     * Injectivity is the safety half. A collapse would not fail any ack assertion elsewhere — it
     * would simply make two different refusals indistinguishable to the operator.
     */
    @Test
    fun `the five wire results are each reachable from exactly one verdict`() {
        val produced = Verdict.values().map { it.toMavResult() }
        assertEquals("no two verdicts may share a wire result", produced.size, produced.toSet().size)
        val wire = setOf(
            MavResult.MAV_RESULT_ACCEPTED,
            MavResult.MAV_RESULT_DENIED,
            MavResult.MAV_RESULT_UNSUPPORTED,
            MavResult.MAV_RESULT_TEMPORARILY_REJECTED,
            MavResult.MAV_RESULT_FAILED,
        )
        assertEquals(wire, produced.toSet())
    }

    /**
     * `MAV_RESULT_IN_PROGRESS` has no verdict, and must not acquire one.
     *
     * `HandshakeResponder.registerCommandHandler` forbids it: QGC keeps a command answered
     * `IN_PROGRESS` pending forever and will not send the next one. Pinned here because [Verdict]
     * is where somebody would add it, one enum value at a time, with the best of intentions.
     */
    @Test
    fun `no verdict answers IN_PROGRESS`() {
        assertTrue(Verdict.values().none { it.toMavResult() == MavResult.MAV_RESULT_IN_PROGRESS })
    }
}
