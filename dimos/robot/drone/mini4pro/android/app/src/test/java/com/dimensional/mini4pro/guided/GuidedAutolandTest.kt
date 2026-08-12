package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.ActionPort
import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.MsdkFlightActions
import com.dimensional.mini4pro.command.Verdict
import com.dimensional.mini4pro.mavlink.StatusTextSink
import com.dimensional.mini4pro.record.CommandSource
import com.dimensional.mini4pro.record.Setpoint
import com.dimensional.mini4pro.record.StickAxes
import com.dimensional.mini4pro.record.StickModes
import com.dimensional.mini4pro.record.StickRange
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import com.dimensional.mini4pro.vision.RangeSource
import io.dronefleet.mavlink.common.ManualControl
import io.dronefleet.mavlink.common.Statustext
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **Stage C — the full autoland as landing04 taught it must work.** The first Stage C design
 * (mimic the pilot's held stick) was refuted in the air the day it was built
 * (`datasets/landing04/20260728-174923.001.jsonl`): the FC floors a virtual-stick descent at
 * ~1.4 m via downward obstacle sensing, holds it indefinitely (12 s measured under a
 * continuous `vd = +0.4`), never raises `KeyIsLandingConfirmationNeeded` — and honours the
 * identical stick-down gesture only from the physical RC. The only landing software gets is
 * DJI's own, so the shipped design **commits**: at the measured FC floor (or the terminal
 * hold), on a fresh in-cone fix under the 2.5 m sanity ceiling, one `land()` through the
 * `DjiLanding` seam, then zero sticks while DJI flies, rule 1 gaining its one action
 * (`KeyStopAutoLanding`), touchdown as motors-off.
 *
 * This suite drives [GuidedStickEngine] — and, for the confirmation gate, a real
 * `MsdkFlightActions` across the same seams `Bridge` wires. Same protocol as
 * `GuidedTagDescentTest`: fake port, hand-cranked clock, no aircraft. The flown-sequence
 * fixtures are landing04's shapes: the 1.4 m floor, the cone-gated 0/0.4 `vd` wobble, the
 * −79° → −33° recenter, `CONFIRM_LANDING`, touchdown ~3 s later — plus landing06's
 * (`datasets/landing06/20260728-205913.001.jsonl`): the 12 cm/s handover momentum and the
 * 3.2 s blind final — plus landing07's (`datasets/landing07/20260729-095413.001.jsonl`): the
 * ~1.2 m baro lie, the size range that was right at 51–57 px, and the tilted-camera fixes
 * measured entering the target window after the commit.
 *
 * **The committed-phase property, as of the odometric pivot** (successor to "our sticks must
 * be exactly neutral", deliberately — not a deletion): the committed phase never commands
 * vertical or yaw; its lateral is the landing law on the frozen window-median of believed
 * fixes, and it dies to zeros the moment the position feed cannot vouch for the aircraft —
 * a dead stick, never a random walk, never an abort.
 *
 * Written to fail loudly for the pivot's landmines:
 *
 *  - a commit without the operator's option, the floor verdict, a fresh in-cone fix, under
 *    the ceiling, or **measurably slow** — `KeyStartAutoLanding` onto ground nobody vouched
 *    for, onto an obstacle, or carrying the momentum that becomes touchdown miss (landing06)
 *  - a second `land()` for one engagement — double-commit
 *  - vertical or yaw commanded while DJI lands, or lateral past the landing's own cap
 *  - blind-final steering surviving a stale position — the random walk, landmine 7
 *  - the steering target re-derived from an unbelieved fix (foreign id, rejuvenated age),
 *    dragged by a last-frame straggler, or averaged into the live tracking loop
 *  - a commit refusal retried, or absorbed silently into a hover at the FC floor
 *  - a commit DJI accepted and never enacted, waited on forever
 *  - rule 1 weakened, or firing without its stop — handing the pilot an aircraft that is
 *    still autonomously descending (and now steering)
 *  - a landing ended by tag loss, latch loss, the camera moving, or DJI's own landing mode
 *  - a confirm sent blind, out of cone, past the interlock, or twice
 *  - touchdown read from the altitude, whose floor is below zero (measured)
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-28 against the pivoted tree (2424 tests), one breakage at a time,
 * whole suite per mutant, test-results deleted first, confirmed red, reverted — counts are
 * failing tests across the whole suite, **measured, not estimated**. Unit twins for the
 * confirm rows in `MsdkFlightActionsTest`; law rows also summarised in
 * `TagDescentGuidanceTest`; the `GimbalRecenter` rows keep their own table.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | commit: floor conjunct dropped (any fresh in-cone tick commits) | 8 |
 *  | commit: fresh-fix conjunct dropped (stale rungs commit) | 1 |
 *  | commit: cone conjunct dropped | 1 |
 *  | commit: `fullAutoland` ignored at both entry points | 7 |
 *  | commit: ceiling conjunct dropped | 1 |
 *  | `COMMIT_CEILING_M` broken (2.5 → 25 m) | 1 |
 *  | floor window broken (`LAND_STALL_MS` 2 s → 0.1 s) | 2 |
 *  | floor window never resets on progress | 1 |
 *  | the law re-emits the commit edge every committed tick | 2 |
 *  | the engine re-asks land() on every committed tick (double-commit) | 20 |
 *  | commit refusal absorbed (run survives a failed land()) | 2 |
 *  | never-engaged timeout dropped | 1 |
 *  | timeout fires even while DJI is measurably landing | 2 |
 *  | committed phase still descends (the machine's zero broken) | 4 |
 *  | rule 1's stop dropped (RC abort no longer sends KeyStopAutoLanding) | 3 |
 *  | rule 1's stop on the GCS channel dropped | 1 |
 *  | stop sent on every abort (interlock-off fights DJI too) | 1 |
 *  | disarm/pause no longer withdraw the landing | 1 |
 *  | touchdown detection dropped | 2 |
 *  | touchdown read from altitude ≤ 0 instead of motors-off | 2 |
 *  | landing-mode seizure exemption widened to every mode | 1 |
 *  | latch/nadir cancels not skipped once committed | 1 |
 *  | watchdog rate limit dropped | 1 |
 *  | watchdog attempt bound dropped | 1 |
 *  | watchdog runs outside the committed phase | 1 |
 *  | confirm gate: cone check dropped | 1 |
 *  | confirm gate: freshness check dropped | 2 |
 *  | confirm gate: interlock check dropped | 2 |
 *  | confirm gate: single-confirm discipline dropped | 3 |
 *
 * ### Two survivors, and what each one taught
 *
 * The campaign's first pass produced two `RED=0` runs, both closed before this table was final:
 *
 *  - **The engine's own `fullAutoland` gate on the floor verdict survived** — because the law's
 *    commit condition already carries the option, the engine's copy was dead code: the
 *    two-places-for-one-property failure this project names, caught by the protocol. Closed by
 *    **deleting the redundant gate** (the law is the single owner) rather than by pinning
 *    redundancy with a test; re-measuring the option mutant afterwards killed 7 including the
 *    engine-level plain-descent test that the dead copy had been masking.
 *  - **The window-reset-on-progress mutant survived** — the fixture descended slowly *above*
 *    the commit ceiling, which masked the broken reset. Closed by moving the fixture below the
 *    ceiling and additionally pinning that a progressing descent writes no `landing_stall`
 *    line; re-measured red (1). That new test also kills the floor-conjunct mutant, which is
 *    why its row reads 8: the count was re-measured on the final tree at review (the
 *    campaign's own pass measured 7, before this test existed).
 *
 * ## The odometric blind final and the velocity gate (2026-07-29) — measured kill counts
 *
 * The landing06 features' campaign, same protocol: one breakage at a time against the
 * 2500-test tree, whole suite per mutant, test-results deleted first, confirmed red,
 * reverted. **No survivors.** Law-side rows are summarised in `TagDescentGuidanceTest` too.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | committed phase commands vertical (0.0 → 0.1 in the landing branch) | 5 |
 *  | committed phase commands yaw (0.0 → 1.0) | 4 |
 *  | steering survives a stale position (the engine's freshness conjunct dropped) | 1 |
 *  | landing lateral cap dropped (V_LAND_LATERAL → the 1.0 tracking cap) | 2 |
 *  | steering re-derives the target from the raw sense (id gate bypassed) | 2 |
 *  | speed conjunct dropped at the TERMINAL entry | 2 |
 *  | speed conjunct dropped at the floor entry | 3 |
 *  | `LAND_COMMIT_SPEED_MS` broken (0.05 → 5) | 4 |
 *  | quantisation semantics inverted (0.05 → 0.1: one quantum passes) | 4 |
 *  | rule 1 exempts a committed steering landing (RC rung gated on the phase) | 4 |
 *  | `LAND_TARGET_WINDOW_MS` stretched (0.5 s → 10 s, the height-walk enters) | 3 |
 *  | the window median replaces the LIVE loop's fix | 1 |
 *  | fewer-than-2-samples fallback broken (lonely fix averaged with old heights) | 1 |
 *
 * ## The range ladder and the frozen commit window (2026-07-29, landing07's features) —
 * ## measured kill counts
 *
 * Landing07's campaign (`datasets/landing07/20260729-095413.001.jsonl`: landing A healthy at
 * 16 cm; landing B's ~1.2 m baro lie root-caused to a 46 cm miss), same protocol: one breakage
 * at a time against the 2511-test tree, whole suite per mutant, test-results deleted first,
 * confirmed red, reverted. Law rows are also summarised in `TagDescentGuidanceTest`; the
 * fix-scale rows live in `TagWorldTest`'s ladder table.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | ladder precedence flipped (baro outranks a fresh tag-derived range) | 6 |
 *  | SIZE rung ignores the px floor (a 10 px smudge's "range" believed) | 1 |
 *  | fix lateral scale and law height split (bearing scaled by alt, SIZE claimed) | 1 |
 *  | `range_baro_divergence` record dropped | 2 |
 *  | target window ingests post-commit fixes (the freeze reverted) | 1 |
 *  | floor detector fed the ladder height instead of the baro (seam broken) | **0 → 1** |
 *  | `descentHeight` freshness dropped (a stale tag range flown) | 2 |
 *  | `height_source` record dropped | 2 |
 *
 * ### One survivor, and what it taught
 *
 * The floor-seam mutant scored **RED=0** on its first run — not dead code, a fixture at a
 * floating-point boundary: the test stepped the ladder 1.4 → 1.3 m expecting the broken seam
 * to read one quantum of fake progress, but the reset threshold is `1.4 − 0.1 =
 * 1.2999999999999998` in IEEE 754 and 1.3 sits just above it, so even the broken detector did
 * not reset. A fixture at the exact threshold tests floating point, not the seam. Closed by
 * stepping 0.2 m (landing B's real step was 1.2 m); re-measured red (1).
 *
 * ## The OWN_LANDING retirement re-measurement (2026-07-30)
 *
 * The OWN_LANDING experiment (the third commit destination beside [TagDescentPhase.DJI_LANDING])
 * was retired with its whole scaffolding — `TagDescentPhase.OWN_LANDING`,
 * `command/PerceptionGuard`, the perception ports, `GuidedOwnLandingTest` and
 * `PerceptionGuardTest` with their own measured tables (RED=38 for the with-it-off
 * byte-identity pin, RED=22 for `ownDescentCleared`, one mutant per restore exit path — those
 * counts are history, recorded in commit f7e83cc / a811176's messages and tables) — after
 * landing11/landing12 closed its question: the per-direction downward OA switch is UNSUPPORTED
 * on the Mini 4 Pro (even the read refuses, landing11 t=55.42) and with
 * `ObstacleAvoidanceType=CLOSE` the floor persists at 0.5 m — the un-disableable infrared ToF
 * owns the last half-metre (landing12 t=66.2). `docs/msdk/actions.md` §7 carries the verdict.
 *
 * The restored DJI_LANDING-only commit path was re-pinned against the shrunk tree
 * (2564 tests): **the one-shot `land()` dropped at the commit edge (`seam.start()` never
 * asked) — RED=28, all in this file**, measured 2026-07-30, whole suite, fresh test-results,
 * confirmed red, reverted. Proportionally consistent with the double-commit row's 20 on the
 * 2424-test tree; the restored path is fully pinned with the experiment's tests gone.
 */
class GuidedAutolandTest {

    private companion object {
        const val LAT = 38.0
        const val LON = 23.7
        const val DATUM = 100.0
        const val ALT = 5.0

        /** landing04's measured FC floor. */
        const val FLOOR = 1.4

        fun latNorthOf(metres: Double): Double = LAT + metres / RepositionGuidance.METRES_PER_DEG

        fun lonEastOf(metres: Double): Double =
            LON + metres / (RepositionGuidance.METRES_PER_DEG * kotlin.math.cos(Math.toRadians(LAT)))
    }

    private class FakeVirtualStickPort : VirtualStickPort {
        var unavailable: String? = null
        var enableCalls = 0
        var disableCalls = 0
        var enableOnSuccess: (() -> Unit)? = null

        data class Sent(val pitch: Double, val roll: Double, val yaw: Double, val verticalThrottle: Double)

        val sent = mutableListOf<Sent>()
        var onState: ((VirtualStickSnapshot) -> Unit)? = null
        var onRc: ((RcSticks) -> Unit)? = null

        val modes = StickModes(
            rollPitch = "VELOCITY", yaw = "ANGULAR_VELOCITY", vertical = "VELOCITY",
            coordinateSystem = "GROUND", advanced = true,
        )

        override fun unavailableReason(): String? = unavailable

        override fun enable(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            enableCalls++
            enableOnSuccess = onSuccess
        }

        override fun disable(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            disableCalls++
        }

        override fun setAdvancedMode(enabled: Boolean) = Unit

        override fun sendAdvancedParam(
            pitch: Double, roll: Double, yaw: Double, verticalThrottle: Double,
        ): SendReport {
            sent += Sent(pitch, roll, yaw, verticalThrottle)
            return SendReport(modes, null)
        }

        override fun listenState(
            onState: (VirtualStickSnapshot) -> Unit,
            onAuthorityReason: (String) -> Unit,
        ) {
            this.onState = onState
        }

        override fun listenRcSticks(onDelivery: (RcSticks) -> Unit) {
            this.onRc = onDelivery
        }

        override fun cancelListens() = Unit
    }

    /** The camera seam, recording every ask — the watchdog's audit trail. */
    private class FakeManoeuvreGimbal : ManoeuvreGimbal {
        val aimed = mutableListOf<Double>()
        override fun pitchRangeDeg(): ClosedFloatingPointRange<Double>? = -90.0..60.0
        override fun aimPitch(pitchDeg: Double) {
            aimed += pitchDeg
        }
    }

    /** The commit seam, recording every ask and answering as the test scripts. */
    private class FakeDjiLanding : DjiLanding {
        var startRefusal: String? = null
        var stopRefusal: String? = null
        var starts = 0
        var stops = 0
        var onStart: (() -> Unit)? = null
        var onStop: (() -> Unit)? = null

        override fun start(): String? {
            starts++
            onStart?.invoke()
            return startRefusal
        }

        override fun stop(): String? {
            stops++
            onStop?.invoke()
            return stopRefusal
        }
    }

    private class RecordedCmd(
        val setpoint: Setpoint?, val source: CommandSource?, val accepted: Boolean?,
    )

    private class Harness(withLandingSeam: Boolean = true) {
        var now = 1_000L
        var interlock = true
        var state = stateAt()

        var latchedId: Int? = 0
        var fixId: Int? = 0
        var fixNorth: Double? = 0.0
        var fixEast: Double? = 0.0
        var fixAtMs: Long? = 900L

        /**
         * The fix's range-ladder facts (`TagFix.tagRangeM()` / `rangeSource`). Null by
         * default — a baro-scaled fix, the pre-ladder world — so every fixture that predates
         * landing07 flies byte-identically; landing07 fixtures set the tag's own range.
         */
        var fixTagRange: Double? = null
        var fixRangeSource: RangeSource? = null
        var cameraPitch: Double? = -90.0

        /** The REPORTED gimbal pitch the landing watchdog reads — DJI's, not ours. */
        var reportedGimbalPitch: Double? = -90.0

        val port = FakeVirtualStickPort()
        val gimbal = FakeManoeuvreGimbal()
        val landing = FakeDjiLanding()
        val wire = mutableListOf<Any>()
        val cmds = mutableListOf<RecordedCmd>()
        val events = mutableListOf<Pair<String, String?>>()

        val engine = GuidedStickEngine(
            port = port,
            interlockEnabled = { interlock },
            aircraftState = { state },
            announcer = Announcer(StatusTextSink { wire += it }),
            manoeuvreGimbal = gimbal,
            djiLanding = if (withLandingSeam) landing else null,
            tagSense = {
                TagDescentSense(
                    latched = latchedId != null,
                    latchedTagId = latchedId,
                    fixTagId = fixId,
                    fixNorthM = fixNorth,
                    fixEastM = fixEast,
                    fixAgeMs = fixAtMs?.let { now - it },
                    fixTagRangeM = fixTagRange,
                    fixRangeSource = fixRangeSource,
                )
            },
            cameraPitchDeg = { cameraPitch },
            gimbalReportedPitchDeg = { reportedGimbalPitch },
            record = object : GuidedRecord {
                override fun stickCmd(
                    setpoint: Setpoint?, axes: StickAxes, modes: StickModes,
                    source: CommandSource?, range: StickRange?, accepted: Boolean?, error: String?,
                ) {
                    cmds += RecordedCmd(setpoint, source, accepted)
                }

                override fun event(code: String, message: String?, warn: Boolean) {
                    events += code to message
                }
            },
            nowMs = { now },
        )

        init {
            engine.attach()
            port.onRc!!(RcSticks(0, 0, 0, 0))
        }

        fun texts(): List<String> = wire.filterIsInstance<Statustext>().map { it.text() }

        fun place(
            northM: Double = 0.0, eastM: Double = 0.0, relAlt: Double? = ALT,
            positionAge: Long = 0L,
        ) {
            val flying = state.isFlying
            val motors = state.motorsOn
            val mode = state.flightMode
            state = stateAt(
                latDeg = latNorthOf(northM), lonDeg = lonEastOf(eastM), relAlt = relAlt,
                positionAge = positionAge,
            ).copy(isFlying = flying, motorsOn = motors, flightMode = mode)
        }

        fun seeTag(northM: Double = 0.0, eastM: Double = 0.0) {
            fixNorth = northM
            fixEast = eastM
            fixAtMs = now
        }

        fun arm(fullAutoland: Boolean = true): Verdict {
            engine.onInbound("heartbeat")
            return engine.armTagDescent(fullAutoland = fullAutoland)
        }

        fun confirmEngage() {
            port.enableOnSuccess?.invoke()
            port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "MSDK"))
            tickAlive(40)
        }

        fun tickAlive(advanceMs: Long = 100) {
            now += advanceMs
            engine.onInbound("heartbeat")
            engine.tick(now)
        }

        fun tickSeeing(advanceMs: Long = 100) {
            now += advanceMs
            seeTag(fixNorth ?: 0.0, fixEast ?: 0.0)
            engine.onInbound("heartbeat")
            engine.tick(now)
        }

        fun frame(x: Int = 0, y: Int = 0, z: Int = 500, r: Int = 0) {
            engine.onInbound(ManualControl.builder().target(1).x(x).y(y).z(z).r(r).buttons(0).build(), null)
        }

        /** Arm (autoland), engage, one seeing tick: TRACKING with the option pinned. */
        fun descending(fullAutoland: Boolean = true): Harness {
            seeTag(0.0, 0.0)
            place(relAlt = ALT)
            check(arm(fullAutoland) == Verdict.ACCEPTED)
            confirmEngage()
            tickSeeing()
            return this
        }

        /**
         * The landing04 floor, replayed: descent commanded, in cone, and the altitude pinned
         * at [FLOOR] — the detector needs [TagDescentGuidance.LAND_STALL_MS] of no progress
         * after a tick that already commanded descent; 25 seeing ticks covers it with margin.
         */
        fun stalledAtFloor(ticks: Int = 25): Harness {
            place(northM = 0.05, relAlt = FLOOR)
            repeat(ticks) { tickSeeing() }
            return this
        }

        /** Drive a descending harness through the floor to a committed DJI landing. */
        fun committed(): Harness {
            stalledAtFloor()
            check(engine.situation().descent!!.landing) { "floor commit did not fire" }
            check(landing.starts == 1) { "expected exactly one land() ask, saw ${landing.starts}" }
            return this
        }

        companion object {
            fun stateAt(
                latDeg: Double = LAT, lonDeg: Double = LON, relAlt: Double? = ALT,
                positionAge: Long = 0L,
            ) = AircraftState(
                latitude = latDeg, longitude = lonDeg,
                homeLatitude = LAT, homeLongitude = LON,
                relativeAltitude = relAlt, takeoffAltitudeAmsl = DATUM,
                velocityNorth = 0.0, velocityEast = 0.0, velocityDown = 0.0,
                yawDeg = 0.0,
                ages = SampleAges.of(
                    Signal.POSITION to positionAge,
                    Signal.ALTITUDE to 0L,
                    Signal.VELOCITY to 0L,
                    Signal.ATTITUDE to 0L,
                ),
            )
        }
    }

    private fun lastSend(h: Harness): FakeVirtualStickPort.Sent {
        assertTrue("nothing was sent", h.port.sent.isNotEmpty())
        return h.port.sent.last()
    }

    private fun endedEvents(h: Harness): List<String?> =
        h.events.filter { it.first == "tag_descent_ended" }.map { it.second }

    // ----------------------------------------------------------- the option, the arm

    @Test
    fun `an autoland arm is announced and recorded as one - a plain arm is not`() {
        val auto = Harness()
        auto.seeTag()
        assertEquals(Verdict.ACCEPTED, auto.arm(fullAutoland = true))
        assertTrue(auto.texts().contains(GuidedStatusTexts.DESCENT_ARMED_AUTOLAND))
        assertTrue(auto.events.any { it.first == "tag_descent_armed" && it.second!!.endsWith("autoland") })

        val plain = Harness()
        plain.seeTag()
        assertEquals(Verdict.ACCEPTED, plain.arm(fullAutoland = false))
        assertTrue(plain.texts().contains(GuidedStatusTexts.DESCENT_ARMED))
        assertTrue(plain.events.none { it.first == "tag_descent_armed" && it.second!!.contains("autoland") })
    }

    @Test
    fun `a plain stage B descent at the FC floor stalls, records, and never commits`() {
        val h = Harness().descending(fullAutoland = false)
        h.stalledAtFloor(ticks = 40)
        assertNotNull(h.engine.situation().descent)
        assertFalse(h.engine.situation().descent!!.landing)
        assertTrue("no land() without the option, ever", h.landing.starts == 0)
        // The floor is still measured — landing_stall stayed a record line for plain descents.
        assertEquals(1, h.events.count { it.first == "landing_stall" })
        // And the descent keeps asking, unchanged: the last send still commands down in-cone
        // (DJI sign: up-positive throttle).
        assertTrue(lastSend(h).verticalThrottle < 0.0)
    }

    @Test
    fun `a plain terminal hold never commits either`() {
        val h = Harness().descending(fullAutoland = false)
        h.place(northM = 0.05, relAlt = TagDescentGuidance.TARGET_HEIGHT_M + 0.1)
        repeat(TagDescentGuidance.TERMINAL_TICKS + 10) { h.tickSeeing() }
        assertTrue(h.engine.situation().descent!!.terminal)
        assertEquals(0, h.landing.starts)
    }

    // ------------------------------------------------------------- the flown sequence

    @Test
    fun `the landing04 sequence, as it should have ended - floor, commit, DJI lands, touchdown`() {
        val h = Harness().descending()

        // The FC floor: altitude pinned at 1.4 m under a commanded descent (the measured 12 s
        // hold, compressed). The stall is recorded, then the commit fires — once — with the
        // floor height and the fix age on the record, and the ask goes to DJI.
        h.stalledAtFloor()
        assertEquals(1, h.events.count { it.first == "landing_stall" })
        assertEquals(1, h.landing.starts)
        assertTrue(h.events.any {
            it.first == "landing_commit" && it.second!!.startsWith("height=1.4") &&
                it.second!!.contains("floor=true") && it.second!!.contains("vlat=0.00")
        })
        assertTrue(h.events.any { it.first == "tag_descent_phase" && it.second!!.startsWith("dji_landing") })
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_DJI_LANDING))
        assertTrue(h.engine.situation().descent!!.landing)

        // DJI is flying the descent; ours is the odometric lateral and nothing else — the
        // measured CONFIRM_LANDING mode is not a seizure, and the recenter takes the latch,
        // the camera claim and the tag with it, which ends nothing and does not stop the
        // steering either: the target is a world constant and the position feed still lives.
        // The aircraft sits 5 cm north of the believed tag, so the steered-blind ticks
        // command south at the law's own magnitude, with vertical and yaw exactly zero.
        h.state = h.state.copy(flightMode = "CONFIRM_LANDING")
        h.latchedId = null
        h.cameraPitch = 0.0
        repeat(20) { h.tickAlive() }
        assertNotNull("the landing survives tag/latch/camera loss and its own mode", h.engine.situation().descent)
        assertTrue(h.engine.situation().descent!!.blind)
        val steered = h.cmds.last { it.source?.messageName == GuidedStickEngine.TAG_DESCENT_SOURCE }
        assertEquals(-RepositionGuidance.KP_PER_S * 0.05, steered.setpoint!!.north!!, 1e-9)
        assertEquals(0.0, steered.setpoint.east!!, 1e-9)
        assertEquals("DJI owns the descent", 0.0, steered.setpoint.down!!, 0.0)
        assertEquals(0.0, lastSend(h).verticalThrottle, 0.0)
        assertEquals(0.0, lastSend(h).yaw, 0.0)

        // The position feed dying is the one thing that kills the steering — to a dead stick,
        // recorded on the edge, NEVER to an abort: DJI keeps landing, we just stop helping.
        h.place(northM = 0.05, relAlt = FLOOR, positionAge = 5_000)
        h.state = h.state.copy(flightMode = "CONFIRM_LANDING")
        repeat(3) { h.tickAlive() }
        assertNotNull("position loss must not end a committed landing", h.engine.situation().descent)
        val dead = h.cmds.last { it.source?.messageName == GuidedStickEngine.TAG_DESCENT_SOURCE }
        assertEquals(0.0, dead.setpoint!!.north!!, 0.0)
        assertEquals(0.0, dead.setpoint.east!!, 0.0)
        assertEquals(
            1,
            h.events.count { it.first == "tag_descent_phase" && it.second == "landing steering dead (position)" },
        )

        // …and the feed coming back resumes it, also on the edge.
        h.place(northM = 0.05, relAlt = FLOOR)
        h.state = h.state.copy(flightMode = "CONFIRM_LANDING")
        repeat(3) { h.tickAlive() }
        assertEquals(
            1,
            h.events.count { it.first == "tag_descent_phase" && it.second == "landing steering live" },
        )
        assertTrue(h.cmds.last { it.source?.messageName == GuidedStickEngine.TAG_DESCENT_SOURCE }
            .setpoint!!.north!! < 0.0)

        // Touchdown: motors off (measured ~3 s after mode entry; the clock is DJI's). One
        // engagement, one ended line, released, disabled.
        h.state = h.state.copy(motorsOn = false, isFlying = false)
        h.tickAlive()
        assertNull(h.engine.situation().descent)
        assertEquals(listOf<String?>("touchdown"), endedEvents(h))
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_TOUCHDOWN))
        repeat(3) { h.tickAlive() }
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.port.disableCalls > 0)
        assertEquals(1, h.events.count { it.first == "tag_descent_armed" })
        assertEquals(1, h.landing.starts)
    }

    @Test
    fun `the terminal hold also commits - the floor may sit below 0,6 m on another scene`() {
        val h = Harness().descending()
        h.place(northM = 0.05, relAlt = TagDescentGuidance.TARGET_HEIGHT_M + 0.1)
        repeat(TagDescentGuidance.TERMINAL_TICKS) { h.tickSeeing() }
        assertTrue(h.engine.situation().descent!!.terminal)
        h.tickSeeing()
        assertTrue(h.engine.situation().descent!!.landing)
        assertEquals(1, h.landing.starts)
        assertTrue(h.events.any {
            it.first == "landing_commit" && it.second!!.contains("floor=false")
        })
    }

    @Test
    fun `no commit before the floor window has genuinely elapsed`() {
        val h = Harness().descending()
        h.place(northM = 0.05, relAlt = FLOOR)
        // The window is 2 s; at ~1.8 s of pinned altitude nothing may fire — jitter and one
        // slow quantum must not hand a landing to DJI.
        repeat(19) { h.tickSeeing() }
        assertEquals("no commit inside the stall window", 0, h.landing.starts)
        assertFalse(h.engine.situation().descent!!.landing)
    }

    @Test
    fun `a descent that keeps descending never commits - progress resets the floor window`() {
        val h = Harness().descending()
        // A slow but real descent BELOW the commit ceiling — the height band where a broken
        // window-reset would actually commit: one 0.1 m quantum every 1.5 s, half the healthy
        // rate, still progress. 15 s of it must produce no commit and no stall line, because
        // every quantum of progress starts the window over.
        var alt = 2.4
        repeat(10) {
            h.place(northM = 0.05, relAlt = alt)
            repeat(15) { h.tickSeeing() }
            alt -= 0.1
        }
        assertEquals(0, h.landing.starts)
        assertFalse(h.engine.situation().descent!!.landing)
        assertEquals("a descending aircraft is not stalled", 0,
            h.events.count { it.first == "landing_stall" })
    }

    // ------------------------------------------------------------- the velocity gate

    @Test
    fun `the floor commit waits until the aircraft is measurably slow - momentum is miss`() {
        // landing06: ~12 cm/s carried across the commit became ~21 cm of touchdown drift. A
        // reading of one quantum (0.1 — true speed maybe 0.15 m/s) must hold at the floor,
        // stall line and all; the first slow tick commits.
        val h = Harness().descending()
        h.place(northM = 0.05, relAlt = FLOOR)
        h.state = h.state.copy(velocityNorth = 0.1)
        repeat(40) { h.tickSeeing() }
        assertEquals("no land() while moving", 0, h.landing.starts)
        assertFalse(h.engine.situation().descent!!.landing)
        // The floor is still measured while the gate holds — the stall line is a measurement.
        assertEquals(1, h.events.count { it.first == "landing_stall" })

        h.state = h.state.copy(velocityNorth = 0.0)
        repeat(3) { h.tickSeeing() }
        assertEquals(1, h.landing.starts)
        assertTrue(h.events.any { it.first == "landing_commit" && it.second!!.contains("vlat=0.00") })
    }

    @Test
    fun `a change-only-quiet velocity feed does not deadlock the commit`() {
        // KeyAircraftVelocity is change-only (measured: once in 35 s at rest), so an aircraft
        // pinned at the FC floor legitimately quiets the feed at exactly the moment the gate
        // is consulted. The usableAltitude rule applies verbatim: an unchanged value from a
        // live component is current, POSITION's continuous jitter is the liveness proxy.
        val h = Harness().descending()
        h.place(northM = 0.05, relAlt = FLOOR)
        h.state = h.state.copy(
            ages = SampleAges.of(
                Signal.POSITION to 0L, Signal.ALTITUDE to 0L,
                Signal.VELOCITY to 30_000L, Signal.ATTITUDE to 0L,
            ),
        )
        repeat(30) { h.tickSeeing() }
        assertEquals("a quiet feed over a fresh position is a cached zero, not a mystery", 1, h.landing.starts)
    }

    @Test
    fun `a velocity reading that never arrived can never commit`() {
        // Null is not slow: a feed that cannot vouch for the speed defaults the commit closed,
        // and the descent simply keeps holding at the floor — the safe direction, visibly.
        val h = Harness().descending()
        h.place(northM = 0.05, relAlt = FLOOR)
        h.state = h.state.copy(velocityNorth = null)
        repeat(40) { h.tickSeeing() }
        assertEquals(0, h.landing.starts)
        assertNotNull(h.engine.situation().descent)
    }

    // ------------------------------------------------------- the odometric blind final

    @Test
    fun `blind-final steering flies the believed target - a foreign id cannot move it`() {
        // The believed fix is id-matched and monotonic (2 false ids in 1978 frames, measured);
        // a fresh fix decoding another tag must neither steer the landing nor rejuvenate the
        // ingest. The steering stays on the engagement's own frozen target.
        val h = Harness().descending().committed()
        h.tickAlive()
        val before = h.cmds.last { it.source?.messageName == GuidedStickEngine.TAG_DESCENT_SOURCE }
        assertEquals(-RepositionGuidance.KP_PER_S * 0.05, before.setpoint!!.north!!, 1e-6)

        h.fixId = 7
        h.fixNorth = 50.0
        h.fixAtMs = h.now
        repeat(3) { h.tickAlive() }
        val after = h.cmds.last { it.source?.messageName == GuidedStickEngine.TAG_DESCENT_SOURCE }
        assertEquals("a foreign id steered the landing", -RepositionGuidance.KP_PER_S * 0.05, after.setpoint!!.north!!, 1e-6)
    }

    @Test
    fun `the blind-final target freezes at the commit edge - post-commit fixes cannot move it`() {
        // The pre-landing07 design let the window keep rolling "while the tag survives the
        // first ~0.2 s after commit". Landing07 measured why that loses
        // (`datasets/landing07/20260729-095413.001.jsonl`, both landings): DJI is already
        // recentering the camera within ~0.2 s of the commit (watchdog: −76.7° at commit+0.2
        // in landing A, −71.1° in B) while the pitch belief — commanded-wins — still says
        // −90°, so tilted-camera pixels are computed as nadir and enter the record as
        // plausible fixes (landing A's stepped N +0.115 → +0.033 across the commit). The
        // window now ends at the newest PRE-commit sample: believed fixes at 0.10 N before
        // the commit, "believed" fixes at 0.50 N after it — the steering must fly
        // kp·(0.10 − 0.05) toward the frozen target, byte-for-byte, however long they flow.
        val h = Harness().descending()
        h.fixNorth = 0.10
        h.stalledAtFloor()
        check(h.landing.starts == 1) { "floor commit did not fire" }
        h.fixNorth = 0.50
        repeat(5) { h.tickSeeing() }
        val cmd = h.cmds.last { it.source?.messageName == GuidedStickEngine.TAG_DESCENT_SOURCE }
        assertEquals(RepositionGuidance.KP_PER_S * 0.05, cmd.setpoint!!.north!!, 1e-6)
    }

    @Test
    fun `the live tracking loop steers on the newest fix, unaveraged`() {
        // The averaging window exists for the blind final only: the tracking loop is
        // latency-limited and filtering it lowers the stable gain (landingTarget's KDoc). A
        // tag that genuinely moved in frame must be chased at the newest fix's full error.
        val h = Harness().descending()
        repeat(5) { h.tickSeeing() }
        h.fixNorth = 0.40
        h.tickSeeing()
        val cmd = h.cmds.last { it.source?.messageName == GuidedStickEngine.TAG_DESCENT_SOURCE }
        assertEquals(
            "tracking must chase the newest fix",
            RepositionGuidance.KP_PER_S * 0.40, cmd.setpoint!!.north!!, 1e-6,
        )
    }

    // -------------------------------------------- the range ladder's height (landing07)

    @Test
    fun `a lying barometer cannot shrink the descent - the law flies the tag's range`() {
        // Landing07's landing B, replayed with the ladder
        // (`datasets/landing07/20260729-095413.001.jsonl`): baro "0.7 m" after a ~1.2 m
        // drift, tag size range 1.93 m at ~55 px — the size was right. The old design
        // believed the baro: descent rate kp·0.1 = 0.05 m/s into a false terminal hold ~1.9 m
        // up, and a commit genuinely half a metre off. The ladder flies the tag's 1.93 m —
        // the descent keeps coming at the full cap toward the real target — and the record
        // names the instrument and the divergence.
        val h = Harness().descending(fullAutoland = false)
        h.fixTagRange = 1.93
        h.fixRangeSource = RangeSource.SIZE
        h.place(northM = 0.0, relAlt = 0.7)
        h.tickSeeing()
        // Height 1.93 → clampedSpeed(1.33, cap 0.4) = 0.4. The lying baro would have said
        // kp·0.1 = 0.05 — an 8× difference, unmistakable in the send.
        assertEquals(-TagDescentGuidance.V_DESCENT_MAX_MS, lastSend(h).verticalThrottle, 1e-9)
        assertTrue(h.events.any {
            it.first == "height_source" && it.second!!.startsWith("size range=1.93")
        })
        assertTrue(h.events.any {
            it.first == "range_baro_divergence" &&
                it.second!!.contains("tag=1.93") && it.second!!.contains("baro=0.70")
        })
    }

    @Test
    fun `the commit carries its height's provenance - landing A's solve, and no divergence line`() {
        // Landing07's landing A, replayed: trusted solve 0.59 m against baro 0.8 m — the
        // known ~0.2 m near-ground baro over-read, ratio 1.36, NOT a divergence. The ladder
        // flies 0.59: already inside the terminal accept band, so the hold latches and the
        // terminal commit fires — recorded with `hsrc=solve`, so the post-flight reader
        // never has to infer which instrument "height=0.6" was.
        val h = Harness().descending()
        h.fixTagRange = 0.59
        h.fixRangeSource = RangeSource.SOLVE
        h.place(northM = 0.05, relAlt = 0.8)
        repeat(TagDescentGuidance.TERMINAL_TICKS + 2) { h.tickSeeing() }
        assertEquals(1, h.landing.starts)
        assertTrue(h.events.any {
            it.first == "landing_commit" &&
                it.second!!.startsWith("height=0.6") && it.second!!.contains("hsrc=solve")
        })
        assertTrue("the healthy pair is not a divergence",
            h.events.none { it.first == "range_baro_divergence" })
    }

    @Test
    fun `the floor detector reads the barometer - a ladder rung switch cannot reset its window`() {
        // The FC floor with the ladder switching rungs mid-stall: baro pinned at 1.4 m under
        // a commanded descent; halfway through the window a solve engages at 1.2 m, stepping
        // the ladder's height down 0.2 m — two fake quanta of "progress" if the detector
        // consumed the ladder, which would reset the window and defer the commit landing04
        // promoted. The detector reads the baro alone: the stall fires on the baro's schedule
        // and the commit with it, at the ladder's height.
        //
        // (The step is 0.2 m, not one quantum, on a lesson from this test's own first
        // survivor: a 1.4 → 1.3 step does not trip the reset threshold even when the seam IS
        // broken, because `1.4 − 0.1` is 1.2999999999999998 in IEEE 754 and 1.3 sits just
        // above it — a fixture at the exact threshold tests floating point, not the seam.
        // Landing B's real step was 1.2 m; 0.2 m is the smallest unambiguous stand-in.)
        val h = Harness().descending()
        h.place(northM = 0.05, relAlt = FLOOR)
        repeat(10) { h.tickSeeing() }
        assertEquals(0, h.landing.starts)
        h.fixTagRange = 1.2
        h.fixRangeSource = RangeSource.SOLVE
        repeat(14) { h.tickSeeing() }
        assertEquals(1, h.landing.starts)
        assertTrue(h.events.any {
            it.first == "landing_commit" && it.second!!.contains("hsrc=solve")
        })
    }

    @Test
    fun `height_source is recorded once per switch, with both instruments' numbers`() {
        val h = Harness().descending() // baro 5.0, no tag range yet
        assertEquals(
            listOf("baro height=5.00 tag=none"),
            h.events.filter { it.first == "height_source" }.map { it.second },
        )
        // The tag's range engaging is one line, not one per tick (4.0 vs 5.0 = 1.25 — the
        // switch is not a divergence, and must not be flagged as one).
        h.fixTagRange = 4.0
        h.fixRangeSource = RangeSource.SIZE
        repeat(3) { h.tickSeeing() }
        assertEquals(
            listOf("baro height=5.00 tag=none", "size range=4.00 baro=5.00"),
            h.events.filter { it.first == "height_source" }.map { it.second },
        )
        assertTrue(h.events.none { it.first == "range_baro_divergence" })
        // The fix going stale falls the ladder back to baro — recorded again, named again.
        repeat(6) { h.tickAlive() }
        val last = h.events.filter { it.first == "height_source" }.last().second!!
        assertTrue("stale tag range must fall back to baro, on the record", last.startsWith("baro height="))
    }

    @Test
    fun `divergence is one line per episode - a measurement that gates nothing`() {
        val h = Harness().descending(fullAutoland = false)
        h.fixTagRange = 1.93
        h.fixRangeSource = RangeSource.SIZE
        h.place(relAlt = 0.7)
        repeat(10) { h.tickSeeing() }
        assertEquals(1, h.events.count { it.first == "range_baro_divergence" })
        // Convergence closes the episode; a new divergence opens a new one.
        h.fixTagRange = 0.8
        repeat(3) { h.tickSeeing() }
        h.fixTagRange = 1.93
        repeat(3) { h.tickSeeing() }
        assertEquals(2, h.events.count { it.first == "range_baro_divergence" })
        // Never a gate: the descent flew on the tag's height throughout and is still armed.
        assertNotNull(h.engine.situation().descent)
    }

    @Test
    fun `double-commit is impossible - one engagement, one land(), whatever happens after`() {
        val h = Harness().descending().committed()
        // More stalls, fresh fixes, mode flaps, watchdog traffic: never a second ask.
        h.state = h.state.copy(flightMode = "CONFIRM_LANDING")
        repeat(50) { h.tickSeeing() }
        h.state = h.state.copy(flightMode = "GPS_ATTI")
        h.state = h.state.copy(flightMode = "CONFIRM_LANDING")
        repeat(50) { h.tickAlive() }
        assertEquals(1, h.landing.starts)
    }

    // ------------------------------------------------------------- commit refusals

    @Test
    fun `a refused commit ends the run to a hold, named - and is never retried`() {
        val h = Harness().descending()
        h.landing.startRefusal = "GPS_DISCONNECT"
        h.stalledAtFloor()
        assertEquals(1, h.landing.starts)
        assertNull("a commit that never left leaves no landing to wait on", h.engine.situation().descent)
        assertTrue(endedEvents(h).any { it == "commit failed: GPS_DISCONNECT" })
        assertTrue(h.texts().any { it.contains("GPS_DISCONNECT") })
        assertEquals("ours and holding", GuidedPhase.ENGAGED, h.engine.phase)
        repeat(30) { h.tickSeeing() }
        assertEquals("no retry - a fresh arm is the only way back", 1, h.landing.starts)
    }

    @Test
    fun `no landing seam - the commit is refused by name, not silently hovered through`() {
        val h = Harness(withLandingSeam = false).descending()
        h.stalledAtFloor()
        assertNull(h.engine.situation().descent)
        assertTrue(endedEvents(h).any { it == "commit failed: NO_LANDING_PATH" })
    }

    @Test
    fun `a commit DJI accepts but never enacts times out to a hold - unless DJI is landing`() {
        val h = Harness().descending().committed()
        // The mode never becomes a landing mode: past the bound, the engagement is handed
        // back to a hold with the reason named — the measured silent-accept failure.
        var remaining = (TagDescentGuidance.DJI_LAND_TIMEOUT_MS / 100 + 5).toInt()
        while (remaining-- > 0 && h.engine.situation().descent != null) h.tickAlive()
        assertNull(h.engine.situation().descent)
        assertTrue(endedEvents(h).any { it!!.startsWith("dji landing never engaged") })
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_DJI_TIMEOUT))

        // With the mode measurably landing, the bound does not apply: a slow landing is never
        // cut short — touchdown is the ending.
        val slow = Harness().descending().committed()
        slow.state = slow.state.copy(flightMode = "CONFIRM_LANDING")
        var ticks = (TagDescentGuidance.DJI_LAND_TIMEOUT_MS / 100 + 50).toInt()
        while (ticks-- > 0) slow.tickAlive()
        assertNotNull("DJI is landing - the timeout must not fire", slow.engine.situation().descent)
    }

    // ------------------------------------------------------------------- rule 1

    @Test
    fun `rule 1 mid-landing - an RC grab kills our engagement AND asks DJI to stop, recorded`() {
        val h = Harness().descending().committed()
        h.state = h.state.copy(flightMode = "CONFIRM_LANDING")
        repeat(5) { h.tickAlive() }
        h.port.onRc!!(RcSticks(0, -390, 0, 0)) // landing04's grab was lv -660 held ~1.5 s
        h.tickAlive(GuidedStickEngine.RC_ABORT_SUSTAIN_MS)
        assertEquals("total disengage", GuidedPhase.IDLE, h.engine.phase)
        assertNull(h.engine.situation().descent)
        assertTrue(endedEvents(h).contains("sticks"))
        assertEquals("the one action rule 1 gains", 1, h.landing.stops)
        assertTrue(h.events.any { it.first == "landing_stop" && it.second == "asked (sticks)" })
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_STOP_SENT))
        // Nothing resumes, nothing re-asks.
        repeat(5) { h.tickAlive() }
        assertEquals(1, h.landing.stops)
        assertEquals(1, h.landing.starts)
    }

    @Test
    fun `rule 1 on the GCS channel does the same - manual sticks are manual sticks`() {
        val h = Harness().descending().committed()
        h.frame() // neutral, recently at rest
        h.now += 40
        h.frame(x = 500)
        assertNull(h.engine.situation().descent)
        assertEquals(1, h.landing.stops)
        assertTrue(h.events.any { it.first == "landing_stop" && it.second == "asked (gcs-sticks)" })
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "sticks" })
    }

    @Test
    fun `a refused stop is recorded verbatim - the exchange is the measurement`() {
        val h = Harness().descending().committed()
        h.landing.stopRefusal = "NO_PRODUCT"
        h.port.onRc!!(RcSticks(200, 0, 0, 0))
        h.tickAlive(GuidedStickEngine.RC_ABORT_SUSTAIN_MS)
        assertTrue(h.events.any {
            it.first == "landing_stop" && it.second == "not asked: NO_PRODUCT (sticks)"
        })
    }

    @Test
    fun `rule 1 after a DJI-confirmed landing still stops, and the record carries the confirm`() {
        val h = Harness().descending().committed()
        h.engine.noteAutolandConfirmed()
        assertTrue(h.events.any { it.first == "tag_descent_phase" && it.second == "landing dji-confirmed" })
        h.port.onRc!!(RcSticks(200, 0, 0, 0))
        h.tickAlive(GuidedStickEngine.RC_ABORT_SUSTAIN_MS)
        assertTrue(endedEvents(h).any { it == "sticks dji-confirmed" })
        assertEquals(1, h.landing.stops)
    }

    @Test
    fun `disarm and pause during a committed landing withdraw DJI's landing too`() {
        val disarmed = Harness().descending().committed()
        assertEquals(Verdict.ACCEPTED, disarmed.engine.disarmTagDescent())
        assertEquals(1, disarmed.landing.stops)
        assertTrue(disarmed.events.any { it.first == "landing_stop" && it.second == "asked (disarmed)" })

        val paused = Harness().descending().committed()
        assertEquals(Verdict.ACCEPTED, paused.engine.pause())
        assertEquals(1, paused.landing.stops)
        assertTrue(paused.events.any { it.first == "landing_stop" && it.second == "asked (paused)" })
    }

    @Test
    fun `a non-stick abort sends no stop - DJI may complete the landing, and only rule 1 withdraws`() {
        val h = Harness().descending().committed()
        h.interlock = false
        h.tickAlive()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(endedEvents(h).contains("interlock"))
        assertEquals("no KeyStopAutoLanding without a hand on the sticks", 0, h.landing.stops)
    }

    @Test
    fun `any non-landing mode is still a seizure during a committed landing`() {
        val h = Harness().descending().committed()
        repeat(16) { h.tickAlive() } // past the seize grace window
        h.state = h.state.copy(flightMode = "GO_HOME")
        h.tickAlive()
        assertNull("a forced GO_HOME must still abort", h.engine.situation().descent)
        assertTrue(endedEvents(h).contains("authority"))
    }

    @Test
    fun `touchdown is motors-off, never the altitude`() {
        val h = Harness().descending().committed()
        h.state = h.state.copy(flightMode = "CONFIRM_LANDING")
        // The altitude reads below zero (measured at rest: −0.1 m) with motors still on: the
        // aircraft is NOT down.
        h.place(northM = 0.0, relAlt = -0.1)
        repeat(5) { h.tickAlive() }
        assertNotNull("altitude may never declare touchdown", h.engine.situation().descent)
        h.state = h.state.copy(motorsOn = false)
        h.tickAlive()
        assertNull(h.engine.situation().descent)
        assertTrue(endedEvents(h).contains("touchdown"))
    }

    // ------------------------------------------------------------ gimbal watchdog

    @Test
    fun `the watchdog re-commands nadir when DJI recenters during its landing - recorded`() {
        val h = Harness().descending().committed()
        h.reportedGimbalPitch = -33.4 // the landing04 slew, mid-flight
        h.tickAlive()
        assertEquals(listOf(TagDescentGuidance.NADIR_PITCH_DEG), h.gimbal.aimed)
        assertTrue(h.events.any {
            it.first == "gimbal_watchdog" && it.second!!.startsWith("re-command nadir attempt=1")
        })
        // And it never touched the neutral stream: the same tick's send is still zero.
        assertEquals(0.0, lastSend(h).verticalThrottle, 1e-9)
    }

    @Test
    fun `the watchdog is rate-limited and bounded - past the bound it logs once and stays quiet`() {
        val h = Harness().descending().committed()
        h.state = h.state.copy(flightMode = "CONFIRM_LANDING")
        h.reportedGimbalPitch = 0.0
        repeat(9) { h.tickAlive() }
        assertEquals(1, h.gimbal.aimed.size)
        var guard = 0
        while (h.gimbal.aimed.size < TagDescentGuidance.GIMBAL_WATCHDOG_MAX && guard++ < 300) {
            h.tickAlive(500)
        }
        assertEquals(TagDescentGuidance.GIMBAL_WATCHDOG_MAX, h.gimbal.aimed.size)
        repeat(30) { h.tickAlive(500) }
        assertEquals(TagDescentGuidance.GIMBAL_WATCHDOG_MAX, h.gimbal.aimed.size)
        assertEquals(
            1,
            h.events.count { it.first == "gimbal_watchdog" && it.second!!.startsWith("gave up") },
        )
        assertNotNull(h.engine.situation().descent)
    }

    @Test
    fun `the watchdog does not exist outside a committed landing`() {
        val h = Harness().descending() // TRACKING
        h.reportedGimbalPitch = 0.0
        repeat(10) { h.tickSeeing() }
        assertTrue("no nadir re-command outside DJI_LANDING", h.gimbal.aimed.isEmpty())
        assertTrue(h.events.none { it.first == "gimbal_watchdog" })
        // Stage B's rule for a *commanded* camera move is untouched: the descent cancels.
        h.cameraPitch = 0.0
        h.tickSeeing()
        assertNull(h.engine.situation().descent)
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_GIMBAL))
    }

    @Test
    fun `a nadir-holding gimbal never trips the watchdog - jitter is not a slew`() {
        val h = Harness().descending().committed()
        h.reportedGimbalPitch = -89.8 // the measured held-sample jitter
        repeat(20) { h.tickAlive() }
        assertTrue(h.gimbal.aimed.isEmpty())
    }

    // ------------------------------------------------- the confirm gate, cross-layer

    /** A minimal recorded [ActionPort] — what the land/stop/confirm paths touch. */
    private class FakeActionPort : ActionPort {
        var reason: String? = null
        val performed = mutableListOf<String>()
        val landStarts = mutableListOf<Pair<() -> Unit, (String) -> Unit>>()
        val confirms = mutableListOf<Pair<() -> Unit, (String) -> Unit>>()
        var confirmationListener: ((Boolean?) -> Unit)? = null
        var landingModeListener: ((Boolean?) -> Unit)? = null

        override fun unavailableReason(): String? = reason
        override fun canStartGoHome() = true
        override fun canStartAutoLanding() = true
        override fun canStopAutoLanding() = true
        override fun canStartTakeoff() = true
        override fun startGoHome(onFailure: (String) -> Unit) = Unit
        override fun startTakeoff(onFailure: (String) -> Unit) = Unit

        override fun startAutoLanding(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            performed.add("startAutoLanding")
            landStarts.add(onSuccess to onFailure)
        }

        override fun stopAutoLanding(onFailure: (String) -> Unit) {
            performed.add("stopAutoLanding")
        }

        override fun confirmLanding(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            performed.add("confirmLanding")
            confirms.add(onSuccess to onFailure)
        }

        override fun listenIsLandingConfirmationNeeded(onDelivery: (Boolean?) -> Unit) {
            confirmationListener = onDelivery
        }

        override fun listenIsInLandingMode(onDelivery: (Boolean?) -> Unit) {
            landingModeListener = onDelivery
        }

        override fun cancelListens() = Unit
    }

    /**
     * [MsdkFlightActions] wired to the live engine across the exact seams `Bridge` wires. The
     * harness's commit seam stays the recording fake — each test that wants the commit's
     * `land()` to reach this class forwards it explicitly, playing `Bridge`'s one line.
     */
    private class ConfirmRig(val h: Harness) {
        val actionPort = FakeActionPort()
        val recorded = mutableListOf<Triple<String, String, Boolean>>()
        var announced = 0

        val actions = MsdkFlightActions(
            port = actionPort,
            reportAsyncDjiError = {},
            announceLandingConfirmed = { announced++ },
            interlockEnabled = { h.interlock },
            autolandClearance = { h.engine.autolandClearance() },
            onAutolandConfirmed = { h.engine.noteAutolandConfirmed() },
            recordEvent = { code, message, warn -> recorded += Triple(code, message, warn) },
        )

        init {
            actions.armAutolandListening()
        }
    }

    @Test
    fun `the full chain - floor commit through land(), DJI's own claim, one confirm at the stall`() {
        val h = Harness().descending()
        val rig = ConfirmRig(h)
        // Bridge's seam, played verbatim: the engine's commit start() runs the real land().
        h.landing.onStart = { rig.actions.land() }

        h.stalledAtFloor()
        assertEquals(1, h.landing.starts)
        assertEquals(listOf("startAutoLanding"), rig.actionPort.performed)
        rig.actionPort.landStarts.single().first() // DJI accepts — the landing is ours to confirm

        // DJI stalls at its confirmation height and asks: the operator scope answers — this
        // landing was started by this class, from the operator's own armed autoland.
        rig.actionPort.confirmationListener!!(true)
        assertEquals(listOf("startAutoLanding", "confirmLanding"), rig.actionPort.performed)
        rig.actionPort.confirms.single().first()
        assertEquals(1, rig.announced)

        // Re-delivery: the episode's one confirm is spent, whichever scope spent it.
        rig.actionPort.confirmationListener!!(true)
        assertEquals(1, rig.actionPort.performed.count { it == "confirmLanding" })
    }

    @Test
    fun `the guided scope answers when no claim exists - aligned confirms, and is noted back`() {
        val h = Harness().descending().committed()
        val rig = ConfirmRig(h)
        // No land() reached rig.actions (the harness seam recorded it), so the operator-scope
        // claim never existed and the guided scope is the deciding one, reading the engine.
        rig.actionPort.confirmationListener!!(true)
        assertEquals(listOf("confirmLanding"), rig.actionPort.performed)
        assertTrue(rig.recorded.any { it.first == "landing_confirm" && it.second == "sent" })
        rig.actionPort.confirms.single().first()
        assertEquals(1, rig.announced)
        assertTrue(h.events.any { it.first == "tag_descent_phase" && it.second == "landing dji-confirmed" })
    }

    @Test
    fun `a blind landing refuses the confirm - recorded with the reason`() {
        val h = Harness().descending().committed()
        repeat(30) { h.tickAlive() } // the tag is gone; the landing is blind
        assertTrue(h.engine.situation().descent!!.blind)
        val rig = ConfirmRig(h)
        rig.actionPort.confirmationListener!!(true)
        assertTrue(rig.actionPort.performed.isEmpty())
        assertTrue(rig.recorded.any {
            it.first == "landing_confirm" && it.second.startsWith("refused: fix")
        })
    }

    @Test
    fun `the confirm sits behind the arm switch`() {
        val h = Harness().descending().committed()
        val rig = ConfirmRig(h)
        h.interlock = false
        rig.actionPort.confirmationListener!!(true)
        assertTrue(rig.actionPort.performed.isEmpty())
        assertTrue(rig.recorded.any {
            it.first == "landing_confirm" && it.second == "refused: interlock off"
        })
    }

    @Test
    fun `no landing, no opinion - a foreign confirmation question is left entirely alone`() {
        val h = Harness()
        val rig = ConfirmRig(h)
        rig.actionPort.confirmationListener!!(true)
        assertTrue(rig.actionPort.performed.isEmpty())
        assertTrue("no refusal spam for a landing that is not ours",
            rig.recorded.none { it.first == "landing_confirm" })
        assertTrue(rig.recorded.any { it.first == "landing_confirm_needed" })
    }

    @Test
    fun `rule 1 between the question and the answer - a dead engagement answers no`() {
        val h = Harness().descending().committed()
        val rig = ConfirmRig(h)
        h.port.onRc!!(RcSticks(200, 0, 0, 0))
        h.tickAlive(GuidedStickEngine.RC_ABORT_SUSTAIN_MS)
        assertNull(h.engine.situation().descent)
        rig.actionPort.confirmationListener!!(true)
        assertTrue("a clearance must die with its engagement", rig.actionPort.performed.isEmpty())
    }

    @Test
    fun `the clearance is null before the commit and carries the landing's facts after it`() {
        val h = Harness().descending()
        assertNull("tracking is not a landing", h.engine.autolandClearance())
        h.committed()
        val clearance = h.engine.autolandClearance()
        assertNotNull(clearance)
        assertTrue(clearance!!.fixWasInCone)
        assertTrue(clearance.fixAgeMs <= MsdkFlightActions.CONFIRM_FRESH_MS)
        repeat(30) { h.tickAlive() }
        assertTrue(h.engine.autolandClearance()!!.fixAgeMs > MsdkFlightActions.CONFIRM_FRESH_MS)
    }

    @Test
    fun `a plain descent never produces a clearance - the option is part of the identity`() {
        val h = Harness().descending(fullAutoland = false)
        h.stalledAtFloor(ticks = 40)
        assertNull(h.engine.autolandClearance())
    }
}
