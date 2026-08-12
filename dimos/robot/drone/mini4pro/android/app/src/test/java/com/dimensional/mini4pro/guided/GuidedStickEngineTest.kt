package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.mavlink.StatusTextSink
import com.dimensional.mini4pro.record.CommandSource
import com.dimensional.mini4pro.record.Setpoint
import com.dimensional.mini4pro.record.StickAxes
import com.dimensional.mini4pro.record.StickModes
import com.dimensional.mini4pro.record.StickRange
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import io.dronefleet.mavlink.common.ManualControl
import io.dronefleet.mavlink.common.Statustext
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The Stage A engagement state machine, driven through a fake [VirtualStickPort] and a
 * hand-cranked clock — every decision between a `MANUAL_CONTROL` frame and a DJI virtual-stick
 * param, without an aircraft.
 *
 * The suite is written to fail loudly for the mistakes that would hurt someone:
 *
 *  - engagement latching on "we asked" instead of on DJI's confirmed state (landmine 5 — the
 *    measured swallowed-callback case)
 *  - a centre-zero throttle stream engaging and its neutral being read as full descend
 *    (landmine 2)
 *  - a climb reaching DJI with the down-positive sign (landmine 1)
 *  - any of the three Q3 abort gestures not firing, or the RC-null component-gone signal
 *    (landmine 6) being read as centred sticks
 *  - a silent stream being trusted (the §3.1 watchdog), or the wrong Q4 terminal sequence
 *
 * Mutation-checked 2026-07-26 — each breakage made deliberately, failing tests counted across
 * both guided suites, code reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | ENGAGED on request: the engage trigger sets `phaseLocked = ENGAGED` directly | 37 |
 *  | ENGAGED on enable's onSuccess instead of on the state report | 2 |
 *  | conjunct A weakened: `authorityOk` ignores `authority` (enabled+advanced only) | 2 |
 *  | conjunct A weakened: `advanced` not required | 1 |
 *  | engage-confirm timeout removed | 2 |
 *  | interlock check removed from the engage gate | 1 |
 *  | neutral-recency gate removed (centre-zero stream engages) | 4 |
 *  | RC-feed gate removed from engagement | 1 |
 *  | input-stale ramp removed (stale frame commanded at full value forever) | 1 |
 *  | link-lost never declared (`LINK_LOST_MS` check short-circuited) | 5 |
 *  | released/link-lost distinction inverted | 4 |
 *  | resume-during-wind-down removed | 1 |
 *  | RC-stick abort removed | 1 |
 *  | RC-stick debounce removed (single blip aborts) | 1 |
 *  | RC null read as centred (component-gone ignored, both paths) | 1 |
 *  | authority-change-reason abort ignores everything | 1 |
 *  | `MSDK_REQUEST` also aborts (own engagement self-destructs) | 1 |
 *  | idle disengage removed | 1 |
 *  | manoeuvre timeout removed | 1 |
 *  | ceiling gate removed | 3 |
 *  | ceiling gate also blocks descent | 1 |
 *  | abort idempotency check dropped | 1 |
 *  | zero setpoint on abort dropped | 1 |
 *  | disable on abort dropped | 5 |
 *  | announce dedup removed | 1 |
 *
 * Added after the first hardware engagement (2026-07-26, mode-seizure abort — see
 * `docs/measurements/2026-07-26-stage-a-first-engagement.md`), same protocol:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | mode-seizure check made inert (`modeSeizedLocked` returns null always) | 3 |
 *  | seizure grace window dropped (any non-JOYSTICK mode aborts instantly) | 1 |
 *  | null mode read as seized | 8 |
 *  | seizure check dropped from the RELEASING tick only | 1 |
 *
 * Added with Stage B (2026-07-26 night — the Stage B rows themselves are in
 * `GuidedRepositionTest` and `RepositionGuidanceTest`), same protocol:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | refusal-log throttle removed (25 Hz flood returns) | 2 |
 *
 * The 37 on the first row is what it looks like when a state machine's core invariant dies:
 * an ENGAGED-without-authority phase sends setpoints with no confirmed authority, and most of
 * the suite notices. The single-test rows are deliberate — each of those properties has a
 * named test built to be the one that catches exactly that regression.
 *
 * ## `ControlOrigin`, 2026-07-27 — and three mutants that are alive on purpose
 *
 * `gcsSeenAtMs` became a map keyed by [ControlOrigin], read for the controller that established
 * the engagement (`docs/zenoh-dimos-transport.md` §4.3). Same protocol, counted across both
 * guided suites:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `onInbound` stops stamping liveness | 5 |
 *  | `reposition` stops stamping its own controller's liveness | 2 |
 *  | the reposition tick's link watchdog removed entirely | 2 |
 *  | the watchdog reads a hardcoded `MAVLINK` key rather than the engagement origin | **0** |
 *  | the watchdog reads the newest entry across *all* origins ("any transport alive") | **0** |
 *  | `engagementOrigin` never recorded (left null forever) | **0** |
 *
 * **The three zeros were honest and were not holes that could be closed** — with one origin the
 * map had one entry, so "was the commanding controller alive?", "was the MAVLink controller
 * alive?" and "was *anyone* alive?" were the same question and no test could separate them.
 * They were recorded rather than papered over because the mutants become killable the moment a
 * second `ControlOrigin` exists, and the tripwire test said so.
 *
 * ## `ControlOrigin.PHONE`, 2026-07-29 — landing08, and the three zeros closed
 *
 * The second origin arrived (`datasets/landing08/20260729-112216.001.jsonl`: the first phone-only
 * flight — every engagement mislabelled MAVLINK, the takeoff climb dead `link-lost` at t=33.93,
 * six descent arms refused `LINK_DOWN`). PHONE's liveness is identity, not traffic
 * (`GuidedStickEngine.controllerSeenAtLocked` is the one owner), which made the three zeros
 * observable at last. Same protocol — one mutation at a time, applied to the shipped source,
 * whole suite per mutant (`test-results` deleted first), confirmed red, reverted — counts are
 * failing tests across the whole suite, **measured, not estimated**:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the liveness owner treats PHONE as MAVLINK (`controllerSeenAtLocked` reads the map for PHONE) | 14 |
 *  | the phone takeoff door mislabels its climb MAVLINK (`takeoffFromPhone`'s dispatch) | 1 |
 *  | the phone descent door mislabels its arm MAVLINK (`armTagDescentFromPhone`) | 4 |
 *  | `TakeoffClimb.arm` drops the origin (pending intention stores MAVLINK regardless) | 2 |
 *  | NO_RC_FEED gate skipped for PHONE arms (`descentGateLocked`) | 1 |
 *  | GCS-stick rule 1 exempts PHONE engagements (`onFrame`'s takeover block) | 2 |
 *  | the MAVLINK watchdog weakened (`controllerSeenAtLocked` answers `now` for MAVLINK too) | 13 |
 *  | *(zero #1 closed)* the watchdog reads a hardcoded `MAVLINK` key rather than the engagement origin | 4 |
 *  | *(zero #2 closed)* the watchdog reads the newest entry across all origins ("anyone alive") | 4 |
 *  | *(zero #3 closed)* `engagementOrigin` never recorded (left null forever) | 4 |
 *
 * The two big rows are the two safety clauses measured. The 13 on the weakened-watchdog row is
 * the old guard holding: link-lost tests across five suites notice when MAVLink silence stops
 * mattering — the "MAVLINK keeps the heartbeat byte-for-byte" clause. The 14 on the first row is
 * its new twin: treating PHONE as MAVLINK re-creates landing08 (the phone climb and descent die,
 * both pinned regressions fail) *and* dismantles shadow mode's phone-origin gate, which the
 * shadow suite notices wholesale. The three closed zeros all score an identical 4, through the
 * same four PHONE-side tests, because each collapses `commandingControllerSeenAtLocked` into a
 * map read that PHONE never writes — the landing08 regression by three different roads. The
 * 1-count rows are the deliberate single-property pins (the dispatcher's door label, the
 * RC-feed gate), one test built for exactly that regression, the table's established idiom.
 * What remains untestable — transport-vs-transport cross-liveness — is the successor tripwire's
 * subject (`one transport today - a second one must bring the cross-transport tests with it`).
 */
class GuidedStickEngineTest {

    // ------------------------------------------------------------------ fixture

    private class FakeVirtualStickPort : VirtualStickPort {
        var unavailable: String? = null
        var enableCalls = 0
        var disableCalls = 0
        val advancedSets = mutableListOf<Boolean>()
        var enableOnSuccess: (() -> Unit)? = null
        var enableOnFailure: ((String) -> Unit)? = null
        data class Sent(val pitch: Double, val roll: Double, val yaw: Double, val verticalThrottle: Double)
        val sent = mutableListOf<Sent>()
        var sendError: String? = null
        var onState: ((VirtualStickSnapshot) -> Unit)? = null
        var onReason: ((String) -> Unit)? = null
        var onRc: ((RcSticks) -> Unit)? = null
        var cancelled = 0

        val modes = StickModes(
            rollPitch = "VELOCITY", yaw = "ANGULAR_VELOCITY", vertical = "VELOCITY",
            coordinateSystem = "GROUND", advanced = true,
        )

        override fun unavailableReason(): String? = unavailable

        override fun enable(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            enableCalls++
            enableOnSuccess = onSuccess
            enableOnFailure = onFailure
        }

        override fun disable(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            disableCalls++
        }

        override fun setAdvancedMode(enabled: Boolean) {
            advancedSets += enabled
        }

        override fun sendAdvancedParam(
            pitch: Double, roll: Double, yaw: Double, verticalThrottle: Double,
        ): SendReport {
            sent += Sent(pitch, roll, yaw, verticalThrottle)
            return SendReport(modes, sendError)
        }

        override fun listenState(
            onState: (VirtualStickSnapshot) -> Unit,
            onAuthorityReason: (String) -> Unit,
        ) {
            this.onState = onState
            this.onReason = onAuthorityReason
        }

        override fun listenRcSticks(onDelivery: (RcSticks) -> Unit) {
            this.onRc = onDelivery
        }

        override fun cancelListens() {
            cancelled++
        }
    }

    private class RecordedCmd(
        val setpoint: Setpoint?, val axes: StickAxes, val modes: StickModes,
        val source: CommandSource?, val range: StickRange?, val accepted: Boolean?, val error: String?,
    )

    private class Harness(policy: LinkLossPolicy = LinkLossPolicy.SHIPPED) {
        var now = 1_000L
        var interlock = true
        var state = AircraftState(
            relativeAltitude = 5.0,
            ages = SampleAges.of(Signal.ALTITUDE to 0L),
        )
        val port = FakeVirtualStickPort()
        val wire = mutableListOf<Any>()
        val cmds = mutableListOf<RecordedCmd>()
        val events = mutableListOf<Pair<String, String?>>()
        val logs = mutableListOf<String>()

        val engine = GuidedStickEngine(
            port = port,
            interlockEnabled = { interlock },
            aircraftState = { state },
            announcer = Announcer(StatusTextSink { wire += it }),
            record = object : GuidedRecord {
                override fun stickCmd(
                    setpoint: Setpoint?, axes: StickAxes, modes: StickModes,
                    source: CommandSource?, range: StickRange?, accepted: Boolean?, error: String?,
                ) {
                    cmds += RecordedCmd(setpoint, axes, modes, source, range, accepted, error)
                }

                override fun event(code: String, message: String?, warn: Boolean) {
                    events += code to message
                }
            },
            policy = policy,
            log = { logs += it },
            nowMs = { now },
        )

        init {
            engine.attach()
            // A healthy, centred RC feed — the state every bench session starts in.
            port.onRc!!(RcSticks(0, 0, 0, 0))
        }

        fun texts(): List<String> = wire.filterIsInstance<Statustext>().map { it.text() }

        fun frame(
            x: Int = 0, y: Int = 0, z: Int = 500, r: Int = 0, seq: Int? = null, target: Int = 1,
            origin: ControlOrigin = ControlOrigin.MAVLINK,
        ) {
            engine.onInbound(
                ManualControl.builder().target(target).x(x).y(y).z(z).r(r).buttons(0).build(),
                seq,
                origin,
            )
        }

        fun tick(advanceMs: Long = 0) {
            now += advanceMs
            engine.tick(now)
        }

        /** Neutral frame → deliberate deflection → DJI accepts → DJI's state confirms → ENGAGED. */
        fun engage(x: Int = 500, origin: ControlOrigin = ControlOrigin.MAVLINK) {
            frame(origin = origin) // neutral, observed at rest
            now += 40
            frame(x = x, origin = origin)
            check(engine.phase == GuidedPhase.ENGAGING) { "expected ENGAGING, got ${engine.phase}" }
            port.enableOnSuccess!!.invoke()
            port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "MSDK"))
            tick(40)
            check(engine.phase == GuidedPhase.ENGAGED) { "expected ENGAGED, got ${engine.phase}" }
        }
    }

    // ------------------------------------------------------------ the engage gate

    @Test
    fun `a neutral stream never engages - receipt is not intent`() {
        val h = Harness()
        repeat(50) {
            h.frame()
            h.tick(40)
        }
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertEquals(0, h.port.sent.size)
    }

    @Test
    fun `interlock off - deflection is announced and nothing reaches DJI`() {
        val h = Harness()
        h.interlock = false
        h.frame()
        h.now += 40
        h.frame(x = 800)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertTrue(h.texts().contains(GuidedStatusTexts.IGNORED_INTERLOCK))
    }

    @Test
    fun `no product - deflection is announced with the port's reason`() {
        val h = Harness()
        h.port.unavailable = "NO_PRODUCT"
        h.frame()
        h.now += 40
        h.frame(x = 800)
        assertEquals(0, h.port.enableCalls)
        assertTrue(h.texts().contains("Sticks ignored: NO_PRODUCT"))
    }

    @Test
    fun `a stream never seen at rest cannot engage`() {
        val h = Harness()
        // First frame ever is already deflected — no neutral observation exists.
        h.frame(x = 800)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertTrue(h.texts().contains(GuidedStatusTexts.CENTER_FIRST))
    }

    @Test
    fun `centre-zero throttle stream can never engage - its rest is not our neutral`() {
        // Landmine 2's guard. QGC's opt-in centre-zero regime idles at z = 0, which under the
        // one convention this bridge interprets would be a full-scale descent. Such a stream
        // must never produce an engagement, only the centre-sticks sentence.
        val h = Harness()
        repeat(100) {
            h.frame(z = 0) // resting centre-zero stream: x/y/r centred, z at ITS neutral
            h.tick(40)
        }
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertEquals(0, h.port.sent.size)
        assertTrue(h.texts().contains(GuidedStatusTexts.CENTER_FIRST))
    }

    @Test
    fun `dead RC stick feed refuses engagement - the abort gesture would be blind`() {
        val h = Harness()
        h.port.onRc!!(RcSticks(0, null, 0, 0))
        h.frame()
        h.now += 40
        h.frame(x = 800)
        assertEquals(0, h.port.enableCalls)
        assertTrue(h.texts().contains(GuidedStatusTexts.NO_RC_FEED))
    }

    @Test
    fun `a frame for another system is ignored entirely`() {
        val h = Harness()
        h.frame()
        h.now += 40
        h.frame(x = 800, target = 2)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertTrue(h.texts().isEmpty())
    }

    @Test
    fun `unreadable axes are refused with a sentence, never guessed at`() {
        val h = Harness()
        h.frame(z = -400)
        assertTrue(h.texts().contains(GuidedStatusTexts.BAD_AXES))
        h.frame(x = 32767)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
    }

    // -------------------------------------------------- engagement confirmation

    @Test
    fun `deliberate deflection asks DJI once and claims nothing`() {
        val h = Harness()
        h.frame()
        h.now += 40
        h.frame(x = 500)
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        assertEquals(1, h.port.enableCalls)
        // Still ENGAGING and still silent toward the aircraft: no setpoint may flow before
        // DJI confirms authority.
        h.tick(100)
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        assertEquals(0, h.port.sent.size)
        assertTrue(h.texts().none { it == GuidedStatusTexts.ENGAGED })
    }

    @Test
    fun `ENGAGED comes only from DJI's state report - not from the enable callback`() {
        val h = Harness()
        h.frame()
        h.now += 40
        h.frame(x = 500)
        h.port.enableOnSuccess!!.invoke() // DJI accepted the call…
        h.tick(100)
        // …and that is still not authority. The callback is not the state.
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        h.port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "MSDK"))
        h.tick(100)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.texts().contains(GuidedStatusTexts.ENGAGED))
        assertTrue(h.port.advancedSets.contains(true))
        // The engage event names the armed link-loss policy — Q4's logging requirement.
        assertTrue(h.events.any { it.first == "guided_engaged" && it.second == "policy=DecelerateThenHandback" })
    }

    @Test
    fun `a swallowed enable never latches - withdrawn after the confirm window`() {
        // Landmine 5: DJI can accept a call and fire neither callback (measured). ENGAGING
        // must die on the deadline, clean up with a disable, and say so.
        val h = Harness()
        h.frame()
        h.now += 40
        h.frame(x = 500)
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        h.tick(GuidedStickEngine.ENGAGE_CONFIRM_MS + 100)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick refused: NO_CONFIRM"))
        assertEquals(0, h.port.sent.size)
    }

    @Test
    fun `enabled but authority still RC never engages - conjunct A is all three facts`() {
        val h = Harness()
        h.frame()
        h.now += 40
        h.frame(x = 500)
        h.port.enableOnSuccess!!.invoke()
        h.port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "RC"))
        h.tick(100)
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        h.tick(GuidedStickEngine.ENGAGE_CONFIRM_MS)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
    }

    @Test
    fun `enabled without advanced mode never engages`() {
        val h = Harness()
        h.frame()
        h.now += 40
        h.frame(x = 500)
        h.port.onState!!(VirtualStickSnapshot(enabled = true, advanced = false, authority = "MSDK"))
        h.tick(100)
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
    }

    @Test
    fun `DJI refusing the enable is announced verbatim and the attempt ends`() {
        val h = Harness()
        h.frame()
        h.now += 40
        h.frame(x = 500)
        h.port.enableOnFailure!!.invoke("FC_AUTH_STATE")
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick refused: FC_AUTH_STATE"))
    }

    @Test
    fun `a failed engage is not retried inside the retry window`() {
        val h = Harness()
        h.frame()
        h.now += 40
        h.frame(x = 500)
        h.port.enableOnFailure!!.invoke("FC_AUTH_STATE")
        assertEquals(1, h.port.enableCalls)
        // Held deflection keeps streaming at 25 Hz — DJI must not be hammered.
        repeat(20) {
            h.now += 40
            h.frame(x = 500)
        }
        assertEquals(1, h.port.enableCalls)
        // Past the window, with the stream seen at rest again, a fresh deflection re-asks.
        h.now += GuidedStickEngine.ENGAGE_RETRY_MS
        h.frame()
        h.now += 40
        h.frame(x = 500)
        assertEquals(2, h.port.enableCalls)
    }

    // ------------------------------------------------------------- passthrough

    @Test
    fun `full forward stick reaches DJI as exactly the 3 m per s envelope on roll`() {
        // North rides DJI `roll` under GROUND — measured 2026-07-26 (StickMappingTest's
        // mapping test carries the full citation).
        val h = Harness()
        h.engage(x = 1000)
        h.tick(100)
        val sent = h.port.sent.last()
        assertEquals(3.0, sent.roll, 1e-9)
        assertEquals(0.0, sent.pitch, 1e-9)
        assertEquals(0.0, sent.yaw, 1e-9)
        assertEquals(0.0, sent.verticalThrottle, 1e-9)
    }

    @Test
    fun `full up-throttle reaches DJI as a POSITIVE verticalThrottle end to end`() {
        // Landmine 1 through the whole engine: z=1000 is a climb; DJI's commanded vertical is
        // up-positive; one sign error here flies a climb into the ground.
        val h = Harness()
        h.engage()
        h.now += 40
        h.frame(z = 1000)
        h.tick(40)
        assertEquals(+1.5, h.port.sent.last().verticalThrottle, 1e-9)
        // And the recorded setpoint says what we meant in NED: down negative.
        assertEquals(-1.5, h.cmds.last().setpoint!!.down!!, 1e-9)
    }

    @Test
    fun `neutral frames keep the 10 Hz stream flowing at zero`() {
        // DJI wants 5–25 Hz and documents nothing about silence; while engaged the stream
        // never stops, and zero commanded velocity is DJI's own position hold.
        val h = Harness()
        h.engage()
        h.now += 40
        h.frame() // neutral
        val before = h.port.sent.size
        repeat(5) {
            h.frame()
            h.tick(100)
        }
        assertEquals(before + 5, h.port.sent.size)
        assertTrue(h.port.sent.takeLast(5).all { it.pitch == 0.0 && it.verticalThrottle == 0.0 })
    }

    @Test
    fun `every send is recorded with the modes read off the sent object and the source frame`() {
        val h = Harness()
        h.engage()
        h.now += 40
        h.frame(x = 500, seq = 77)
        h.tick(40)
        val cmd = h.cmds.last()
        assertEquals("VELOCITY", cmd.modes.rollPitch)
        assertEquals("ANGULAR_VELOCITY", cmd.modes.yaw)
        assertEquals("VELOCITY", cmd.modes.vertical)
        assertEquals("GROUND", cmd.modes.coordinateSystem)
        assertEquals(true, cmd.modes.advanced)
        assertEquals("MANUAL_CONTROL", cmd.source!!.messageName)
        assertEquals(77, cmd.source!!.sequence)
        assertEquals(true, cmd.accepted)
        assertNull(cmd.error)
        assertEquals("NED_VELOCITY", cmd.setpoint!!.frame)
    }

    @Test
    fun `the envelope range is recorded once per engagement, not on every command`() {
        val h = Harness()
        h.engage()
        h.tick(100)
        h.tick(100)
        val ranges = h.cmds.map { it.range }
        assertNotNull(ranges.first())
        assertEquals(3.0, ranges.first()!!.rollPitchMax!!, 1e-9)
        assertEquals(1.5, ranges.first()!!.verticalMax!!, 1e-9)
        assertTrue(ranges.drop(1).all { it == null })
    }

    // ---------------------------------------------------------------- watchdog

    @Test
    fun `stale input ramps to zero and stays engaged - the measured 1 second gap is survivable`() {
        val h = Harness()
        h.engage(x = 1000)
        h.tick(100) // fresh: 3.0
        assertEquals(3.0, h.port.sent.last().roll, 1e-9)
        // No frames for 750 ms: 250 ms into the 500 ms ramp → half value.
        h.tick(610)
        val ramped = h.port.sent.last()
        assertEquals(1.5, ramped.roll, 1e-9)
        // The ramp is our withdrawal, not the operator's command: source must be null.
        assertNull(h.cmds.last().source)
        // Past ramp end: zero, still engaged.
        h.tick(300)
        assertEquals(0.0, h.port.sent.last().roll, 1e-9)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `input resuming after a gap restores full passthrough`() {
        val h = Harness()
        h.engage(x = 1000)
        h.tick(900) // deep in the ramp
        h.frame(x = 1000)
        h.tick(10)
        assertEquals(3.0, h.port.sent.last().roll, 1e-9)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    // ------------------------------------------------- stream end: the Q4 shapes

    @Test
    fun `sticks stop but link alive - standard wind-down, reason released`() {
        val h = Harness()
        h.engage()
        // The GCS keeps talking (heartbeats), only MANUAL_CONTROL stops.
        var t = 0L
        while (t < GuidedEnvelope.LINK_LOST_MS + 200) {
            h.engine.onInbound("heartbeat")
            h.tick(100)
            t += 100
        }
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick stopping: released"))
        // Run the plan out: ramp 500 + hold 1000, then release.
        repeat(20) {
            h.engine.onInbound("heartbeat")
            h.tick(100)
        }
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: released"))
        assertTrue(h.events.any { it.first == "guided_released" && it.second == "released" })
    }

    @Test
    fun `total silence - the armed policy runs, reason link-lost`() {
        val h = Harness()
        h.engage()
        repeat(35) { h.tick(100) } // no inbound anything for 3.5 s
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick stopping: link-lost"))
        repeat(20) { h.tick(100) }
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: link-lost"))
    }

    // ------------------------------------------------- the commanding controller (ControlOrigin)

    /**
     * The released/link-lost discrimination, driven through an explicitly named origin.
     *
     * `gcsSeenAtMs` stopped being a field and became a map keyed by [ControlOrigin], read for the
     * controller that established the engagement. With one origin that is arithmetically the old
     * field, and this is the same wind-down the suite already pins, asserted through the new
     * spelling: heartbeats from the engaging controller keep it `released`, and the ladder ends
     * where it did.
     */
    @Test
    fun `traffic from the engaging controller keeps the wind-down at released`() {
        val h = Harness()
        h.engage(origin = ControlOrigin.MAVLINK)
        var t = 0L
        while (t < GuidedEnvelope.LINK_LOST_MS + 200) {
            h.engine.onInbound("heartbeat", origin = ControlOrigin.MAVLINK)
            h.tick(100)
            t += 100
        }
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick stopping: released"))
        assertTrue(h.texts().none { it.contains("link-lost") })
    }

    /**
     * And its complement: silence from that controller is link loss, not a release, however the
     * origin was spelled.
     */
    @Test
    fun `silence from the engaging controller is still link loss`() {
        val h = Harness()
        h.engage(origin = ControlOrigin.MAVLINK)
        repeat(35) { h.tick(100) }
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick stopping: link-lost"))
    }

    /**
     * **The tripwire, re-armed.** Its predecessor (`one origin today - a second one must bring
     * the cross-origin tests with it`) fired on 2026-07-29 when [ControlOrigin.PHONE] was added
     * for landing08, and the debt it named was paid: the three formerly-unkillable mutants
     * (hardcoded MAVLINK key, any-origin-alive, origin never recorded) are now red — see the
     * class KDoc's 2026-07-29 table — killed by the PHONE-origin tests in
     * `GuidedTakeoffClimbTest` (the landing08 climb pair and the mid-climb QGC takeover) and
     * `GuidedTagDescentTest` (the landing08 arm/descent section).
     *
     * What remains untestable, and this trips for: PHONE is not a *transport* — it carries no
     * inbound traffic — so §4.3's transport-vs-transport property (a manoeuvre commanded by A
     * must run the armed link-loss policy when A goes silent, even while transport B chatters)
     * still cannot be expressed. Whoever adds `ZENOH` owes those tests, and this failing is how
     * they find out.
     */
    @Test
    fun `one transport today - a second one must bring the cross-transport tests with it`() {
        assertEquals(
            "adding a transport ControlOrigin (ZENOH) means §4.3's cross-transport liveness " +
                "rule becomes testable — write those tests before deleting this one",
            listOf(ControlOrigin.MAVLINK, ControlOrigin.PHONE),
            ControlOrigin.values().toList(),
        )
    }

    @Test
    fun `freeze-and-hold holds zero forever and never releases`() {
        val h = Harness(policy = FreezeAndHold())
        h.engage()
        repeat(35) { h.tick(100) }
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        // Thirty seconds later: still holding zero, still engaged, never disabled.
        repeat(300) { h.tick(100) }
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        assertEquals(0, h.port.disableCalls)
        assertEquals(0.0, h.port.sent.last().pitch, 1e-9)
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-9)
        assertTrue(h.texts().none { it.startsWith("Virtual stick off:") })
    }

    @Test
    fun `instant handback releases without a wind-down`() {
        val h = Harness(policy = InstantHandback())
        h.engage()
        var guard = 0
        while (h.engine.phase != GuidedPhase.RELEASING && guard++ < 50) h.tick(100)
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        val sendsAtLoss = h.port.sent.size
        // One more tick executes the zero-length plan: disable, no ramp/hold sends at all.
        h.tick(100)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: link-lost"))
        assertEquals(sendsAtLoss, h.port.sent.size)
    }

    @Test
    fun `input returning during the wind-down resumes passthrough`() {
        val h = Harness()
        h.engage()
        repeat(35) { h.tick(100) }
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        h.frame(x = 500)
        h.tick(10)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.texts().contains(GuidedStatusTexts.RESUMED))
        assertEquals(0, h.port.disableCalls)
        h.tick(50)
        assertEquals(1.5, h.port.sent.last().roll, 1e-9)
    }

    // ------------------------------------------------------- the abort gestures

    @Test
    fun `gesture 1 - RC deflection sustained past the debounce hands back`() {
        val h = Harness()
        h.engage()
        h.port.onRc!!(RcSticks(0, 0, 0, 200)) // 200/660 ≈ 0.30, well past the deadband
        h.tick(GuidedStickEngine.RC_ABORT_SUSTAIN_MS)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: sticks"))
        // The last thing streamed was the zero setpoint — get out of the way, then let go.
        val last = h.port.sent.last()
        assertEquals(0.0, last.pitch, 1e-9)
        assertEquals(0.0, last.verticalThrottle, 1e-9)
    }

    @Test
    fun `gesture 1 - a brief blip inside the debounce does not abort`() {
        val h = Harness()
        h.engage()
        h.port.onRc!!(RcSticks(0, 0, 0, 200))
        h.tick(100) // 100 ms < 200 ms sustain
        h.port.onRc!!(RcSticks(0, 0, 0, 0)) // back to centre
        h.tick(100)
        h.tick(100)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertEquals(0, h.port.disableCalls)
    }

    @Test
    fun `gesture 2 - the interlock switch aborts on the next tick`() {
        val h = Harness()
        h.engage()
        h.interlock = false
        h.tick(100)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: interlock"))
    }

    @Test
    fun `gesture 3 - DJI reporting authority elsewhere aborts immediately, no tick needed`() {
        val h = Harness()
        h.engage()
        h.port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "RC"))
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: authority RC"))
    }

    @Test
    fun `gesture 3 - virtual stick reported off aborts even with authority still MSDK`() {
        val h = Harness()
        h.engage()
        h.port.onState!!(VirtualStickSnapshot(enabled = false, advanced = true, authority = "MSDK"))
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
    }

    // ------------------------------------------- gesture 3 by another wire: mode seizure
    //
    // Measured 2026-07-26 (record 20260726-204721.001): DJI's overheat GO_HOME flew the
    // aircraft for 40 s while VirtualStickState still reported enabled+advanced+MSDK. The
    // flight mode is the only wire that says DJI seized the aircraft.

    @Test
    fun `mode leaving JOYSTICK while engaged aborts as an authority loss`() {
        val h = Harness()
        h.engage()
        h.state = h.state.copy(flightMode = "GO_HOME")
        h.tick(GuidedStickEngine.MODE_SEIZE_GRACE_MS + 100)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: authority MODE_GO_HOME"))
    }

    @Test
    fun `a non-JOYSTICK mode inside the grace window does not abort - DJI's flip is not instant`() {
        // The measured mode flip after ENGAGED was <=0.3 s; the grace exists so that delivery
        // lag is not read as a takeover.
        val h = Harness()
        h.engage()
        h.state = h.state.copy(flightMode = "GPS_ATTI")
        h.tick(GuidedStickEngine.MODE_SEIZE_GRACE_MS - 500)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        h.state = h.state.copy(flightMode = "JOYSTICK")
        h.tick(1000)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `null mode is never read as seizure - component-gone stays with its own abort paths`() {
        val h = Harness()
        h.engage()
        check(h.state.flightMode == null)
        h.tick(GuidedStickEngine.MODE_SEIZE_GRACE_MS + 1000)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `JOYSTICK mode past the grace window stays engaged`() {
        val h = Harness()
        h.engage()
        h.state = h.state.copy(flightMode = "JOYSTICK")
        h.tick(GuidedStickEngine.MODE_SEIZE_GRACE_MS + 1000)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `mode seizure also ends a freeze-and-hold wind-down - no infinite zombie hold`() {
        // FreezeAndHold holds zero forever by design; a DJI takeover during that hold must
        // still end it, or the engine holds a dead engagement for the rest of the flight.
        val h = Harness(policy = FreezeAndHold())
        h.engage()
        repeat(35) { h.tick(100) }
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        h.state = h.state.copy(flightMode = "GO_HOME")
        h.tick(100)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: authority MODE_GO_HOME"))
    }

    @Test
    fun `gesture 3 - every authority-change reason aborts except our own MSDK_REQUEST`() {
        for (reason in listOf(
            "RC_SWITCH", "RC_PAUSE_STOP", "RC_NOT_P_MODE", "RC_ONE_KEY_GO_HOME", "RC_LOST",
            "NEAR_BOUNDARY", "BATTERY_LOW_GO_HOME", "BATTERY_SUPER_LOW_LANDING",
        )) {
            val h = Harness()
            h.engage()
            h.port.onReason!!(reason)
            assertEquals("reason $reason must abort", GuidedPhase.IDLE, h.engine.phase)
            assertTrue(h.texts().contains(GuidedStatusTexts.off("authority", reason)))
        }
        val h = Harness()
        h.engage()
        h.port.onReason!!("MSDK_REQUEST")
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `gesture 3 - a null RC stick is component-gone, never centred sticks`() {
        // Landmine 6: the measured FC blackout delivers null across the board. A null while
        // holding authority is authority lost.
        val h = Harness()
        h.engage()
        h.port.onRc!!(RcSticks(0, null, 0, 0))
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick off: authority RC_STICK_NULL"))
    }

    @Test
    fun `gesture 3 - the product disappearing aborts on the next tick`() {
        val h = Harness()
        h.engage()
        h.port.unavailable = "NO_PRODUCT"
        h.tick(100)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick off: authority NO_PRODUCT"))
    }

    @Test
    fun `a send that fails costs the engagement`() {
        val h = Harness()
        h.engage()
        h.port.sendError = "java.lang.IllegalStateException: not flying"
        h.tick(100)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick off: authority SEND_FAILED"))
        assertEquals(false, h.cmds.first { it.error != null }.accepted)
    }

    @Test
    fun `abort is idempotent - a second trigger finds nothing to do`() {
        val h = Harness()
        h.engage()
        h.engine.abort(GuidedStickEngine.DisengageReason.RC_STICKS)
        h.engine.abort(GuidedStickEngine.DisengageReason.INTERLOCK)
        assertEquals(1, h.port.disableCalls)
        assertEquals(1, h.texts().count { it.startsWith("Virtual stick off:") })
    }

    // ---------------------------------------------------------- envelope limits

    @Test
    fun `five minutes of neutral releases with reason idle`() {
        val h = Harness()
        h.engage()
        h.now += 40
        h.frame() // neutral
        var elapsed = 0L
        while (elapsed <= GuidedEnvelope.IDLE_DISENGAGE_MS + 1_000) {
            h.frame()
            h.tick(500)
            elapsed += 500
        }
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: idle"))
    }

    @Test
    fun `sixty seconds of continuous deflection is a stuck axis and aborts`() {
        val h = Harness()
        h.engage(x = 500)
        var elapsed = 0L
        while (elapsed <= GuidedEnvelope.MANOEUVRE_TIMEOUT_MS + 1_000) {
            h.frame(x = 500)
            h.tick(200)
            elapsed += 200
        }
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick off: timeout"))
    }

    @Test
    fun `crossing neutral resets the manoeuvre clock`() {
        val h = Harness()
        h.engage(x = 500)
        var elapsed = 0L
        while (elapsed <= GuidedEnvelope.MANOEUVRE_TIMEOUT_MS - 10_000) {
            h.frame(x = 500)
            h.tick(200)
            elapsed += 200
        }
        h.frame() // through neutral
        h.tick(100)
        repeat(100) {
            h.frame(x = 500)
            h.tick(200)
        }
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `at the ceiling a climb is zeroed and announced - lateral and descent untouched`() {
        val h = Harness()
        h.state = AircraftState(relativeAltitude = 101.0, ages = SampleAges.of(Signal.ALTITUDE to 0L))
        h.engage()
        h.now += 40
        h.frame(x = 1000, z = 1000)
        h.tick(40)
        val sent = h.port.sent.last()
        assertEquals(0.0, sent.verticalThrottle, 1e-9)
        assertEquals(3.0, sent.roll, 1e-9)
        assertTrue(h.texts().contains(GuidedStatusTexts.CEILING))
        // Descent is never the ceiling's business.
        h.frame(z = 0)
        h.tick(40)
        assertEquals(-1.5, h.port.sent.last().verticalThrottle, 1e-9)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `below the ceiling a climb passes`() {
        val h = Harness()
        h.state = AircraftState(relativeAltitude = 29.0, ages = SampleAges.of(Signal.ALTITUDE to 0L))
        h.engage()
        h.now += 40
        h.frame(z = 1000)
        h.tick(40)
        assertEquals(+1.5, h.port.sent.last().verticalThrottle, 1e-9)
    }

    @Test
    fun `no altitude reading blocks the climb - the ceiling cannot be enforced blind`() {
        val h = Harness()
        h.state = AircraftState() // no relativeAltitude at all
        h.engage()
        h.now += 40
        h.frame(z = 1000)
        h.tick(40)
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-9)
        assertTrue(h.texts().contains(GuidedStatusTexts.NO_ALTITUDE))
    }

    @Test
    fun `a stale altitude blocks the climb the same way`() {
        val h = Harness()
        h.state = AircraftState(
            relativeAltitude = 5.0,
            ages = SampleAges.of(Signal.ALTITUDE to 10_000L),
        )
        h.engage()
        h.now += 40
        h.frame(z = 1000)
        h.tick(40)
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-9)
    }

    // -------------------------------------------------------------- housekeeping

    @Test
    fun `stop aborts an engagement and cancels the listens`() {
        val h = Harness()
        h.engage()
        h.engine.stop()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: stopped"))
        assertEquals(1, h.port.cancelled)
    }

    @Test
    fun `per-frame refusal logging is deduplicated - one line per reason per window`() {
        // The gates are consulted at QGC's 25 Hz; before the throttle a held stick against a
        // refusing gate printed 25 logcat lines a second (measured, first-engagement session).
        val h = Harness()
        h.interlock = false
        repeat(50) {
            h.frame(x = 800)
            h.now += 40 // 50 frames over 2 s, all inside the window
        }
        assertEquals(1, h.logs.count { it.startsWith("deliberate deflection not engaging") })
        // Past the window the same reason logs once more.
        h.now += GuidedStickEngine.LOG_REPEAT_MS
        h.frame(x = 800)
        assertEquals(2, h.logs.count { it.startsWith("deliberate deflection not engaging") })
    }

    @Test
    fun `a changed refusal reason logs immediately - the first occurrence is never suppressed`() {
        val h = Harness()
        h.interlock = false
        h.frame(x = 800)
        assertEquals(1, h.logs.count { it.contains("not engaging") })
        // Same instant, different reason: the interlock goes on but the SDK goes away.
        h.interlock = true
        h.port.unavailable = "NO_PRODUCT"
        h.frame(x = 800)
        assertEquals(1, h.logs.count { it.contains("not engaging: Sticks ignored: NO_PRODUCT") })
        // And the retry-window line is throttled on the same mechanism.
        h.port.unavailable = null
        h.frame() // seen at rest
        h.now += 40
        h.frame(x = 500) // engages: stamps the retry window
        h.engine.abort(GuidedStickEngine.DisengageReason.INTERLOCK)
        repeat(30) {
            h.frame(x = 500)
            h.now += 40
        }
        assertEquals(1, h.logs.count { it.contains("engage retry window") })
    }

    @Test
    fun `identical announcements are suppressed inside the window and re-emitted after it`() {
        val h = Harness()
        h.frame(x = 800) // never seen at rest → CENTER_FIRST
        repeat(10) {
            h.now += 40
            h.frame(x = 800)
        }
        assertEquals(1, h.texts().count { it == GuidedStatusTexts.CENTER_FIRST })
        h.now += GuidedStickEngine.ANNOUNCE_REPEAT_MS
        h.frame(x = 800)
        assertEquals(2, h.texts().count { it == GuidedStatusTexts.CENTER_FIRST })
    }
}
