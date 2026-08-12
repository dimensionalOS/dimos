package com.dimensional.mini4pro.simulator

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Every decision this bridge makes about the aircraft simulator, driven through a fake
 * [SimulatorPort] and a fake clock.
 *
 * The suite is written to fail loudly for the mistakes that would put a fabricated aircraft in
 * front of an operator who believes it is real:
 *
 *  - reporting "off" from anything other than DJI saying so — a fresh process, a lost component,
 *    a start that failed
 *  - claiming the simulator is running because *we asked*, rather than because DJI observed it
 *  - failing to notice a simulator this process did not start (the forgotten enable, arriving
 *    through the one door `CommandInterlock`'s technique cannot close: the state lives on the
 *    aircraft, not in this process)
 *  - warning the ground station once and never again
 *  - silently stopping a simulator somebody else started
 *  - surviving a restart, i.e. any storage at all
 *
 * Mutation-checked 2026-07-26 — each breakage was made deliberately, the failing tests counted,
 * and the code reverted:
 *
 *  | # | mutation | tests that failed |
 *  |---|---|---|
 *  | M1 | fresh `observedStarted` initialised to `false` (UNKNOWN becomes OFF) | 3 |
 *  | M2 | `observedStarted == null` mapped to OFF in `computePhase` | 4 |
 *  | M3 | a `null` delivery stored as `false` (component-gone reads as "no simulator") | 1 |
 *  | M4 | FOREIGN collapsed into ACTIVE (`observedStarted == true -> ACTIVE`) | 6 |
 *  | M5 | ACTIVE claimed from our own request (`null && startRequested -> ACTIVE`) | **0, then 2** |
 *  | M6 | `noticeIfDue` emits for STARTING as well — the echo | 2 |
 *  | M7 | `noticeIfDue` returns the text on every call (no period) | 1 |
 *  | M8 | `noticeIfDue` fires once and never repeats | 1 |
 *  | M9 | `text != lastNoticeText` dropped (a changed claim waits out the period) | **0, then 1** |
 *  | M10 | `stopIfOurs` stops a FOREIGN simulator too | 3 |
 *  | M11 | `stopIfOurs` never withdraws anything | 3 |
 *  | M12 | `ALREADY_RUNNING` guard removed from `start` | 1 |
 *  | M13 | satellite range check removed | 2 |
 *  | M14 | satellite count silently clamped to `[0,20]` instead of refused | 2 |
 *  | M15 | `Geo` location check removed from `start` | 3 |
 *  | M16 | `unavailableReason` not consulted in `start` | 1 |
 *  | M17 | a delivered `false` stops clearing `startRequested` | 1 |
 *  | M18 | `observe()` not idempotent (re-subscribes every call) | 1 |
 *
 * Mutation-checked again 2026-07-27, for `startConfirmed` — the fix for a phase a real teardown
 * reported wrongly (`docs/measurements/2026-07-27-takeoff-climb-and-ccw-orbit.md` §7.2):
 *
 *  | # | mutation | tests that failed |
 *  |---|---|---|
 *  | M19 | the `!startConfirmed` conjunct dropped — i.e. the original defect restored | 1 |
 *  | M20 | `start()` stops clearing `startConfirmed` (a new request inherits the old answer) | **0, then 1** |
 *  | M21 | a delivered `false` stops clearing `startConfirmed` | **0, and it stays 0** |
 *
 * **Two of those need explaining, and neither explanation is "the test suite is fine".**
 *
 * *M20* survived its first test because that test reached the second `start()` via a delivered
 * `false`, which clears `startConfirmed` on the way past — so `start()`'s own clear was never the
 * thing under test. Rewritten to go confirmed → `null` → start, which is both the only route that
 * isolates it and the realistic one: a link that dropped out and came back is exactly when the
 * button gets pressed again.
 *
 * *M21 is honestly unkillable and the line is kept anyway.* `startConfirmed` is only ever read
 * alongside `startRequested`, and a delivered `false` clears `startRequested` in the same branch —
 * so nothing can observe whether `startConfirmed` was cleared with it. It stays because the branch's
 * contract is *"an observed off retires everything about the request"*, and a flag left set behind
 * that sentence is a trap for whoever reads it next. Recorded as 0 rather than quietly dropped from
 * the table: a mutation score of zero is a fact about the test suite, and hiding it would make the
 * other numbers mean less.
 *
 * **Two of the 2026-07-26 mutations found real holes, and both were in this file rather than in the
 * code.**
 *
 * *M5* survived the entire original suite. Every start test went through `startedFromOff()`,
 * which delivers a `false` before starting, so the `observedStarted == null && startRequested`
 * branch was never reached with a request outstanding — and that branch is the dangerous one: a
 * fresh process whose link never delivered the flag, where an operator presses the button and the
 * bridge would then have claimed ACTIVE, and warned QGC, from nothing but its own request. Two
 * tests were added (`a start from an unknown state still only claims STARTING` and its
 * accepted-by-DJI twin).
 *
 * *M9* survived because the re-arm test used a *second* `SimulatorControl`, which has no
 * outstanding notice and therefore cannot exercise a re-arm at all. Rewritten to change the claim
 * on one control, via a late `onFailure` after a delivered `true` — ACTIVE becoming FOREIGN, a
 * real race and the one case where the operator is currently being told the wrong thing.
 */
class SimulatorControlTest {

    // ------------------------------------------------------------------- rig

    private val port = FakeSimulatorPort()
    private val logs = mutableListOf<String>()
    private var nowMs = 0L

    private val control = SimulatorControl(
        port = port,
        log = { logs.add(it) },
        nowMs = { nowMs },
    )

    private fun elapse(ms: Long) {
        nowMs += ms
    }

    /** Athens, the site every measurement in this project was taken at. Not 0/0, not filler. */
    private val athens = SimulatorRequest(latitude = 37.9938232, longitude = 23.7253477, satelliteCount = 14)

    /** Bring the control up subscribed, with DJI having said the simulator is off. */
    private fun startedFromOff() {
        control.observe()
        port.deliverStarted(false)
    }

    /**
     * A recorded MSDK. Callbacks are captured, never resolved on their own: each test plays DJI's
     * side explicitly, which is what makes "DJI accepted but never reported it running" — the
     * measured 2026-07-26 shape of a `performAction` — writable as straight-line code.
     */
    private class FakeSimulatorPort : SimulatorPort {
        var reason: String? = null

        /** Every call that would have reached DJI, in order. */
        val performed = mutableListOf<String>()

        val starts = mutableListOf<Pair<() -> Unit, (String) -> Unit>>()
        val stops = mutableListOf<Pair<() -> Unit, (String) -> Unit>>()

        var startedListener: ((Boolean?) -> Unit)? = null
        var stateListener: ((SimulatedAircraft?) -> Unit)? = null
        var subscriptions = 0
        var cancelled = false

        override fun unavailableReason(): String? = reason

        override fun start(
            latitude: Double,
            longitude: Double,
            satelliteCount: Int,
            onSuccess: () -> Unit,
            onFailure: (String) -> Unit,
        ) {
            performed.add("start($latitude,$longitude,$satelliteCount)")
            starts.add(onSuccess to onFailure)
        }

        override fun stop(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            performed.add("stop")
            stops.add(onSuccess to onFailure)
        }

        override fun listenIsSimulatorStarted(onDelivery: (Boolean?) -> Unit) {
            subscriptions++
            startedListener = onDelivery
        }

        override fun listenSimulatorState(onDelivery: (SimulatedAircraft?) -> Unit) {
            stateListener = onDelivery
        }

        override fun cancelListens() {
            cancelled = true
        }

        fun deliverStarted(value: Boolean?) = startedListener!!.invoke(value)
        fun deliverState(value: SimulatedAircraft?) = stateListener!!.invoke(value)
    }

    // ------------------------------------------- "off" is a claim, not a default

    /**
     * The single most important assertion in the file. A fresh process knows nothing about a
     * simulator that lives on the aircraft, and the honest word for that is UNKNOWN.
     */
    @Test
    fun `a fresh control is UNKNOWN, never OFF`() {
        assertEquals(SimulatorPhase.UNKNOWN, control.phase)
    }

    @Test
    fun `subscribing alone does not make it off`() {
        control.observe()
        assertEquals(SimulatorPhase.UNKNOWN, control.phase)
    }

    @Test
    fun `only a delivered false is OFF`() {
        control.observe()
        port.deliverStarted(false)
        assertEquals(SimulatorPhase.OFF, control.phase)
    }

    /**
     * DJI's component-gone signal. We stop being able to see the flag, so we stop claiming to
     * know it — a `null` that read as `false` would report a running simulator as absent.
     */
    @Test
    fun `a null delivery returns to UNKNOWN, it is not an off`() {
        control.observe()
        port.deliverStarted(true)
        assertEquals(SimulatorPhase.FOREIGN, control.phase)
        port.deliverStarted(null)
        assertEquals(SimulatorPhase.UNKNOWN, control.phase)
    }

    /**
     * **The state a real teardown reported wrongly**, 2026-07-27: an aircraft disconnecting under a
     * running simulator of ours, logged as `ACTIVE → STARTING (DJI reports started=null)` 34 ms
     * before `fcConnected` went false — see
     * `docs/measurements/2026-07-27-takeoff-climb-and-ccw-orbit.md` §7.2.
     *
     * A simulator *going away* was announced to the operator as one *starting up*: the banner in
     * STARTING reads "⬤ SIMULATOR STARTING — DJI HAS NOT CONFIRMED", which is a true sentence about
     * a request in flight and a false one about a request that was answered nine minutes ago.
     * `startRequested` stayed true across the null — deliberately, because it also means "ours" and
     * [stopIfOurs] must not abandon a simulator that will come back — so the stale request was the
     * only thing still speaking. Hence `startConfirmed`.
     *
     * The distinction this pins: **STARTING is about us, UNKNOWN is about DJI.** Both are honest;
     * only one of them is true here.
     */
    @Test
    fun `a null after DJI confirmed is UNKNOWN, not a second STARTING`() {
        startedFromOff()
        control.start(athens)
        assertEquals(SimulatorPhase.STARTING, control.phase)
        port.deliverStarted(true)
        assertEquals(SimulatorPhase.ACTIVE, control.phase)
        port.deliverStarted(null)
        assertEquals(SimulatorPhase.UNKNOWN, control.phase)
    }

    /**
     * And the converse, so the fix cannot be read as "null is always UNKNOWN": a start that DJI has
     * genuinely never answered still says STARTING through a null, because there the sentence is
     * true. This is the case `a start from an unknown state still only claims STARTING` covers from
     * the other end, stated here as the pair to the test above so the two cannot drift apart.
     */
    @Test
    fun `a null with a start still in flight remains STARTING`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(null)
        assertEquals(SimulatorPhase.STARTING, control.phase)
    }

    /**
     * A second start after the first has been confirmed and the simulator has gone away must be
     * able to say STARTING again — otherwise the fix would make the banner permanently silent for
     * the rest of the process.
     *
     * **This route deliberately never delivers a `false`**, and that is the whole point of it. The
     * obvious version — confirmed, then `false`, then start again — passes even when `start()`
     * forgets to clear `startConfirmed`, because the `false` delivery clears it on the way past.
     * Written that way first; mutation M20 survived it (0 kills). Going confirmed → `null` → start
     * is the only path on which `start()`'s own clear is the thing being tested, and it is also the
     * realistic one: an aircraft that dropped out and came back is exactly when an operator presses
     * start again.
     */
    @Test
    fun `a fresh request after a confirmed one claims STARTING again`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        port.deliverStarted(null)
        assertEquals(SimulatorPhase.UNKNOWN, control.phase)
        control.start(athens)
        assertEquals(SimulatorPhase.STARTING, control.phase)
    }

    /** A component-gone must not make us forget the simulator is ours to stop. */
    @Test
    fun `a null delivery does not discard our claim on the simulator`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        port.deliverStarted(null)
        assertNotNull(control.stopIfOurs())
        assertTrue(port.performed.contains("stop"))
    }

    // ------------------------------------------------- the claim comes from DJI

    @Test
    fun `requesting a start claims nothing`() {
        startedFromOff()
        assertEquals(SimulatorOutcome.Requested, control.start(athens))
        assertEquals(SimulatorPhase.STARTING, control.phase)
    }

    /**
     * DJI accepting the request is still not the aircraft simulating. This is the same
     * distinction `MsdkFlightActions` draws for a landing start, for the same measured reason.
     */
    @Test
    fun `DJI accepting the start still does not make it ACTIVE`() {
        startedFromOff()
        control.start(athens)
        port.starts.single().first()
        assertEquals(SimulatorPhase.STARTING, control.phase)
    }

    /**
     * The same rule from the state we are *most* likely to press start in, and the one the
     * happy-path tests miss: a fresh process where DJI has never answered.
     *
     * Added after mutation M5 — `observedStarted == null && startRequested -> ACTIVE` — passed
     * the whole suite. Every other start test goes through `startedFromOff()`, which delivers a
     * `false` first and so never exercises the null branch with a request outstanding. Under that
     * mutation an operator pressing "Start simulator (state unknown)" on a link that had not
     * delivered the flag would have made this bridge claim ACTIVE, and send QGC the
     * SIMULATOR ACTIVE warning, purely from its own request. That is the exact echo the design
     * forbids, and nothing caught it.
     */
    @Test
    fun `a start from an unknown state still only claims STARTING`() {
        control.observe()
        assertEquals(SimulatorPhase.UNKNOWN, control.phase)
        control.start(athens)
        assertEquals(SimulatorPhase.STARTING, control.phase)
        assertNull(control.noticeIfDue(nowMs))
    }

    /** And DJI accepting it from that state is still not an observation. */
    @Test
    fun `a start accepted from an unknown state claims nothing either`() {
        control.observe()
        control.start(athens)
        port.starts.single().first()
        assertEquals(SimulatorPhase.STARTING, control.phase)
        assertNull(control.noticeIfDue(nowMs))
    }

    @Test
    fun `only the delivered flag makes it ACTIVE`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        assertEquals(SimulatorPhase.ACTIVE, control.phase)
    }

    @Test
    fun `a refused start retires the request without claiming anything`() {
        startedFromOff()
        control.start(athens)
        port.starts.single().second("FC_AUTH_STATE")
        assertEquals(SimulatorPhase.OFF, control.phase)
        assertTrue(logs.any { it.contains("FC_AUTH_STATE") })
    }

    /**
     * A start we asked for, DJI refused, and a simulator running anyway is not ours. Getting this
     * backwards would let `stopIfOurs` stop somebody else's.
     */
    @Test
    fun `a simulator running after our start was refused is FOREIGN`() {
        startedFromOff()
        control.start(athens)
        port.starts.single().second("FC_AUTH_STATE")
        port.deliverStarted(true)
        assertEquals(SimulatorPhase.FOREIGN, control.phase)
        assertNull(control.stopIfOurs())
    }

    // ------------------------------------------------- the forgotten enable

    /**
     * The failure this whole class exists for: a simulator left running by a crashed session,
     * DJI Assistant, or another app. `CommandInterlock` cannot help — its state dies with the
     * process and the simulator's does not.
     */
    @Test
    fun `a simulator running before we ever asked is FOREIGN`() {
        control.observe()
        port.deliverStarted(true)
        assertEquals(SimulatorPhase.FOREIGN, control.phase)
    }

    @Test
    fun `a foreign simulator is not stopped automatically`() {
        control.observe()
        port.deliverStarted(true)
        assertNull(control.stopIfOurs())
        assertTrue(port.performed.isEmpty())
    }

    /** But the operator can always end one deliberately. */
    @Test
    fun `a foreign simulator can be stopped on request`() {
        control.observe()
        port.deliverStarted(true)
        assertEquals(SimulatorOutcome.Requested, control.stop())
        assertEquals(listOf("stop"), port.performed)
        assertEquals(SimulatorPhase.STOPPING, control.phase)
    }

    // ------------------------------------------------------------- not persisted

    /**
     * The structural half of "does not survive a restart": there is no constructor parameter for
     * an initial state and no storage of any kind, so a second instance over the same port begins
     * knowing nothing, exactly as a second process would.
     *
     * The behavioural half is that the *aircraft* may still be simulating — which is why the
     * second instance reads FOREIGN rather than OFF once DJI answers, and why that is a louder
     * state than the first instance's ACTIVE.
     */
    @Test
    fun `a new instance over the same aircraft starts blind and then reads FOREIGN`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        assertEquals(SimulatorPhase.ACTIVE, control.phase)

        val restarted = SimulatorControl(port = port, nowMs = { nowMs })
        assertEquals(SimulatorPhase.UNKNOWN, restarted.phase)
        restarted.observe()
        port.deliverStarted(true)
        assertEquals(SimulatorPhase.FOREIGN, restarted.phase)
    }

    // ------------------------------------------------------------------ stopping

    @Test
    fun `stopIfOurs stops a simulator we started`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        assertEquals(SimulatorOutcome.Requested, control.stopIfOurs())
        assertTrue(port.performed.contains("stop"))
    }

    @Test
    fun `stopIfOurs does nothing when nothing is ours`() {
        startedFromOff()
        assertNull(control.stopIfOurs())
        assertTrue(port.performed.isEmpty())
    }

    /** DJI accepting the stop is not the simulator having stopped. */
    @Test
    fun `accepting the stop leaves it STOPPING until DJI reports off`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        control.stop()
        port.stops.single().first()
        assertEquals(SimulatorPhase.STOPPING, control.phase)
        port.deliverStarted(false)
        assertEquals(SimulatorPhase.OFF, control.phase)
    }

    /**
     * An unreachable aircraft cannot be told to stop. Reporting that is the honest outcome;
     * quietly forgetting the simulator would leave a running one nobody is tracking.
     */
    @Test
    fun `stopIfOurs refuses rather than pretends when the aircraft is gone`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        port.reason = "NO_PRODUCT"
        assertEquals(SimulatorOutcome.Refused("NO_PRODUCT"), control.stopIfOurs())
        assertTrue(port.performed.none { it == "stop" })
    }

    @Test
    fun `stop refuses when DJI says the simulator is already off`() {
        startedFromOff()
        assertEquals(SimulatorOutcome.Refused("NOT_RUNNING"), control.stop())
    }

    // ------------------------------------------------------------ start refusals

    @Test
    fun `a second start while one is running is refused`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        assertEquals(SimulatorOutcome.Refused("ALREADY_RUNNING"), control.start(athens))
        assertEquals(1, port.starts.size)
    }

    @Test
    fun `a second start while one is in flight is refused`() {
        startedFromOff()
        control.start(athens)
        assertEquals(SimulatorOutcome.Refused("START_IN_FLIGHT"), control.start(athens))
        assertEquals(1, port.starts.size)
    }

    @Test
    fun `an unregistered SDK refuses the start before any key is touched`() {
        startedFromOff()
        port.reason = "SDK_NOT_REGISTERED"
        assertEquals(SimulatorOutcome.Refused("SDK_NOT_REGISTERED"), control.start(athens))
        assertTrue(port.performed.isEmpty())
    }

    /**
     * DJI's range is `@IntRange(from=0,to=20)`. Refused rather than clamped: the satellite count
     * decides whether the simulated aircraft believes it has a fix, and silently changing it
     * would make the operator's mental model wrong about the thing they are testing.
     */
    @Test
    fun `a satellite count above DJI's range is refused, not clamped`() {
        startedFromOff()
        assertEquals(
            SimulatorOutcome.Refused("BAD_SATELLITE_COUNT"),
            control.start(athens.copy(satelliteCount = 21)),
        )
        assertTrue(port.performed.isEmpty())
    }

    @Test
    fun `a negative satellite count is refused`() {
        startedFromOff()
        assertEquals(
            SimulatorOutcome.Refused("BAD_SATELLITE_COUNT"),
            control.start(athens.copy(satelliteCount = -1)),
        )
    }

    @Test
    fun `the range bounds themselves are accepted`() {
        startedFromOff()
        assertEquals(SimulatorOutcome.Requested, control.start(athens.copy(satelliteCount = 0)))
        port.starts.single().second("x")
        assertEquals(SimulatorOutcome.Requested, control.start(athens.copy(satelliteCount = 20)))
    }

    /**
     * The same `Geo` rule the telemetry path uses, so a coordinate this bridge would refuse to
     * *report* is also one it refuses to *seed a simulation with*. 0/0 is the canonical
     * unknown-encoded-as-zero mistake and `Geo` rejects it via the lat==lon rule.
     */
    @Test
    fun `null island is refused`() {
        startedFromOff()
        assertEquals(
            SimulatorOutcome.Refused("BAD_LOCATION"),
            control.start(athens.copy(latitude = 0.0, longitude = 0.0)),
        )
    }

    @Test
    fun `an out-of-range latitude is refused`() {
        startedFromOff()
        assertEquals(
            SimulatorOutcome.Refused("BAD_LOCATION"),
            control.start(athens.copy(latitude = 4.583662361046586E7)),
        )
    }

    @Test
    fun `a NaN coordinate is refused`() {
        startedFromOff()
        assertEquals(
            SimulatorOutcome.Refused("BAD_LOCATION"),
            control.start(athens.copy(longitude = Double.NaN)),
        )
    }

    @Test
    fun `an accepted request reaches DJI with the operator's own numbers`() {
        startedFromOff()
        control.start(athens)
        assertEquals(listOf("start(37.9938232,23.7253477,14)"), port.performed)
    }

    // ------------------------------------------------------------- the wire notice

    @Test
    fun `nothing goes on the wire while the simulator is off`() {
        startedFromOff()
        assertNull(control.noticeIfDue(nowMs))
    }

    /**
     * The echo rule. A warning emitted on our own start request would be a claim sourced from a
     * request rather than an observation — the thing `PLAN.md` forbids for the heartbeat's mode,
     * applied to the one other place this bridge makes a claim.
     */
    @Test
    fun `nothing goes on the wire while a start is merely requested`() {
        startedFromOff()
        control.start(athens)
        assertEquals(SimulatorPhase.STARTING, control.phase)
        assertNull(control.noticeIfDue(nowMs))
    }

    @Test
    fun `the notice goes out as soon as DJI reports it running`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        assertEquals(SimulatorNotice.WIRE_ACTIVE, control.noticeIfDue(nowMs))
    }

    @Test
    fun `a foreign simulator gets its own wording`() {
        control.observe()
        port.deliverStarted(true)
        assertEquals(SimulatorNotice.WIRE_FOREIGN, control.noticeIfDue(nowMs))
    }

    /** Once per period, not once per tick: the telemetry loop calls this five times a second. */
    @Test
    fun `the notice is not repeated within its period`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        assertNotNull(control.noticeIfDue(nowMs))
        elapse(SimulatorControl.NOTICE_PERIOD_MS - 1)
        assertNull(control.noticeIfDue(nowMs))
    }

    /**
     * And it *is* repeated. A single line on connect is missed by a ground station that attached
     * afterwards, by an operator who joined late, and by QGC's scrolling message panel.
     */
    @Test
    fun `the notice repeats every period for as long as it is true`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        assertNotNull(control.noticeIfDue(nowMs))
        elapse(SimulatorControl.NOTICE_PERIOD_MS)
        assertNotNull(control.noticeIfDue(nowMs))
        elapse(SimulatorControl.NOTICE_PERIOD_MS)
        assertNotNull(control.noticeIfDue(nowMs))
    }

    /**
     * A change of claim must not wait out the period — on the *same* control.
     *
     * Written this way after mutation M9 (dropping the `text != lastNoticeText` clause) passed
     * against an earlier version of this test that used a second `SimulatorControl`: a fresh
     * instance has no outstanding notice, so it could never have exercised the re-arm at all.
     *
     * The sequence is a real race, not a contrivance. DJI reports the simulator running, so we
     * claim ACTIVE and warn; *then* our own start's `onFailure` arrives late, which means the
     * running simulator was never ours and the correct claim is FOREIGN. The operator must be
     * told that within a tick, not up to five seconds later.
     */
    @Test
    fun `a changed claim re-arms the notice immediately on the same control`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        assertEquals(SimulatorNotice.WIRE_ACTIVE, control.noticeIfDue(nowMs))

        port.starts.single().second("FC_AUTH_STATE")
        assertEquals(SimulatorPhase.FOREIGN, control.phase)
        assertEquals(SimulatorNotice.WIRE_FOREIGN, control.noticeIfDue(nowMs))
    }

    /** And a second, independent observer of the same aircraft calls it foreign from the start. */
    @Test
    fun `another instance watching the same aircraft warns that it is foreign`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)

        val other = SimulatorControl(port = port, nowMs = { nowMs })
        other.observe()
        port.deliverStarted(true)
        assertEquals(SimulatorNotice.WIRE_FOREIGN, other.noticeIfDue(nowMs))
    }

    @Test
    fun `the notice stops the moment DJI reports it off`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        assertNotNull(control.noticeIfDue(nowMs))
        port.deliverStarted(false)
        assertNull(control.noticeIfDue(nowMs))
    }

    /** A simulator we can no longer see is one we can no longer make claims about. */
    @Test
    fun `the notice stops when the flag goes unreadable`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        assertNotNull(control.noticeIfDue(nowMs))
        port.deliverStarted(null)
        assertNull(control.noticeIfDue(nowMs))
    }

    /** Still simulated while a stop is pending, so the warning must not stop early. */
    @Test
    fun `the notice continues while a stop is unconfirmed`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        control.stop()
        elapse(SimulatorControl.NOTICE_PERIOD_MS)
        assertEquals(SimulatorNotice.WIRE_ACTIVE, control.noticeIfDue(nowMs))
    }

    // ---------------------------------------------------------------- plumbing

    @Test
    fun `observe subscribes once however often it is called`() {
        control.observe()
        control.observe()
        control.observe()
        assertEquals(1, port.subscriptions)
    }

    @Test
    fun `the simulated aircraft is carried through for display`() {
        control.observe()
        port.deliverStarted(true)
        port.deliverState(SimulatedAircraft(motorsOn = true, flying = true, positionZ = -12.5))
        val snapshot = control.snapshot()
        assertEquals(true, snapshot.aircraft?.motorsOn)
        assertEquals(-12.5, snapshot.aircraft?.positionZ!!, 1e-9)
    }

    @Test
    fun `how long it has been running is measured from the first delivery`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        elapse(30_000)
        assertEquals(30_000L, control.snapshot().activeForMs)
    }

    @Test
    fun `an off delivery clears the elapsed time and the aircraft`() {
        startedFromOff()
        control.start(athens)
        port.deliverStarted(true)
        port.deliverState(SimulatedAircraft(motorsOn = true))
        port.deliverStarted(false)
        val snapshot = control.snapshot()
        assertNull(snapshot.activeForMs)
        assertNull(snapshot.aircraft)
    }
}
