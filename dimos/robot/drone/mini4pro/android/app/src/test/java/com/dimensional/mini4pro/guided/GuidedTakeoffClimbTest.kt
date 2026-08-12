package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.ClimbArm
import com.dimensional.mini4pro.command.CommandDispatcher
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
import io.dronefleet.mavlink.common.ManualControl
import io.dronefleet.mavlink.common.Statustext
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The takeoff's **second phase** as [GuidedStickEngine] flies it: the pending climb armed by
 * M2.5's accepted takeoff, held while DJI flies its own ~1.2 m hop, and then flown as an ordinary
 * Change Altitude. The phase machine's own logic is `TakeoffClimbTest`; this suite is about the
 * engine — that the climb reaches the real guidance path, and that **every rung of the abort
 * ladder reaches the climb**.
 *
 * Same protocol as `GuidedRepositionTest`: fake port, hand-cranked clock, no aircraft.
 *
 * The dangerous failure here is not a climb that does not happen. It is **a pending climb that
 * survives an abort and fires later**, into a situation nobody remembers creating — an operator
 * who grabbed the RC between the phases, watched the bridge let go, and then watched it take the
 * aircraft back thirty seconds later. Six tests exist for that one property, one per rung, and
 * each of them runs the full DJI takeoff sequence *after* the abort and asserts that nothing
 * engages and nothing is sent.
 *
 * Written to fail loudly for:
 *
 *  - a climb that fires while DJI is still flying its own takeoff (`AUTO_TAKE_OFF`)
 *  - a climb that survives any abort rung
 *  - the takeoff's relative height put through the AMSL datum subtraction, which would turn a
 *    3 m climb into a 97 m descent request against this suite's datum of 100
 *  - a horizontal component in what is meant to be a vertical-only manoeuvre
 *  - a ceiling cap that happens silently, or not at all
 *  - an armed climb with no expiry
 *
 * Mutation-checked 2026-07-27, one breakage at a time, `./gradlew testDebugUnitTest` after each,
 * code reverted after each. Counts are across this file, `TakeoffClimbTest` and
 * `CommandDispatcherTest` — measured, not estimated. The full table is in `TakeoffClimbTest`'s
 * header for the machine's own mutants; these are the engine's:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `abort` clears the climb *below* its IDLE early-return (an abort at IDLE leaves it armed) | 2 |
 *  | `abort` stops clearing the climb at all | 2 |
 *  | the interlock rung removed from `tickTakeoffClimbLocked` | 2 |
 *  | the RC-sticks rung removed from `tickTakeoffClimbLocked` | 1 |
 *  | the GCS-deflection cancel removed from `onFrame` | 1 |
 *  | the GCS-deflection cancel ungated (a centre-zero stream cancels every climb) | 1 |
 *  | `tickTakeoffClimbLocked` decides before it cancels (a revoking tick still fires) | 3 |
 *  | `cancelTakeoffClimb` no longer reached by an async DJI error | 1 |
 *  | the climb fired as `Amsl` instead of `Relative` (datum subtracted twice) | 7 |
 *  | the climb fired with an explicit coordinate of (0, 0) instead of the current fix | 7 |
 *  | `tickTakeoffClimbLocked` not called from `tick` at all | 13 |
 *  | arming moved outside `onTakeoff`'s ACCEPTED branch (a refused takeoff arms a climb) | 6 |
 *
 * **No mutant survived**, and two of the numbers are worth reading rather than counting.
 *
 * The two abort mutants score **2 each — the lowest in the table, and they are the dangerous
 * ones.** That is not a gap; it is the shape of the property. Only two tests can express "an
 * abort left the climb armed", because only two of the six rungs reach `abort` at all: the other
 * four (the interlock and RC rungs in the tick, the GCS deflection, the async DJI error) cancel
 * through their own paths and are pinned by their own mutants, immediately above and below. The
 * layering is deliberate — several independent reasons a cancelled climb cannot fly — and each
 * layer is pinned alone, which is `GuidedRepositionTest`'s own lesson about the
 * target-survives-abort mutant that first scored 0.
 *
 * Every "…and it does not come back" test then re-runs the entire DJI takeoff sequence after the
 * abort and asserts `enableCalls == 0`, so a mutant that merely *delays* the firing is caught by
 * the same two tests rather than by nothing.
 *
 * ## Addendum, 2026-07-28: the phone takeoff and the believed camera pitch
 *
 * The phone's own Take off button (10 m, camera to nadir at DJI's handback) rides this machinery
 * — `CommandDispatcher.takeoffFromPhone` → the same corridor as `MAV_CMD_NAV_TAKEOFF` → the
 * `aimCameraNadir` flag on the armed climb — and the same day's camera work (`gimbal/PitchBelief`,
 * the descent gate's honest camera refusal) shares its campaign. Mutation-checked 2026-07-28
 * against the finished tree (suite **2458**), one breakage at a time, whole suite per mutant,
 * `test-results` deleted first, confirmed red, reverted — counts are failing tests across the
 * whole suite, **measured, not estimated**:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the interlock gate dropped from `takeoffFromPhone` | 1 |
 *  | `PHONE_TAKEOFF_HEIGHT_M` broken (10 → 1 m) | 5 |
 *  | the gimbal command dropped from the sequence (`flyTakeoffClimb` no longer aims) | 2 |
 *  | the phone door bypasses the shared corridor (range gate skipped) | 1 |
 *  | the honest-refusal reorder reverted (camera gate back below the fix gate) | 2 |
 *  | `PitchBelief` substitutes 0.0 when both sources are null | 4 |
 *  | precedence flipped (reported outranks commanded) | 3 |
 *  | the reported provenance flag stuck false | 3 |
 *  | the nadir tolerance skipped for reported-pitch poses (`TagWorld.fix`) | 1 |
 *  | the camera flag dropped at the dispatcher's climb handoff | 1 |
 *  | the machine's handback drops the camera flag (`TakeoffClimb.observe`) | 3 |
 *  | `GimbalManager.believedPitch` grows a second commanded/reported resolution | 1 |
 *
 * **No mutant survived.** The three 1-count rows are single-property gates each pinned by the one
 * test written for exactly that property (the interlock refusal, the corridor's range gate, the
 * manager's one-owner forwarding) — narrow on purpose, like the abort rows above: one layer, one
 * test, pinned alone. Note what the single-owner row does and does not prove: the *resolution*
 * cannot be duplicated inside `gimbal/` without going red, while the Bridge wirings that consume
 * it (`believedCameraPitch` feeding the engine seam, the CameraPose and the `tf` edge) are
 * Android-side and pinned by review plus the KDoc rule at `Bridge.believedCameraPitch`, the same
 * standing every Bridge wiring has.
 *
 * ## Addendum, 2026-07-29: `ControlOrigin.PHONE` (landing08) and the both-doors camera flag
 *
 * Two changes from the same day share this campaign (protocol as above; whole suite per mutant,
 * `test-results` deleted first, confirmed red, reverted; counts are failing tests across the
 * whole suite, **measured, not estimated**). First, the commanding origin now rides the armed
 * climb beside the camera flag — landing08's fix; the origin-side mutants and their counts live
 * in `GuidedStickEngineTest`'s 2026-07-29 table, the origin story's home. Second, the sole
 * operator declared the camera-down sequence wanted on every takeoff (*"I'm the only operator
 * and prefer camera down"*), so **both** doors now pass `aimCameraNadir = true`; the flag's
 * plumbing stays per-door, and the addendum row above whose truth this changed was re-measured
 * against the new tree:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the camera flag dropped at the dispatcher's handoff — **both** doors pass false (re-measure of the 2026-07-28 row) | 3 |
 *
 * The origin and the flag are independent facts on the climb (a MAVLINK-origin climb now
 * carries the flag too); the independence is pinned in `CommandDispatcherTest`'s
 * `a QGC takeoff labels its climb MAVLINK` (flag true, origin MAVLINK, same arm).
 */
class GuidedTakeoffClimbTest {

    private companion object {
        /** The project's home latitude family — deliberately non-equatorial. */
        const val LAT = 38.0
        const val LON = 23.7

        /**
         * The published pressure-altitude datum this session. Large and non-zero on purpose: if
         * the takeoff's *relative* height is ever put through the AMSL subtraction, a 3.048 m
         * climb becomes −96.95 m and is refused as below-datum, which several tests here catch.
         */
        const val DATUM = 100.0

        /** QGC's own takeoff floor, `FirmwarePlugin.h:204` — the height a stock press asks for. */
        const val QGC_MIN_TAKEOFF_M = 3.048

        /** Where DJI's own takeoff leaves the aircraft. */
        const val DJI_HOP_M = CommandDispatcher.DJI_TAKEOFF_HEIGHT_M
    }

    private class FakeVirtualStickPort : VirtualStickPort {
        var unavailable: String? = null
        var enableCalls = 0
        var disableCalls = 0
        var enableOnSuccess: (() -> Unit)? = null
        var enableOnFailure: ((String) -> Unit)? = null

        data class Sent(val pitch: Double, val roll: Double, val yaw: Double, val verticalThrottle: Double)

        val sent = mutableListOf<Sent>()
        var onState: ((VirtualStickSnapshot) -> Unit)? = null
        var onReason: ((String) -> Unit)? = null
        var onRc: ((RcSticks) -> Unit)? = null

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
            this.onReason = onAuthorityReason
        }

        override fun listenRcSticks(onDelivery: (RcSticks) -> Unit) {
            this.onRc = onDelivery
        }

        override fun cancelListens() = Unit
    }

    private class Harness {
        var now = 1_000L
        var interlock = true

        /** What DJI reports. Starts as a parked aircraft on the ground with the link up. */
        var flying: Boolean? = false
        var mode: String? = "GPS_ATTI"
        var alt: Double? = 0.0

        val port = FakeVirtualStickPort()
        val wire = mutableListOf<Any>()
        val events = mutableListOf<Pair<String, String?>>()

        /** The camera, for the phone sequence: what was aimed, in order. Open loop, like the real one. */
        val aimed = mutableListOf<Double>()

        val engine = GuidedStickEngine(
            port = port,
            interlockEnabled = { interlock },
            aircraftState = { state() },
            announcer = Announcer(StatusTextSink { wire += it }),
            manoeuvreGimbal = object : ManoeuvreGimbal {
                override fun pitchRangeDeg(): ClosedFloatingPointRange<Double>? = null
                override fun aimPitch(pitchDeg: Double) {
                    aimed += pitchDeg
                }
            },
            record = object : GuidedRecord {
                override fun stickCmd(
                    setpoint: Setpoint?, axes: StickAxes, modes: StickModes,
                    source: CommandSource?, range: StickRange?, accepted: Boolean?, error: String?,
                ) = Unit

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

        fun state() = AircraftState(
            latitude = LAT, longitude = LON,
            relativeAltitude = alt, takeoffAltitudeAmsl = DATUM,
            velocityNorth = 0.0, velocityEast = 0.0, velocityDown = 0.0,
            isFlying = flying, flightMode = mode,
            ages = SampleAges.of(
                Signal.POSITION to 0L, Signal.ALTITUDE to 0L, Signal.VELOCITY to 0L,
            ),
        )

        fun texts(): List<String> = wire.filterIsInstance<Statustext>().map { it.text() }

        /**
         * Whether a ground station exists on this flight. True is the pre-landing08 default —
         * every tick stamps MAVLink liveness, as QGC's continuous traffic does in the field.
         * False is landing08's shape: a phone-only flight, zero `mav_in` lines, nothing ever
         * heard on the wire — the state in which a MAVLINK-labelled engagement dies `link-lost`
         * and a PHONE-labelled one must not.
         */
        var gcsChattering = true

        /** One tick; with [gcsChattering] the GCS link is demonstrably alive (inbound traffic). */
        fun tick(advanceMs: Long = 100) {
            now += advanceMs
            if (gcsChattering) engine.onInbound("heartbeat")
            engine.tick(now)
        }

        /** DJI confirms an engagement the way the state listener would. */
        fun confirm() {
            port.enableOnSuccess?.invoke()
            port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "MSDK"))
            tick(40)
        }

        fun frame(x: Int = 0, y: Int = 0, z: Int = 500, r: Int = 0) {
            engine.onInbound(ManualControl.builder().target(1).x(x).y(y).z(z).r(r).buttons(0).build(), null)
        }

        /**
         * DJI's own takeoff, replayed at the shape both 2026-07-27 records show: motors, the two
         * takeoff modes, `isFlying` going true **during** `AUTO_TAKE_OFF`, then the mode returning
         * to normal. The engine is ticked throughout, so anything that fires early fires here.
         */
        fun djiTakeoff() {
            mode = "MOTOR_START"; tick(1_400)
            mode = "AUTO_TAKE_OFF"; tick(600)
            // The trap: flying, 2.4 s of DJI's climb still to run.
            flying = true; alt = 0.4; tick(600)
            alt = 0.9; tick(900)
            alt = DJI_HOP_M; tick(900)
        }

        /** The last thing DJI does: hands the aircraft back to a normal mode. */
        fun djiLetsGo() {
            mode = "GPS_ATTI"
            tick(100)
        }
    }

    // -------------------------------------------------------------- the happy path

    @Test
    fun `a QGC takeoff arms a climb and nothing at all happens until DJI lets go`() {
        val h = Harness()
        assertEquals(
            ClimbArm.Armed(QGC_MIN_TAKEOFF_M, capped = false),
            h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M),
        )
        assertTrue(h.events.any { it.first == "takeoff_climb_armed" })
        // Arming actuates nothing: no enable, no setpoint, no engagement, no sentence.
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertTrue(h.texts().isEmpty())

        h.djiTakeoff()
        // Still nothing, and this is the whole point of the class: the aircraft has been
        // reporting `isFlying` for two seconds and DJI is still flying it.
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertEquals(0, h.port.sent.size)

        h.djiLetsGo()
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        assertEquals(1, h.port.enableCalls)
        assertTrue(h.texts().contains(GuidedStatusTexts.TAKEOFF_CLIMB_ENGAGING))
        assertTrue(h.texts().contains(GuidedStatusTexts.GOTO_STARTED))
        assertTrue(h.events.any { it.first == "takeoff_climb_ended" && it.second?.startsWith("fired") == true })
        assertTrue(h.events.any { it.first == "goto_accepted" })
    }

    @Test
    fun `the climb flies straight up to the requested height and stops there`() {
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        h.djiTakeoff()
        h.djiLetsGo()
        h.confirm()
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        h.tick()
        val sent = h.port.sent.last()
        // Vertical only. A horizontal component here would mean the target was not the current
        // fix — the (0, 0) mutant, or an explicit coordinate that came from nowhere.
        assertEquals(0.0, sent.roll, 1e-9)
        assertEquals(0.0, sent.pitch, 1e-9)
        assertEquals(0.0, sent.yaw, 1e-9)
        // Up, at the law's own rate for a 1.85 m error: min(0.5·e, 1.5, sqrt(2·0.5·e)).
        val error = QGC_MIN_TAKEOFF_M - DJI_HOP_M
        assertEquals(RepositionGuidance.clampedSpeed(error, GuidedEnvelope.VERTICAL_MAX_MS), sent.verticalThrottle, 1e-9)
        assertTrue("must climb, not descend", sent.verticalThrottle > 0.0)

        // Fly it: at the target the vertical command goes to zero and arrival is declared.
        h.alt = QGC_MIN_TAKEOFF_M
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tick() }
        assertTrue(h.texts().contains(GuidedStatusTexts.GOTO_ARRIVED))
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-9)
    }

    @Test
    fun `the requested height is not put through the AMSL datum a second time`() {
        // The datum here is 100 m. A relative 3.048 m read as an AMSL would be −96.95 m relative,
        // which acceptTarget refuses as below-datum — so this test is the whole of landmine 1 for
        // the takeoff path, and it fails on a `DENIED` rather than on a number.
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        h.djiTakeoff()
        h.djiLetsGo()
        assertFalse(h.texts().any { it.startsWith("Goto refused") })
        h.confirm()
        h.tick()
        assertTrue(h.port.sent.last().verticalThrottle > 0.0)
    }

    // ------------------------------------------------------- nothing worth flying

    @Test
    fun `a request inside DJI's own hop arms nothing and never engages`() {
        val h = Harness()
        assertEquals(ClimbArm.NothingToDo, h.engine.armTakeoffClimb(2.0))
        assertFalse(h.events.any { it.first == "takeoff_climb_armed" })
        h.djiTakeoff()
        h.djiLetsGo()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertTrue(h.texts().isEmpty())
    }

    // ------------------------------------------------------------------- the ceiling

    @Test
    fun `above the ceiling the climb is capped, announced at arm time, and flown to the cap`() {
        val h = Harness()
        val armed = h.engine.armTakeoffClimb(121.92) // the top of QGC's own takeoff slider
        assertEquals(ClimbArm.Armed(GuidedEnvelope.CEILING_M, capped = true), armed)
        h.djiTakeoff()
        h.djiLetsGo()
        h.confirm()
        h.tick()
        // Flown to the ceiling, at the envelope's own vertical maximum for so large an error…
        assertEquals(GuidedEnvelope.VERTICAL_MAX_MS, h.port.sent.last().verticalThrottle, 1e-9)
        // …and the engine's own cap sentence never fires, because the target was already bound
        // before it got here. The operator's notice is the dispatcher's, at arm time.
        assertFalse(h.texts().contains(GuidedStatusTexts.GOTO_CAPPED))
    }

    // -------------------------------------------- the phone sequence's camera half

    @Test
    fun `a phone takeoff points the camera at nadir exactly when DJI lets go, and exactly once`() {
        val h = Harness()
        h.engine.armTakeoffClimb(CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M, aimCameraNadir = true)
        assertTrue(h.events.any { it.first == "takeoff_climb_armed" && it.second?.endsWith("nadir") == true })
        // Nothing touches the camera while the aircraft is on the ground or in DJI's own hop:
        // the unmeasured window (does an auto-takeoff disturb a pre-pointed gimbal?) is never
        // entered, so the commanded angle can never be a lie our own sequencing manufactured.
        h.djiTakeoff()
        assertEquals(emptyList<Double>(), h.aimed)
        h.djiLetsGo()
        assertEquals(listOf(TagDescentGuidance.NADIR_PITCH_DEG), h.aimed)
        // And the climb itself is the ordinary one, flying to 10 m.
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        repeat(30) { h.tick() }
        assertEquals(listOf(TagDescentGuidance.NADIR_PITCH_DEG), h.aimed)
    }

    /**
     * The deliberate successor of `a QGC takeoff never touches the camera - its operator owns
     * it`. Since 2026-07-29 the *dispatcher* passes the flag from both doors (the sole-operator
     * decision — `CommandDispatcherTest` pins each door's value); what this engine-side test
     * pins is the flag's semantics, unchanged: an arm that does not carry it never moves the
     * camera, whatever door it came from. The flag is the only trigger there is.
     */
    @Test
    fun `an arm without the camera flag never touches the camera - the flag is the only trigger`() {
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        h.djiTakeoff()
        h.djiLetsGo()
        h.confirm()
        repeat(10) { h.tick() }
        assertEquals(emptyList<Double>(), h.aimed)
    }

    @Test
    fun `a cancelled phone climb never aims the camera - the abort kills the whole sequence`() {
        val h = Harness()
        h.engine.armTakeoffClimb(CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M, aimCameraNadir = true)
        h.interlock = false
        h.tick()
        h.interlock = true
        h.djiTakeoff()
        h.djiLetsGo()
        assertEquals("a takeoff that was cancelled must not aim a camera", emptyList<Double>(), h.aimed)
        assertEquals(0, h.port.enableCalls)
    }

    @Test
    fun `an expired phone climb never aims the camera`() {
        val h = Harness()
        h.engine.armTakeoffClimb(CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M, aimCameraNadir = true)
        repeat(((TakeoffClimb.WAIT_LIMIT_MS / 100) + 2).toInt()) { h.tick() }
        h.djiTakeoff()
        h.djiLetsGo()
        assertEquals(emptyList<Double>(), h.aimed)
    }

    @Test
    fun `the camera is aimed even when the climb is refused by the ordinary gates`() {
        // DJI has flown the takeoff and the aircraft is airborne at ~1.2 m: the camera half of
        // the operator's ask is due, and the climb refusal is read right beside it. An aircraft
        // hovering at 1.2 m with the camera down is the sequence honestly half-delivered; one
        // hovering with the camera wherever it was is the sequence silently abandoned.
        val h = Harness()
        h.port.unavailable = "NO_PRODUCT"
        h.engine.armTakeoffClimb(CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M, aimCameraNadir = true)
        h.djiTakeoff()
        h.djiLetsGo()
        assertEquals(listOf(TagDescentGuidance.NADIR_PITCH_DEG), h.aimed)
        assertTrue(h.texts().contains(GuidedStatusTexts.gotoRefused("NO_PRODUCT")))
    }

    // ------------------------------- the commanding origin: landing08, the phone-only flight
    //
    // datasets/landing08/20260729-112216.001.jsonl, 2026-07-29 — the first flight with no QGC
    // attached, zero mav_in lines on the record. The phone Take off button's climb engaged at
    // t=32.33 and the Q4 commanding-controller watchdog released it at t=33.93
    // (guided_released link-lost): every engagement was labelled MAVLINK because no other label
    // existed, so the climb demanded a heartbeat from a ground station that was never there.
    // The aircraft hopped to DJI's 1.2 m and never climbed. These tests are that timeline, both
    // ways round: the PHONE label must keep the climb alive through total silence, and the
    // MAVLINK label must keep dying exactly as the record shows — the watchdog itself is not
    // weakened, it is aimed at the right controller.

    @Test
    fun `landing08 fixed - a phone-origin climb flies through total MAVLink silence`() {
        val h = Harness()
        h.gcsChattering = false // the whole flight: not one inbound byte, like the record
        h.engine.armTakeoffClimb(
            CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M, aimCameraNadir = true,
            origin = ControlOrigin.PHONE,
        )
        assertTrue(h.events.any { it.first == "takeoff_climb_armed" && it.second?.contains("origin=phone") == true })
        h.djiTakeoff()
        h.djiLetsGo()
        h.confirm()
        h.mode = "JOYSTICK" // what the FC reports while virtual stick owns the aircraft
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        // Landing08's climb was dead 1.6 s after engaging. Fly four times the link-lost bound
        // in continued total silence: the engagement must hold and the climb must still climb.
        repeat(((GuidedEnvelope.LINK_LOST_MS * 4) / 100).toInt()) { h.tick() }
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue("still climbing", h.port.sent.last().verticalThrottle > 0.0)
        assertTrue(h.texts().none { it.contains("link-lost") })
        assertTrue(h.events.none { it.first == "guided_released" })
        assertEquals(0, h.port.disableCalls)
    }

    @Test
    fun `landing08 regression pinned - the same climb labelled MAVLINK dies link-lost as the record shows`() {
        // The pre-2026-07-29 spelling, kept dying on purpose: a MAVLINK-labelled climb on a
        // QGC-less flight is an engagement whose commanding controller has never been heard
        // from, and the watchdog must kill it — this is the byte-for-byte heartbeat rule that
        // protects every real QGC flight, and the test that goes red if PHONE's
        // alive-by-identity reading ever leaks onto the MAVLINK label.
        val h = Harness()
        h.gcsChattering = false
        h.engine.armTakeoffClimb(CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M, aimCameraNadir = true)
        h.djiTakeoff()
        h.djiLetsGo()
        h.confirm() // t=32.33's engage; the first engaged tick is the watchdog's
        h.mode = "JOYSTICK"
        h.tick()
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick stopping: link-lost"))
        // The wind-down completes and the record carries the reason — t=33.93's line.
        repeat(30) { h.tick() }
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.events.any { it.first == "guided_released" && it.second?.contains("link-lost") == true })
    }

    @Test
    fun `a QGC that appears mid-climb and deflects takes the phone's engagement - rule 1 unweakened`() {
        // A phone-only flight that stops being phone-only: QGC connects while the phone's climb
        // is flying and the operator deflects its sticks. Rule 1's GCS half must win exactly as
        // it does over a MAVLINK manoeuvre — the deflection cancels the climb's goto, the
        // sticks' sender owns the engagement from that moment, and with it comes that sender's
        // watchdog: MAVLink silence afterwards is link loss, phone-in-hand notwithstanding.
        val h = Harness()
        h.gcsChattering = false
        h.engine.armTakeoffClimb(
            CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M, aimCameraNadir = true,
            origin = ControlOrigin.PHONE,
        )
        h.djiTakeoff()
        h.djiLetsGo()
        h.confirm()
        h.mode = "JOYSTICK"
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        // QGC arrives: a frame at rest, then a deliberate deflection.
        h.frame()
        h.now += 40
        h.frame(x = 800)
        assertTrue(h.texts().contains(GuidedStatusTexts.GOTO_STICKS))
        // The engagement is MAVLINK's now: total silence runs its armed link-loss policy.
        h.gcsChattering = false
        repeat(((GuidedEnvelope.LINK_LOST_MS + 600) / 100).toInt()) { h.tick() }
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick stopping: link-lost"))
    }

    // ------------------------------------ every rung of the abort ladder cancels it

    @Test
    fun `the interlock going off cancels a pending climb, and it does not come back`() {
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        h.interlock = false
        h.tick()
        assertTrue(h.texts().contains(GuidedStatusTexts.takeoffClimbCancelled("interlock")))
        assertTrue(h.events.any { it.first == "takeoff_climb_ended" && it.second == "cancelled interlock" })
        // The dangerous mutant: run the whole takeoff and prove nothing fires, even with the
        // interlock switched back on.
        h.interlock = true
        h.djiTakeoff()
        h.djiLetsGo()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertEquals(0, h.port.sent.size)
    }

    @Test
    fun `an RC stick grab between the phases cancels a pending climb`() {
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        // Gesture 1: past the deadband, sustained past the debounce.
        h.port.onRc!!(RcSticks(400, 0, 0, 0))
        h.tick(GuidedStickEngine.RC_ABORT_SUSTAIN_MS + 1)
        assertTrue(h.texts().contains(GuidedStatusTexts.takeoffClimbCancelled("sticks")))
        h.port.onRc!!(RcSticks(0, 0, 0, 0))
        h.djiTakeoff()
        h.djiLetsGo()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
    }

    @Test
    fun `a momentary RC nudge inside the debounce does not cancel`() {
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        h.port.onRc!!(RcSticks(400, 0, 0, 0))
        h.tick(50)
        h.port.onRc!!(RcSticks(0, 0, 0, 0))
        h.tick(50)
        assertFalse(h.texts().any { it.startsWith("Takeoff climb cancelled") })
        h.djiTakeoff()
        h.djiLetsGo()
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
    }

    @Test
    fun `a deliberate GCS deflection cancels a pending climb`() {
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        h.frame() // the stream, at rest — the gate every stick decision passes
        h.now += 40
        h.frame(x = 500)
        assertTrue(h.texts().contains(GuidedStatusTexts.takeoffClimbCancelled("sticks")))
        h.djiTakeoff()
        h.djiLetsGo()
        // The deflection engaged passthrough, which is correct and is not the climb: no reposition
        // was ever taken, so the aircraft is the operator's.
        assertFalse(h.texts().contains(GuidedStatusTexts.TAKEOFF_CLIMB_ENGAGING))
        assertFalse(h.events.any { it.first == "goto_accepted" })
    }

    @Test
    fun `a centre-zero stream cannot cancel a climb by merely existing`() {
        // The landmine the goto interrupt already guards: a −1000..1000-convention sender's *idle*
        // frame reads deliberate (z = 0 is 500 off our centre), so without the seen-at-rest gate
        // enabling that regime would silently cancel the climb of every takeoff.
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        h.frame(z = 0)
        h.frame(z = 0)
        assertFalse(h.texts().any { it.startsWith("Takeoff climb cancelled") })
        h.djiTakeoff()
        h.djiLetsGo()
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
    }

    @Test
    fun `an explicit abort cancels a pending climb even though nothing was engaged`() {
        // The IDLE early-return is exactly where this used to be lost: while DJI flies its own
        // takeoff this engine holds no authority, so `abort` has nothing else to do — and must
        // still do this.
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        h.engine.abort(GuidedStickEngine.DisengageReason.AUTHORITY, "NEAR_BOUNDARY")
        assertTrue(h.texts().contains(GuidedStatusTexts.takeoffClimbCancelled("authority")))
        h.djiTakeoff()
        h.djiLetsGo()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertEquals(0, h.port.sent.size)
    }

    @Test
    fun `stop cancels a pending climb`() {
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        h.engine.stop()
        assertTrue(h.texts().contains(GuidedStatusTexts.takeoffClimbCancelled("stopped")))
        h.djiTakeoff()
        h.djiLetsGo()
        assertEquals(0, h.port.enableCalls)
    }

    @Test
    fun `an async DJI error cancels a pending climb - a takeoff in doubt leaves nothing armed`() {
        // `SYSTEM_ERROR` shortly after a landing is a *measured* refusal on this airframe, and it
        // arrives on a channel that is not the takeoff's return path at all.
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        h.engine.cancelTakeoffClimb("SYSTEM_ERROR")
        assertTrue(h.texts().contains(GuidedStatusTexts.takeoffClimbCancelled("SYSTEM_ERROR")))
        h.djiTakeoff()
        h.djiLetsGo()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
    }

    @Test
    fun `cancelling when nothing is armed says nothing`() {
        val h = Harness()
        h.engine.cancelTakeoffClimb("SYSTEM_ERROR")
        h.engine.abort(GuidedStickEngine.DisengageReason.INTERLOCK)
        assertTrue(h.texts().none { it.startsWith("Takeoff climb cancelled") })
    }

    @Test
    fun `a tick that both revokes and completes revokes`() {
        // Cancel-before-decide, in the one tick where the two race: DJI hands the aircraft back on
        // the very tick the operator's interlock goes off. The intention loses.
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        h.djiTakeoff()
        h.interlock = false
        h.mode = "GPS_ATTI"
        h.tick()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertTrue(h.texts().contains(GuidedStatusTexts.takeoffClimbCancelled("interlock")))
        assertFalse(h.texts().contains(GuidedStatusTexts.TAKEOFF_CLIMB_ENGAGING))
    }

    // ------------------------------------------------------------------ the bound

    @Test
    fun `a takeoff the aircraft never flies expires with a sentence`() {
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        // The measured shape of a refusal we cannot hear: DJI answers the pilot with an RC beep,
        // neither callback fires, and nothing on any key ever changes.
        repeat(((TakeoffClimb.WAIT_LIMIT_MS / 100) + 2).toInt()) { h.tick() }
        assertTrue(h.texts().contains(GuidedStatusTexts.TAKEOFF_CLIMB_EXPIRED))
        assertTrue(h.events.any { it.first == "takeoff_climb_ended" && it.second == "expired" })
        // And it is gone: a takeoff that finally happens a minute later is not this one.
        h.djiTakeoff()
        h.djiLetsGo()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
    }

    @Test
    fun `the expiry sentence is said once, not on every tick`() {
        val h = Harness()
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        repeat(((TakeoffClimb.WAIT_LIMIT_MS / 100) + 20).toInt()) { h.tick() }
        assertEquals(1, h.texts().count { it == GuidedStatusTexts.TAKEOFF_CLIMB_EXPIRED })
    }

    // ------------------------------------------------- the climb is an ordinary goto

    @Test
    fun `a climb refused by the ordinary gates says why and does not retry`() {
        val h = Harness()
        h.port.unavailable = "NO_PRODUCT"
        h.engine.armTakeoffClimb(QGC_MIN_TAKEOFF_M)
        h.djiTakeoff()
        h.djiLetsGo()
        assertTrue(h.texts().contains(GuidedStatusTexts.gotoRefused("NO_PRODUCT")))
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        // No second chance from a machine that has forgotten why it was armed: the product coming
        // back does not resurrect the climb.
        h.port.unavailable = null
        repeat(10) { h.tick() }
        assertEquals(0, h.port.enableCalls)
    }

    @Test
    fun `a climb replaced by a real goto is the goto`() {
        // Nothing special: once fired, the climb is an ordinary reposition and the ordinary
        // replacement rules apply.
        val h = Harness()
        h.engine.armTakeoffClimb(50.0)
        h.djiTakeoff()
        h.djiLetsGo()
        h.confirm()
        assertEquals(
            Verdict.ACCEPTED,
            h.engine.reposition(
                RepositionCommand(
                    isCommandInt = true, frame = 0,
                    latE7 = (LAT * 1e7).toInt(), lonE7 = (LON * 1e7).toInt(),
                    zAmslM = (DATUM + 10.0).toFloat(), groundSpeedMs = -1f, yawRad = Float.NaN,
                ),
            ),
        )
        assertTrue(h.events.any { it.first == "goto_ended" && it.second == "replaced" })
    }
}
