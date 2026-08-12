package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.Verdict
import com.dimensional.mini4pro.mavlink.StatusTextSink
import com.dimensional.mini4pro.record.CommandSource
import com.dimensional.mini4pro.record.EventCode
import com.dimensional.mini4pro.record.Setpoint
import com.dimensional.mini4pro.record.StickAxes
import com.dimensional.mini4pro.record.StickModes
import com.dimensional.mini4pro.record.StickRange
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import io.dronefleet.mavlink.common.Statustext
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos

/**
 * **A plan's `NAV_LAND` with "Precision Land" set, inside the engine** — the gates at the item's
 * begin, the leg to the *recorded* takeoff point, the camera, the lowering, and the hand-off to the
 * tag descent through the door the phone's arm uses.
 *
 * Design authority: Ivan, 2026-07-30, on
 * `/home/lesh/Documents/QGroundControl Daily/Missions/big1.plan` — takeoff 10 m, six waypoints to
 * 60 m, item 7 `NAV_LAND` frame 3 at 15 m, `param2 = 2` (Required), drawn 0.3 m from that plan's own
 * planned home. Every fixture here is that plan's shape unless it is deliberately broken.
 *
 * Same protocol as `GuidedMissionTest` and `GuidedTagDescentTest`: fake port, fake gimbal, fake tag
 * sense, hand-cranked clock, no aircraft. The pure arithmetic and the gate rows are pinned next door
 * in `PrecisionLandTest`; the parse in `MissionStoreTest`; the route translation in
 * `MissionLaunchTest`. **This file is about what the engine does with them.**
 *
 * The camera belief is wired the way `Bridge` wires it and that is load-bearing for this suite:
 * `FakeGimbal.aimPitch` moves the harness's believed pitch, exactly as `gimbal/PitchBelief` prefers a
 * **commanded** angle once this bridge has aimed the camera. So an ROI that keeps aiming the camera
 * really does keep the belief off nadir here, which is the deadlock one of these tests exists for.
 *
 * Written to fail loudly for:
 *
 *  - **either gate dropped at the engine level** — not only in the arithmetic: the gates must be
 *    evaluated at the moment the item begins, once, against the world as it then is.
 *  - **the reference point becoming a drawn coordinate.** Ivan corrected this feature's first design
 *    on exactly this point; the plan and the executed takeoff can differ by metres and the tag is on
 *    the pad the aircraft actually left.
 *  - **the item's altitude ignored**, so the transit flies at whatever height it had.
 *  - **the never-climb clamp dropped**, so a low Land item climbs back to 8 m.
 *  - **the arrival test bypassed**, so the arm fires while the aircraft is still moving toward the pad.
 *  - **the camera step dropped or unwaited**, so the descent is armed (or refused) with a belief nobody
 *    established — the failure mode two whole flights lost on 2026-07-28.
 *  - **an ROI left running**, which re-aims the gimbal on every tick and deadlocks the sequence.
 *  - **the arm bypassing a gate the phone's door enforces**, or a refused arm falling back to anything
 *    other than a hold.
 *  - **`param2 = 0` landing anyway** — the forward-compatibility property: every plan authored before
 *    this feature must still be flyable, as a hold, and must not land.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-30, one breakage at a time applied to the shipped `src/main`, the **whole**
 * suite run per mutant with `test-results` deleted first, confirmed red, reverted. Counts are failing
 * tests across all 2625 — **measured, not estimated**. **No survivors.** The three ROI-altitude rows
 * of the same campaign live in `GuidedRoiTest`'s own table, because that is the code they break.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the 20 m takeoff-radius gate dropped | 2 |
 *  | the below-plan-takeoff-height gate dropped | 2 |
 *  | the item-coordinate cross-check dropped | 3 |
 *  | the reference point becomes the DRAWN coordinate instead of the recorded takeoff point | 4 |
 *  | the item's altitude ignored in the transit (flies at the current Z) | 2 |
 *  | the never-climb clamp dropped (`min` → the constant) | 2 |
 *  | the arrival test bypassed (`arriveTicks` counts unsettled ticks) | 3 |
 *  | the gimbal-nadir command dropped | 8 |
 *  | the nadir wait dropped (LOWER entered without the belief) | 1 |
 *  | the ROI not cleared when the sequence begins | 1 |
 *  | the arm's `fullAutoland` dropped (the descent stops at the terminal hold) | 1 |
 *  | the descent door's fix-freshness gate weakened | 3 |
 *  | a refused arm reports the plan complete instead of pausing | 2 |
 *  | `mission` not nulled before the arm (two setpoint sources for one instant) | 3 |
 *  | `param2 == 0` lands anyway (the forward-compatibility property) | 7 |
 *
 * ### The four results worth reading rather than counting
 *
 * **The gimbal-nadir command is the most load-bearing line in the feature: 8.** Dropping
 * `gimbal.aimPitch(-90)` fails almost every test in this file, and the reason is the whole argument for
 * why the step exists at all — without it the *belief* never reaches nadir, so the sequence either
 * refuses itself at the aim bound or (with the wait also broken) hands the descent a camera whose angle
 * nothing has established. Ivan's plan makes this concrete: its ROI item leaves the camera tens of
 * degrees off nadir, and the arm's own gate would answer `CAMERA_NOT_NADIR` on a fully autonomous
 * mission. The 8 is not redundancy, it is one property every path depends on.
 *
 * **The descent door's own gate scores 3 — all of them in `GuidedTagDescentTest`, and that is the
 * point.** Weakening the arm's fix-freshness conjunct breaks the door's own suite rather than this one,
 * because there is exactly **one** door: this sequence calls [GuidedStickEngine.armTagDescent] and adds
 * no gate, skips no gate and relaxes no gate. A row that died in *both* suites would mean the arm's
 * conditions had been restated here, which is the failure the single-owner rule exists to prevent. What
 * this file pins instead is that a refusal *reaches the mission* — `THE ARM DOOR` and the stale-fix test
 * assert the hold, the pause and the named sentence, and those are the rows measured at 2.
 *
 * **`param2 == 0` lands anyway scores 7**, and five of those are older tests: `MissionStoreTest`'s
 * projection, `MissionLaunchTest`'s M4-5 hold, `MissionAdmissionTest`'s terminal-altitude rule, and
 * `MissionBig1PlanTest`'s flip. That is the forward-compatibility property being held by the whole
 * mission suite rather than by one new assertion — every plan authored before 2026-07-30 carries that
 * zero, and it must keep meaning "arrive and hover".
 *
 * **One caveat about re-measuring, found while spot-checking this table.**
 * `TagRecogniserTest::threeSightingsLatchAndTheEdgeIsReportedOnce` drives a real background thread and
 * polls with `eventually`, so under the load of a mutant run it can fail on timing alone — it did once
 * here, on a mutant that has nothing to do with the detector, and passed 3/3 in isolation and in every
 * clean full-suite run. A re-measurement that comes back one higher than a row above should check
 * **which** test failed before believing the number. Pre-existing, not this feature's, and left alone.
 *
 * **The ordering row (3) is the one that would fail silently in the air.** With `mission` still set when
 * the arm is attempted, `descentGateLocked` answers `MANOEUVRE_ACTIVE` — so the aircraft would arrive
 * over the pad at 8 m, refuse its own landing for the mission it is flying, and hover until the idle
 * window. Every symptom is a *refusal*, which is exactly the shape a reader would mistake for a gate
 * doing its job.
 *
 * ## The acquisition descent — measured 2026-07-30, the landing16 campaign
 *
 * The design above armed **once**, on arrival at [PrecisionLand.LAND_TAG_ARM_HEIGHT_M], and
 * landing16 (`datasets/landing16/20260730-161329.001.jsonl`) refuted it in the air on the day it
 * was written. Two things went wrong at once and both are now fixed separately: the detector was
 * disarmed for the whole flight (the mission takeoff never aimed the camera — `vision/TagArming`
 * and `GuidedMissionTest` carry that half), **and the arm height itself is marginal**. The video
 * says so directly: frame 4710 of `20260730-161329.v002.h264` (t ≈ 195 s) puts the tag at pixel
 * (923, 512) against `CameraCalibration`'s measured nadir point (970.7, 615.2) — 48 px and 103 px,
 * which at 8.8 m through fx = 1457 is **0.29 m and 0.62 m: the pad lay 0.69 m from directly beneath
 * the aircraft**, and the marker subtended ~11 px. The decode-rate curve is 92.6 % per frame at
 * 7–8 m and **2.7 % at 8–9 m** (`docs/measurements/2026-07-27-apriltag-c-vs-opencv.md` §1) against
 * a 400 ms freshness bound, so one attempt at 8.8 m is a coin flip with the coin weighted against.
 * (That 0.69 m is also the feature's own justification: a GPS-flown approach put the aircraft two
 * thirds of a metre off a pad it must land *on*.)
 *
 * So the sequence now **descends to acquire**: [PrecisionLand.Phase.ACQUIRE] keeps coming down to
 * [PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M], polling the descent's own arm gate every tick, and arms
 * at the first height that clears. Whole suite per mutant, **2644 tests**, `test-results` deleted
 * first, confirmed red, reverted. **No survivors** (one was found and closed — see below).
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | arming once at the nominal height only (the landing16 regression) | 6 |
 *  | the acquisition floor dropped (it descends past 5 m forever) | 5 |
 *  | the `landingOnTag` flag latches on and is never cleared at the run's end | 2 |
 *  | the arm-success height not recorded (the nominal goes on the record instead) | 1 |
 *  | the lookahead's own-mission exemption leaks onto the path that arms | 1 |
 *
 * ### The survivor, and what it taught
 *
 * **`ignoringOwnMission` defaulted to `true` killed nothing on the first sweep.** The acquisition
 * poll has to ask the gate a question it cannot otherwise ask — *"would the arm be accepted the
 * instant I end?"* — because [GuidedStickEngine] `armLandTagLocked` must null `mission` under the
 * lock before arming, and that step is destructive. The flag excuses exactly one disjunct of the
 * busy conjunct for exactly that read-only poll. Flipping its default put the exemption on the path
 * that actually arms — a descent that no longer refuses over a live manoeuvre, which is the failure
 * this whole engine is arranged around — and **not one test noticed**, because the `mission != null`
 * disjunct had never been pinned in the direction that matters. `THE ORDERING` asserts the refusal
 * is *absent* after the run ends; nothing asserted it was *present* before. Closed by `THE BUSY
 * GATE`, and re-measured at 1.
 *
 * ### The counts worth reading
 *
 * **The regression scores 6 and the floor scores 5**, which is the right shape: the acquisition
 * descent is not one line, it is a phase, and every row about what happens below the arm height
 * runs through it. The floor row is the one that would be dangerous rather than merely wrong — a
 * sequence that descends past 5 m looking for a tag is a machine flying an aircraft at the ground
 * with no sensor, and Ivan's rule for this feature is *"just error out"*.
 *
 * **The arm-success height scores 1 and is a *measurement*, not a formality.** `land_tag_phase
 * armed at=…` now carries the height the gate actually cleared at rather than the height the
 * sequence started looking from; accumulated over flights that number says where this marker's real
 * acquisition band is, and it is the evidence that would justify moving either end of the band.
 *
 * ## The hold at the floor — measured 2026-07-30, the landing17 campaign
 *
 * The acquisition descent above was right about *where* to look and wrong about *when to stop*.
 * landing17 (`datasets/landing17/20260730-172355.001.jsonl`) refuted it by 267 milliseconds:
 *
 * | t (s) | the record |
 * |---|---|
 * | 258.887 | `land_tag_phase acquiring from=8.0 to=5.0` |
 * | 262.192 | `land_tag_refused … no tag by 5m … still said TAG_NOT_IN_VIEW: newest fix 193188ms old` |
 * | **262.459** | **the flight's first sighting** — px 14.8, indicated 5.7 m, solved range 7.39 m |
 * | 262.6 → 288.5 | 133 more fixes while the aircraft sat there |
 * | 288.607 | armed by hand at 3.5 m, `fixAge=85`; the landing then worked first time |
 *
 * Three things that flight measured, all of them now in
 * [PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS]'s KDoc: the refusal fired **0.8 m above its own floor
 * and still descending at 0.4 m/s** (the arrival test's vertical band is 1 m); the barometer read
 * **5.7 m where the camera solved 7.39 m**; and the sightings that followed were *intermittent* —
 * median gap 116 ms, but 13 gaps over the 400 ms freshness bound and a worst of 2.20 s.
 *
 * So the floor became a **bounded hold**: [PrecisionLand.Phase.HOLD], the same gate asked every
 * tick for 10 s, arming at the first tick that clears and refusing by name — with the wait on the
 * record — only when the window expires.
 *
 * Whole suite per mutant, **2701 tests**, `test-results` deleted first, confirmed red, reverted.
 * **No survivors.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the hold dropped — the floor refuses on arrival again (the landing17 regression) | 5 |
 *  | the hold never expires (`LAND_TAG_ACQUIRE_HOLD_MS` = `Long.MAX_VALUE`) | 6 |
 *  | the hold does not re-ask the gate (decided once, at the door) | 2 |
 *  | the wait not recorded (`held=` on the arm and the refusal both zeroed) | 2 |
 *
 * ### What the four counts say
 *
 * **The regression scores 5 and the never-expiring hold scores 6**, and the asymmetry is the right
 * way round: refusing early loses a landing that was available, while never refusing leaves an
 * aircraft hovering over a pad it cannot see until the battery decides — so the second is the one
 * more of the suite is arranged to catch, including the tests that assert the mission *ends*.
 *
 * **The re-ask and the wait score 2 each, and both are properties nothing else would notice.** A
 * hold that decides at the door still hovers for ten seconds and still refuses by name; the only
 * observable difference is that a fix arriving at second nine is ignored, which is landing17 with
 * extra patience. The wait is a *measurement* rather than a behaviour — `held=` is what turns "it
 * armed" into "it armed 2.3 s after we stopped, at 5.4 m", which is the number the next flights
 * accumulate into this marker's real acquisition band.
 */
class GuidedPrecisionLandTest {

    private companion object {
        /** The project's home latitude family — cos 38° = 0.788, so a missing term is 21 %. */
        const val LAT = 38.0
        const val LON = 23.7
        const val DATUM = 100.0

        /** big1.plan's numbers. */
        const val PLAN_TAKEOFF_ALT = 10.0
        const val ITEM_ALT = 15.0
        const val CRUISE_ALT = 60.0

        /** How far the Land item is drawn from the pad in big1.plan. */
        const val DRAWN_OFFSET_M = 0.3

        /** The wire seq of the land item in the fixture route. */
        const val LAND_SEQ = 7

        fun latNorthOf(metres: Double): Double = LAT + metres / RepositionGuidance.METRES_PER_DEG

        fun lonEastOf(metres: Double): Double =
            LON + metres / (RepositionGuidance.METRES_PER_DEG * cos(Math.toRadians(LAT)))

        /**
         * big1.plan's shape, flattened: the takeoff item at 10 m over the pad, one waypoint at 60 m,
         * and the precision land item at [ITEM_ALT] drawn [DRAWN_OFFSET_M] north of the pad.
         */
        fun route(
            itemAltM: Double = ITEM_ALT,
            drawnNorthM: Double = DRAWN_OFFSET_M,
            mode: Int = 2,
            precision: Boolean = true,
            planTakeoff: Boolean = true,
        ): MissionRoute {
            val steps = ArrayList<MissionStep>()
            if (planTakeoff) {
                steps += MissionStep(
                    seq = 0, kind = MissionStepKind.TAKEOFF, latDeg = LAT, lonDeg = LON,
                    relAltM = PLAN_TAKEOFF_ALT, switchRadiusM = MissionGuidance.R_SWITCH_M, rest = true,
                )
            }
            steps += MissionStep(
                seq = 6, kind = MissionStepKind.WAYPOINT, latDeg = latNorthOf(8.0), lonDeg = LON,
                relAltM = CRUISE_ALT, switchRadiusM = MissionGuidance.R_SWITCH_M, rest = false,
            )
            steps += MissionStep(
                seq = LAND_SEQ,
                kind = if (precision) MissionStepKind.PRECISION_LAND else MissionStepKind.HOLD,
                latDeg = latNorthOf(drawnNorthM), lonDeg = LON,
                relAltM = if (precision) itemAltM else null,
                switchRadiusM = MissionGuidance.R_SWITCH_M, rest = true,
                precisionLandMode = if (precision) mode else 0,
            )
            return MissionRoute.of(planId = 1, steps = steps)
        }
    }

    private class FakeVirtualStickPort : VirtualStickPort {
        var unavailable: String? = null
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
            enableOnSuccess = onSuccess
        }

        override fun disable(onSuccess: () -> Unit, onFailure: (String) -> Unit) = Unit

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

    private class RecordingSink : MissionRunSink {
        val reached = mutableListOf<Int>()
        val cursors = mutableListOf<Int>()
        val finished = mutableListOf<Int>()
        val paused = mutableListOf<Pair<MissionPauseCause, Int>>()

        override fun onItemReached(seq: Int) { reached += seq }
        override fun onCursor(seq: Int) { cursors += seq }
        override fun onFinished(seq: Int) { finished += seq }
        override fun onPaused(cause: MissionPauseCause, cursorSeq: Int) { paused += cause to cursorSeq }
    }

    private class RecordedCmd(val setpoint: Setpoint?, val source: CommandSource?)

    private class Harness {
        var now = 1_000L
        var interlock = true

        /** Where the aircraft is: metres north/east of the pad, and metres above the datum. */
        var northM = 8.0
        var eastM = 0.0
        var relAlt: Double? = CRUISE_ALT
        var speed = 0.0

        /** DJI's own home point — the recorded takeoff position, and this feature's reference. */
        var homeSet: Boolean? = true
        var homeLat: Double? = LAT
        var homeLon: Double? = LON

        /**
         * How many times the engine has asked for a tag sense, and the read at which the fix is to
         * vanish — the only way to drive a wedge between the acquisition phase's **lookahead poll**
         * and the **arm** that follows it microseconds later, which in the air is an ordinary event
         * (the recogniser runs on its own thread and a fix ages by itself) and in a hand-cranked
         * harness is otherwise unreachable.
         */
        var senseReads = 0
        var fixDiesAtSenseRead: Int? = null

        // The tag sensor, as the Bridge seam reports it.
        var latchedId: Int? = 0
        var fixId: Int? = 0
        var fixNorth: Double? = 0.0
        var fixEast: Double? = 0.0
        var fixAtMs: Long? = 1_000L

        /**
         * The believed camera pitch. Starts at an **ROI-ish** angle deliberately: a mission has no
         * reason to have the camera down, and this is the value the nadir step exists to change.
         */
        var cameraPitch: Double? = -25.0

        /** When true the gimbal accepts the command and the belief never moves — a camera we cannot vouch for. */
        var gimbalDeaf = false

        val port = FakeVirtualStickPort()
        val sink = RecordingSink()
        val wire = mutableListOf<Any>()
        val cmds = mutableListOf<RecordedCmd>()
        val events = mutableListOf<Pair<String, String?>>()
        val aimed = mutableListOf<Double>()

        /** Every gimbal aim, and the altitude the aircraft was at when it happened. */
        val aimedAt = mutableListOf<Pair<Double, Double?>>()

        val gimbal = object : ManoeuvreGimbal {
            override fun pitchRangeDeg(): ClosedFloatingPointRange<Double>? = -90.0..30.0
            override fun aimPitch(pitchDeg: Double) {
                aimed += pitchDeg
                aimedAt += pitchDeg to relAlt
                // `gimbal/PitchBelief` believes a commanded angle immediately — the seam `Bridge`
                // wires, and what makes the arm's nadir gate satisfiable at all.
                if (!gimbalDeaf) cameraPitch = pitchDeg
            }
        }

        val engine = GuidedStickEngine(
            port = port,
            interlockEnabled = { interlock },
            aircraftState = { state() },
            announcer = Announcer(StatusTextSink { wire += it }),
            manoeuvreGimbal = gimbal,
            tagSense = {
                senseReads++
                val dead = fixDiesAtSenseRead?.let { senseReads >= it } == true
                TagDescentSense(
                    latched = latchedId != null,
                    latchedTagId = latchedId,
                    fixTagId = fixId,
                    fixNorthM = fixNorth,
                    fixEastM = fixEast,
                    fixAgeMs = if (dead) null else fixAtMs?.let { now - it },
                )
            },
            cameraPitchDeg = { cameraPitch },
            record = object : GuidedRecord {
                override fun stickCmd(
                    setpoint: Setpoint?, axes: StickAxes, modes: StickModes,
                    source: CommandSource?, range: StickRange?, accepted: Boolean?, error: String?,
                ) {
                    cmds += RecordedCmd(setpoint, source)
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

        fun state(): AircraftState = AircraftState(
            fcConnected = true,
            latitude = latNorthOf(northM), longitude = lonEastOf(eastM),
            relativeAltitude = relAlt, takeoffAltitudeAmsl = DATUM,
            velocityNorth = speed, velocityEast = 0.0, velocityDown = 0.0,
            yawDeg = 0.0, isFlying = true,
            homeLatitude = homeLat, homeLongitude = homeLon, homeLocationSet = homeSet,
            ages = SampleAges.of(
                Signal.POSITION to 0L, Signal.ALTITUDE to 0L,
                Signal.VELOCITY to 0L, Signal.ATTITUDE to 0L,
            ),
        )

        fun texts(): List<String> = wire.filterIsInstance<Statustext>().map { it.text() }

        fun codes(): List<String> = events.map { it.first }

        fun messagesFor(code: String): List<String> =
            events.filter { it.first == code }.mapNotNull { it.second }

        fun place(north: Double = northM, east: Double = eastM, alt: Double? = relAlt, v: Double = 0.0) {
            northM = north
            eastM = east
            relAlt = alt
            speed = v
        }

        /** The camera sees the tag over the pad, right now. */
        fun seeTag() {
            fixNorth = 0.0
            fixEast = 0.0
            fixAtMs = now
        }

        fun tickAlive(advanceMs: Long = 100) {
            now += advanceMs
            engine.onInbound("heartbeat")
            seeTagIfArmable()
            engine.tick(now)
        }

        /**
         * The height below which this marker actually decodes, or null for "always, from anywhere"
         * — the shape every fixture used before 2026-07-30 and a fiction the video corrected.
         *
         * The real curve is 92.6 % per frame at 7–8 m and **2.7 % at 8–9 m**
         * (`docs/measurements/2026-07-27-apriltag-c-vs-opencv.md` §1), and landing16 measured the
         * consequence directly: the sequence arrived at 8.8 m where the tag subtended ~11 px
         * (frame 4710 of `20260730-161329.v002.h264`, tag at pixel (923, 512) against the
         * calibrated nadir point (970.7, 615.2) — 0.29 m and 0.62 m off, so the pad lay 0.69 m
         * from directly beneath the aircraft) and held there for 14 s with nothing to arm on.
         * Setting this to 7.5 m is that flight's own shape, and it is what the acquisition descent
         * exists to survive.
         */
        var fixVisibleBelowM: Double? = null

        /** Keep the fix fresh whenever the tag would be in view — the healthy cadence. */
        private fun seeTagIfArmable() {
            if (fixAtMs == null) return
            val ceiling = fixVisibleBelowM
            val alt = relAlt
            // Above the decode height the newest fix simply stops being replaced, which is exactly
            // what the gate sees in the air: not an error, an age that grows past ARM_FRESH_MS.
            fixAtMs = if (ceiling != null && (alt == null || alt > ceiling)) {
                now - TagDescentGuidance.ARM_FRESH_MS * 25
            } else {
                now
            }
        }

        fun confirm() {
            port.enableOnSuccess?.invoke()
            port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "MSDK"))
            tickAlive(40)
        }

        /** Start the plan at its land item, in the air at cruise — the state the cursor arrives in. */
        fun startAtLandItem(route: MissionRoute = route()): Harness {
            val index = route.size - 1
            assertEquals(Verdict.ACCEPTED, engine.missionStart(route, index, false, sink))
            confirm()
            return this
        }

        /** Sit on the target and tick until the arrival predicate has held its consecutive ticks. */
        fun settleOn(north: Double, east: Double, alt: Double, extra: Int = 2) {
            place(north, east, alt, v = 0.0)
            repeat(RepositionGuidance.ARRIVE_TICKS + extra) { tickAlive() }
        }

        /**
         * Spend the whole of [PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS] at the floor, the way a
         * flight whose marker never decodes spends it — 100 ticks of 100 ms, plus a couple so the
         * strictly-greater deadline test is past.
         *
         * Every test that used to assert a refusal *on arrival* at the floor now goes through
         * here, and that is the landing17 fix stated as a test-suite property: arriving is no
         * longer an ending.
         */
        fun waitOutTheHold(extra: Int = 2) {
            repeat((PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS / 100).toInt() + extra) { tickAlive() }
        }

        /** The last setpoint the recorder saw, in our own NED frame. */
        fun lastSetpoint(): Setpoint? = cmds.lastOrNull { it.setpoint != null }?.setpoint

        fun lastSource(): String? = cmds.lastOrNull { it.source != null }?.source?.messageName
    }

    // ------------------------------------------------------------------ the happy path

    @Test
    fun `THE SEQUENCE - gates, the leg to the pad, the camera, the lowering, and the descent armed`() {
        val h = Harness().startAtLandItem()
        h.tickAlive()

        // The gates were measured at the item's begin, once, and the numbers are on the record.
        assertTrue(h.codes().contains(EventCode.LAND_TAG_BEGUN))
        assertTrue(h.texts().contains(GuidedStatusTexts.LAND_TAG_TRANSIT))
        val begun = h.messagesFor(EventCode.LAND_TAG_BEGUN).single()
        assertTrue("the record must carry the mode: $begun", begun.contains("mode=2"))
        assertTrue("the record must carry the plan's takeoff height: $begun", begun.contains("planTakeoff=10.0"))

        // The transit: still 8 m from the pad at 60 m, so it is being flown north-ish and down.
        val transit = h.lastSetpoint()!!
        assertTrue("the transit must fly toward the pad", (transit.north ?: 0.0) < -0.01)
        assertTrue("the transit must descend to the item's altitude", (transit.down ?: 0.0) > 0.01)
        assertEquals(GuidedStickEngine.LAND_TAG_SOURCE, h.lastSource())

        // Arrive at the item's altitude over the pad. The camera is commanded to nadir *there* —
        // Ivan's answer to "do we point down at 8 m or at 15": at the item's altitude, before the
        // descent, so the detector can latch on the way down.
        h.settleOn(north = 0.0, east = 0.0, alt = ITEM_ALT)
        assertTrue(h.texts().contains(GuidedStatusTexts.LAND_TAG_AIMING))
        val aim = h.aimedAt.single { it.first == TagDescentGuidance.NADIR_PITCH_DEG }
        assertEquals("the camera must be aimed at the ITEM's altitude", ITEM_ALT, aim.second!!, 0.5)
        assertTrue(h.texts().contains(GuidedStatusTexts.LAND_TAG_LOWERING))

        // The lowering: down to the arm height, at the same lat/lon, never climbing. Its first
        // setpoint belongs to its own tick — the aim tick commands zero, exactly as the mission
        // takeoff's handback tick does, so that the descent begins on a fresh look at the altitude.
        val lowering = h.messagesFor(EventCode.LAND_TAG_PHASE).single { it.startsWith("lowering") }
        assertTrue(lowering, lowering.contains("to=8.0"))
        h.tickAlive()
        assertTrue("the lowering must descend", (h.lastSetpoint()!!.down ?: 0.0) > 0.01)

        // At the arm height the mission's last item is done and the descent has it.
        h.settleOn(north = 0.0, east = 0.0, alt = PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        assertTrue(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        val armed = h.messagesFor(EventCode.TAG_DESCENT_ARMED).single()
        // Full autoland — Ivan: "trigger a full landing including the last half a metre" — and the
        // approach segment, because 8 m is above the 7 m band.
        assertTrue("the arm must carry full autoland: $armed", armed.contains("autoland"))
        assertTrue("the arm must enter the approach: $armed", armed.contains("approach"))
        assertEquals(listOf(LAND_SEQ), h.sink.reached)
        assertEquals(listOf(LAND_SEQ), h.sink.finished)
        assertTrue(h.sink.paused.isEmpty())
        assertTrue(h.texts().contains(GuidedStatusTexts.LAND_TAG_ARMED))
        // The mission's own "holding, land manually" sentence must NOT be said: this one lands.
        assertFalse(h.texts().contains(GuidedStatusTexts.MISSION_DONE_HOLDING))
    }

    // ---------------------------------------------------- the acquisition descent (landing16)

    /**
     * **The landing16 regression, at the engine.** The marker only decodes below 7.5 m — the
     * measured shape, not a convenience: 92.6 % per frame at 7–8 m against **2.7 % at 8–9 m**
     * (`docs/measurements/2026-07-27-apriltag-c-vs-opencv.md` §1), and landing16's own arrival at
     * 8.8 m put ~11 px of marker in the frame (see [Harness.fixVisibleBelowM] for the pixel
     * measurement off the video). The old design armed **once**, on arrival at the nominal 8 m, and
     * refused the whole landing when that one attempt found nothing.
     *
     * What must happen instead: the sequence keeps coming down, asking the descent's own arm gate
     * every tick, and arms the moment it clears — **below** the nominal arm height, at a height the
     * record then carries as a measurement.
     */
    @Test
    fun `THE ACQUISITION DESCENT - the sequence keeps descending until the arm gate clears`() {
        val h = Harness()
        h.fixVisibleBelowM = 7.5
        h.startAtLandItem()
        h.tickAlive()
        h.settleOn(0.0, 0.0, ITEM_ALT)
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ARM_HEIGHT_M)

        // The old design armed here. This one says so on the record and keeps going.
        assertFalse("armed at the nominal height with nothing to arm on", h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        val acquiring = h.messagesFor(EventCode.LAND_TAG_PHASE).single { it.startsWith("acquiring") }
        assertTrue(acquiring, acquiring.contains("from=8.0"))
        assertTrue(acquiring, acquiring.contains("to=5.0"))
        assertTrue(h.texts().contains(GuidedStatusTexts.LAND_TAG_ACQUIRING))
        h.tickAlive()
        assertTrue("the acquisition phase must keep descending", (h.lastSetpoint()!!.down ?: 0.0) > 0.01)

        // Down through the decode height: the gate clears and the descent takes the aircraft.
        h.settleOn(0.0, 0.0, 7.0)
        assertTrue(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        assertEquals(listOf(LAND_SEQ), h.sink.finished)
        assertTrue(h.sink.paused.isEmpty())

        // **The measured height, not the nominal one.** This is the number the next flights
        // accumulate: it says where this marker's real acquisition band is.
        val armed = h.messagesFor(EventCode.LAND_TAG_PHASE).single { it.startsWith("armed") }
        assertTrue("the arm must record the height it actually happened at: $armed", armed.contains("at=7.0"))
        assertTrue("the arm must record the band it searched: $armed", armed.contains("band=8.0-5.0"))
    }

    /**
     * **The floor is a floor, and then a wait, and only then a refusal.** Nothing ever decodes —
     * landing16's own state, where the detector was disarmed for the whole flight and no tag ever
     * latched — so the sequence descends its band, stops at
     * [PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M], **holds there for
     * [PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS] still asking**, and refuses **by name**, carrying
     * the wait it spent and the gate's own last words so the record says which conjunct was still
     * failing and for how long.
     *
     * Ivan's *"just error out"*, unchanged at the end of the window: no fallback landing, no
     * go-home, a hover the operator can act on. What must never happen is the descent continuing
     * past the floor on the hope that something appears — that is a machine flying an aircraft at
     * the ground with no sensor. **The hold is the opposite of that**: it stops the descent and
     * keeps only the looking.
     */
    @Test
    fun `THE ACQUISITION FLOOR - never acquiring ends after the hold, refused by name, holding`() {
        val h = Harness()
        h.latchedId = null // nothing latched this flight, exactly as landing16 ended up
        h.startAtLandItem()
        h.tickAlive()
        h.settleOn(0.0, 0.0, ITEM_ALT)
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        assertTrue(h.messagesFor(EventCode.LAND_TAG_PHASE).any { it.startsWith("acquiring") })

        // Down the band with nothing to find. Nothing is armed on the way.
        h.settleOn(0.0, 0.0, 6.0)
        assertFalse(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M)

        // **The floor is reached and nothing is refused yet** — this is the landing17 property.
        val holding = h.messagesFor(EventCode.LAND_TAG_PHASE).single { it.startsWith("holding") }
        assertTrue("the hold must record the height it waits at: $holding", holding.contains("floor=5.0"))
        assertTrue("the hold must record the window it was given: $holding", holding.contains("for=10000"))
        assertTrue(h.texts().contains(GuidedStatusTexts.LAND_TAG_HOLDING))
        assertTrue(
            "the floor must not refuse on arrival — landing17 refused 267 ms before the tag decoded",
            h.messagesFor(EventCode.LAND_TAG_REFUSED).isEmpty(),
        )

        // Only when the window is spent.
        h.waitOutTheHold()
        assertRefused(h, GuidedStatusTexts.REASON_LAND_NO_ACQUIRE)
        assertFalse("nothing may be armed after the floor refusal", h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        val refused = h.messagesFor(EventCode.LAND_TAG_REFUSED).last()
        assertTrue("the refusal must name the gate's last word: $refused", refused.contains(GuidedStatusTexts.REASON_LAND_NO_ACQUIRE))
        // **The wait is on the record**, because a flight that waited it all measures the band's
        // failing edge as sharply as an arm measures its working one.
        assertTrue("the refusal must carry the wait it spent: $refused", refused.contains("held 10"))
        // And the aircraft holds where it is — no setpoint driving it further down.
        h.tickAlive()
        assertEquals(0.0, h.lastSetpoint()?.down ?: 0.0, 1e-9)
    }

    /**
     * **landing17, as a regression test.** `datasets/landing17/20260730-172355.001.jsonl`:
     * `land_tag_phase acquiring from=8.0 to=5.0` at t=258.887, `land_tag_refused … no tag by 5m …
     * newest fix 193188ms old` at **t=262.192**, and the first sighting of the flight at
     * **t=262.459** — 267 ms later, px 14.8, solving the marker at 7.39 m. The tag then stayed in
     * view for 26 s while the aircraft sat there, and the landing Ivan flew by hand from that
     * position worked on the first attempt.
     *
     * So: a marker that becomes decodable *shortly after* the floor is reached must be armed on,
     * not refused past. The fixture reproduces the shape exactly — invisible for the whole
     * descent, visible a few ticks into the hold — and the arm must carry both halves of the
     * measurement: the height it happened at, and how long the wait had run.
     */
    @Test
    fun `THE HOLD - a tag that decodes just after the floor is armed on, not refused`() {
        val h = Harness()
        // The marker does not decode anywhere in the band — landing17's own state at 5.8 m, where
        // the barometer read a metre and a half under the solved range.
        h.fixVisibleBelowM = 4.0
        h.startAtLandItem()
        h.tickAlive()
        h.settleOn(0.0, 0.0, ITEM_ALT)
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M)
        assertTrue(h.messagesFor(EventCode.LAND_TAG_PHASE).any { it.startsWith("holding") })
        assertFalse(h.codes().contains(EventCode.TAG_DESCENT_ARMED))

        // A few ticks into the wait, the marker decodes — landing17's 267 ms.
        repeat(3) { h.tickAlive() }
        h.fixVisibleBelowM = 6.0
        h.tickAlive()
        h.tickAlive()

        assertTrue("the hold must arm on the first tick that clears", h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        assertTrue(h.messagesFor(EventCode.LAND_TAG_REFUSED).isEmpty())
        assertEquals(listOf(LAND_SEQ), h.sink.finished)
        val armed = h.messagesFor(EventCode.LAND_TAG_PHASE).single { it.startsWith("armed") }
        assertTrue("the arm must record the height it happened at: $armed", armed.contains("at=5.0"))
        // **The wait, recorded.** Zero would mean the arm came on the way down; a positive number
        // is the acquisition delay this feature exists to accumulate.
        val held = Regex("held=(\\d+)").find(armed)?.groupValues?.get(1)?.toLong()
        assertNotNull("the arm must record the wait: $armed", held)
        assertTrue("the wait must be the time actually spent holding: $armed", held!! in 300..1500)
    }

    /**
     * **The gate is asked on every tick of the wait, not once on entering it.** A hold that
     * decided its answer at the door would be a ten-second pause followed by the same premature
     * refusal — landing17 with extra steps.
     *
     * The marker here decodes only in the last second of the window, which is the case that
     * separates "asks again" from "asked once": a fix that arrives at 9 s must still land the
     * aircraft.
     */
    @Test
    fun `THE HOLD - the gate is re-asked every tick, and a fix at the end of the window still arms`() {
        val h = Harness()
        h.fixVisibleBelowM = 4.0
        h.startAtLandItem()
        h.tickAlive()
        h.settleOn(0.0, 0.0, ITEM_ALT)
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M)

        // Nine seconds of nothing. The sequence must still be alive and still be asking.
        repeat(90) { h.tickAlive() }
        assertTrue("the hold must not refuse early", h.messagesFor(EventCode.LAND_TAG_REFUSED).isEmpty())
        assertFalse("nothing may be armed while the gate refuses", h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        assertNotNull("the mission must still be flying its hold", h.engine.missionCursorSeq())

        h.fixVisibleBelowM = 6.0
        h.tickAlive()
        h.tickAlive()
        assertTrue(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        assertTrue(h.messagesFor(EventCode.LAND_TAG_REFUSED).isEmpty())
    }

    /**
     * **The wait is bounded, and it is bounded by its own number.** A hold that never expired
     * would leave an aircraft hovering over a pad it cannot see until the idle window or the
     * battery took the decision instead — the failure mode Ivan's *"just error out"* is written
     * against.
     *
     * Pinned against [PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS] itself rather than against a
     * literal, so the constant remains the single owner, and *timed* rather than merely observed:
     * the refusal must land inside the tick after the window, not somewhere later.
     */
    @Test
    fun `THE HOLD - the window is bounded by LAND_TAG_ACQUIRE_HOLD_MS and refuses at its end`() {
        val h = Harness()
        h.fixAtMs = null
        h.startAtLandItem()
        h.tickAlive()
        h.settleOn(0.0, 0.0, ITEM_ALT)
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        // `extra = 0` so the arrival — and with it the hold's own clock — lands on the last tick
        // of the settle, which is what makes the window measurable to a tick from here.
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M, extra = 0)
        val startedAt = h.now
        assertTrue(h.messagesFor(EventCode.LAND_TAG_PHASE).any { it.startsWith("holding") })

        // One tick short of the window: still holding.
        repeat(((PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS / 100) - 1).toInt()) { h.tickAlive() }
        assertTrue(
            "refused after ${h.now - startedAt}ms, inside the ${PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS}ms window",
            h.messagesFor(EventCode.LAND_TAG_REFUSED).isEmpty(),
        )

        // And two ticks later it is over, once and once only.
        h.tickAlive()
        h.tickAlive()
        assertEquals(1, h.messagesFor(EventCode.LAND_TAG_REFUSED).size)
        assertTrue(
            "the wait ran ${h.now - startedAt}ms, which is not the window it was given",
            h.now - startedAt <= PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS + 300,
        )
        assertRefused(h, GuidedStatusTexts.REASON_LAND_NO_ACQUIRE)
    }

    /**
     * **The hold is a hover, not a descent with the brakes off.** The one thing that must not
     * happen while the sequence waits is the aircraft continuing toward the ground: the floor is
     * the lowest this machine goes without a fix, and the wait does not spend altitude.
     *
     * The leg is still flown — landing17's arrival test fired 0.8 m high with 0.4 m/s still on the
     * aircraft, so the last of the descent belongs inside this phase — but it is flown *to the
     * floor*, and once there the vertical command is zero.
     */
    @Test
    fun `THE HOLD - the wait never descends below the floor`() {
        val h = Harness()
        h.fixAtMs = null
        h.startAtLandItem()
        h.tickAlive()
        h.settleOn(0.0, 0.0, ITEM_ALT)
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M)
        val before = h.cmds.size
        repeat(50) { h.tickAlive() }
        val vertical = h.cmds.drop(before).mapNotNull { it.setpoint?.down }
        assertTrue("the hold commanded nothing at all", vertical.isNotEmpty())
        assertTrue(
            "the hold kept descending below the floor: $vertical",
            vertical.all { it < 0.01 },
        )
        assertTrue("the hold commanded a climb: $vertical", vertical.all { it > -0.01 })
    }

    /**
     * **Never a climb to go looking.** A Land item authored below the acquisition floor — QGC's
     * altitude field is the operator's, and a low number there is a common way to say "come down" —
     * arms where it is. A fixed 5 m floor would fly the aircraft back *up* to hunt, in a sequence
     * whose whole remaining job is downward.
     */
    @Test
    fun `a land item below the acquisition floor never climbs to look`() {
        val h = Harness()
        h.startAtLandItem(route(itemAltM = 4.0))
        h.tickAlive()
        h.settleOn(0.0, 0.0, 4.0)
        val lowering = h.messagesFor(EventCode.LAND_TAG_PHASE).single { it.startsWith("lowering") }
        assertTrue(lowering, lowering.contains("to=4.0"))
        // Whether it armed at once or entered the acquisition phase, nothing may command a climb.
        repeat(6) { h.tickAlive() }
        val up = h.cmds.mapNotNull { it.setpoint?.down }.filter { it < -0.01 }
        assertTrue("the sequence commanded a climb to go looking: $up", up.isEmpty())
    }

    // ------------------------------------------------- the detector's "a landing is running" flag

    /**
     * **`landingOnTag` is the fact `vision/TagArming` reads to keep the detector alive through a
     * commanded landing** — and its lifetime is the whole property. It must be true from the moment
     * the camera is aimed (so the descent from the item's altitude is *observed*, which is what
     * makes the arm gate satisfiable when the aircraft gets there), and false again the instant the
     * landing run ends, because a flag that latched on would leave 0.68 cores burning for the rest
     * of a flight that had already given up.
     */
    @Test
    fun `LANDING ON TAG - true from the nadir aim, and false again when the sequence is refused`() {
        val h = Harness()
        h.latchedId = null
        assertFalse("nothing is landing before the sequence begins", h.engine.landingOnTag())
        h.startAtLandItem()
        h.tickAlive()
        assertFalse("the transit is not yet a commitment to the pad", h.engine.landingOnTag())

        // The camera goes to nadir: from here the aircraft is going down onto the marker.
        h.settleOn(0.0, 0.0, ITEM_ALT)
        assertTrue("the detector must be looking from the aim onward", h.engine.landingOnTag())

        // All the way down the band with nothing to find, to the refusal.
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        assertTrue(h.engine.landingOnTag())
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M)
        assertTrue("the detector must stay alive through the whole wait", h.engine.landingOnTag())
        h.waitOutTheHold()
        assertRefused(h, GuidedStatusTexts.REASON_LAND_NO_ACQUIRE)
        assertFalse("the flag must die with the landing run", h.engine.landingOnTag())
    }

    /** It survives the hand-off: the mission ends, the descent begins, and it is still a landing. */
    @Test
    fun `LANDING ON TAG - stays true across the hand-off and dies with the descent`() {
        val h = Harness().startAtLandItem()
        h.tickAlive()
        h.settleOn(0.0, 0.0, ITEM_ALT)
        assertTrue(h.engine.landingOnTag())
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        assertTrue(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        assertNull("the mission is over", h.engine.missionCursorSeq())
        assertTrue("the descent is still a tag landing", h.engine.landingOnTag())
        assertEquals(Verdict.ACCEPTED, h.engine.disarmTagDescent())
        assertFalse("a disarmed descent is not a landing", h.engine.landingOnTag())
    }

    /**
     * **A gate refusal before the camera ever moves leaves it false.** The flag is not "a precision
     * land item exists", it is "we are going down onto the marker" — and a sequence refused at its
     * own gates never was.
     */
    @Test
    fun `LANDING ON TAG - a sequence refused at the gates never arms the detector`() {
        val h = Harness()
        h.northM = PrecisionLand.LAND_TAG_RADIUS_M + 5.0
        h.startAtLandItem()
        h.tickAlive()
        assertRefused(h, GuidedStatusTexts.REASON_LAND_TOO_FAR)
        assertFalse(h.engine.landingOnTag())
    }

    /**
     * **The busy conjunct still refuses a live mission — the lookahead's exemption belongs to the
     * sequence's read-only poll and to nothing else.**
     *
     * Found as a mutation *survivor* on 2026-07-30 and closed here. The acquisition descent needs
     * to ask "would the arm be accepted the instant I end?", which it cannot do while its own
     * mission is the thing blocking it, so `descentGateLocked` gained an `ignoringOwnMission`
     * lookahead flag. Flipping that flag's default — so the path that actually *arms* also excused
     * a live mission — killed **nothing**: the `mission != null` disjunct of the busy conjunct had
     * never been pinned by a test in either direction. The existing `THE ORDERING` row asserts the
     * refusal is *absent* after the run ends; nothing asserted it was *present* before.
     *
     * That is the gate whose whole content is *"a descent does not silently cancel a manoeuvre"* —
     * two setpoint sources on one aircraft is the failure this engine is arranged around — so it
     * gets a row of its own, in the direction that matters: an operator's phone arm, mid-mission,
     * with every sensor conjunct satisfied, must be refused by name and must leave the mission
     * flying.
     */
    @Test
    fun `THE BUSY GATE - a phone arm during a live mission is refused, and the mission survives`() {
        val h = Harness().startAtLandItem()
        h.tickAlive()
        // Through the aim, so the camera conjunct passes and the busy one is what is left standing.
        h.settleOn(north = 0.0, east = 0.0, alt = ITEM_ALT)
        // Inside the approach ceiling, mid-lowering: the mission is unmistakably still flying.
        h.place(north = 0.0, east = 0.0, alt = 10.0)
        h.tickAlive()
        assertNotNull("the fixture must still have a mission to be blocked by", h.engine.missionCursorSeq())

        assertEquals(Verdict.DENIED, h.engine.armTagDescentFromPhone(fullAutoland = true))
        assertTrue(
            "expected the busy gate, got ${h.messagesFor(EventCode.TAG_DESCENT_DENIED)}",
            h.messagesFor(EventCode.TAG_DESCENT_DENIED).contains(GuidedStatusTexts.REASON_DESCENT_BUSY),
        )
        assertFalse(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        assertNotNull("a refused arm must not disturb the mission", h.engine.missionCursorSeq())
    }

    @Test
    fun `THE ORDERING - the run is ended before the arm, so there is one setpoint source at every instant`() {
        // The descent's own busy gate refuses over a live manoeuvre and is NOT relaxed for this
        // caller. The ordering is what makes the arm possible: `mission` is nulled under the lock and
        // only then is the arm attempted. A mutant that arms first sees MANOEUVRE_ACTIVE.
        val h = Harness().startAtLandItem()
        h.tickAlive()
        h.settleOn(0.0, 0.0, ITEM_ALT)
        h.settleOn(0.0, 0.0, PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        assertFalse(
            "the arm must not be refused for the mission it belongs to",
            h.messagesFor(EventCode.TAG_DESCENT_DENIED).contains(GuidedStatusTexts.REASON_DESCENT_BUSY),
        )
        assertNull("the mission cursor must be gone once the descent has it", h.engine.missionCursorSeq())
        assertTrue(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
    }

    // ------------------------------------------------------------------ the reference point

    @Test
    fun `THE REFERENCE POINT - the sequence flies to the RECORDED takeoff point, not the drawn one`() {
        // Ivan's correction, and the mutation this test exists for: "plan's lat/lng will slightly
        // differ from executed one, can be 5 m away for example". The aircraft is parked exactly on
        // the *drawn* item, 5 m north of where it actually took off — so a reference point read from
        // the plan would command nothing at all, and the recorded one commands 5 m of south.
        val h = Harness()
        h.startAtLandItem(route(drawnNorthM = 5.0))
        h.place(north = 5.0, east = 0.0, alt = ITEM_ALT)
        h.tickAlive()
        val v = h.lastSetpoint()!!
        assertTrue("the leg must fly south, toward the pad the aircraft left: $v", (v.north ?: 0.0) < -0.1)
    }

    @Test
    fun `with no recorded takeoff point the sequence is refused and the aircraft holds`() {
        // DJI's own homeLocationSet is the authority: before a home exists the coordinate is a
        // populated 4.58e7 sentinel (measured 2026-07-26), so the flag is what admits the point.
        val h = Harness()
        h.homeSet = false
        h.startAtLandItem()
        h.tickAlive()
        assertRefused(h, GuidedStatusTexts.REASON_LAND_NO_TAKEOFF_POINT)
    }

    // ------------------------------------------------------------------ the two gates

    @Test
    fun `THE 20 M GATE at the engine - too far from the pad ends the mission holding, and never arms`() {
        val h = Harness()
        h.northM = PrecisionLand.LAND_TAG_RADIUS_M + 5.0
        h.startAtLandItem()
        h.tickAlive()
        assertRefused(h, GuidedStatusTexts.REASON_LAND_TOO_FAR)
    }

    @Test
    fun `THE PLAN-HEIGHT GATE at the engine - below the plan's takeoff height ends the mission holding`() {
        val h = Harness()
        h.relAlt = PLAN_TAKEOFF_ALT - 1.0
        h.startAtLandItem()
        h.tickAlive()
        assertRefused(h, GuidedStatusTexts.REASON_LAND_TOO_LOW)
    }

    @Test
    fun `THE SITE CROSS-CHECK at the engine - a plan drawn elsewhere ends the mission holding`() {
        val h = Harness()
        h.startAtLandItem(route(drawnNorthM = 120.0))
        h.tickAlive()
        assertRefused(h, GuidedStatusTexts.REASON_LAND_SITE)
    }

    @Test
    fun `a plan with no takeoff item is refused by name rather than compared against zero`() {
        val h = Harness()
        h.startAtLandItem(route(planTakeoff = false))
        h.tickAlive()
        assertRefused(h, GuidedStatusTexts.REASON_LAND_NO_PLAN_TAKEOFF)
    }

    @Test
    fun `the gates are evaluated once, at the item's begin, not re-run per tick`() {
        // The aircraft drifts past the 20 m bound *after* the sequence has begun. That is not a fresh
        // refusal: the gates are a statement about the moment the item began, and the leg to the pad
        // is what closes the distance. Re-running them mid-sequence would abort a landing for being
        // where it started.
        val h = Harness().startAtLandItem()
        h.tickAlive()
        assertTrue(h.codes().contains(EventCode.LAND_TAG_BEGUN))
        h.place(north = PrecisionLand.LAND_TAG_RADIUS_M + 10.0, alt = CRUISE_ALT)
        repeat(5) { h.tickAlive() }
        assertFalse(h.codes().contains(EventCode.LAND_TAG_REFUSED))
        assertEquals(1, h.messagesFor(EventCode.LAND_TAG_BEGUN).size)
    }

    // ------------------------------------------------------------------ the item's altitude

    @Test
    fun `THE ITEM'S ALTITUDE - the transit is a real altitude change, and does not complete without it`() {
        // Ivan: "the way we should treat this land message height is the actual height you create a
        // waypoint to and just go there". Over the pad at 60 m, the transit must NOT be complete —
        // a mutant that ignores the item's height would settle here and aim the camera at 60 m.
        val h = Harness().startAtLandItem()
        h.settleOn(north = 0.0, east = 0.0, alt = CRUISE_ALT, extra = 10)
        assertTrue(
            "the transit must not complete 45 m above the item's altitude",
            h.aimed.none { it == TagDescentGuidance.NADIR_PITCH_DEG },
        )
        assertTrue("the vertical axis must be descending", (h.lastSetpoint()!!.down ?: 0.0) > 0.01)
        // Down at the item's altitude it completes.
        h.settleOn(north = 0.0, east = 0.0, alt = ITEM_ALT)
        assertTrue(h.aimed.contains(TagDescentGuidance.NADIR_PITCH_DEG))
    }

    @Test
    fun `THE NEVER-CLIMB CLAMP - a land item below the arm height arms where it is`() {
        // A Land item authored at 4 m: the transit ends at 4 m and the arm height is min(8, 4) = 4.
        // A mutant that drops the clamp climbs 4 m back up, above the band, for no reason anybody
        // asked for.
        val h = Harness()
        h.startAtLandItem(route(itemAltM = 4.0))
        // The sequence still *begins* at cruise, above the height the plan cleared — the low number is
        // the item's, not the aircraft's.
        h.tickAlive()
        h.settleOn(north = 0.0, east = 0.0, alt = 4.0)
        val lowering = h.messagesFor(EventCode.LAND_TAG_PHASE).single { it.startsWith("lowering") }
        assertTrue(lowering, lowering.contains("to=4.0"))
        // Nothing ever commanded a climb.
        assertTrue(
            "the sequence must never climb to reach the arm height",
            h.cmds.mapNotNull { it.setpoint?.down }.all { it >= -1e-9 },
        )
        h.settleOn(north = 0.0, east = 0.0, alt = 4.0)
        val armed = h.messagesFor(EventCode.LAND_TAG_PHASE).single { it.startsWith("armed") }
        assertTrue(armed, armed.contains("at=4.0"))
    }

    // ------------------------------------------------------------------ the arrival test

    @Test
    fun `THE ARRIVAL TEST - the arm never fires while the aircraft is still moving toward the pad`() {
        val h = Harness().startAtLandItem()
        h.tickAlive()
        // Inside the acceptance radius and at the arm height, but doing 3 m/s: a fly-through must
        // never complete a resting leg, and it must certainly never arm a landing.
        h.place(north = 1.0, east = 0.0, alt = PrecisionLand.LAND_TAG_ARM_HEIGHT_M, v = 3.0)
        repeat(RepositionGuidance.ARRIVE_TICKS * 4) { h.tickAlive() }
        assertFalse(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        assertTrue(h.sink.finished.isEmpty())
    }

    @Test
    fun `a phase that never finishes ends the mission holding, naming the timeout`() {
        val h = Harness().startAtLandItem()
        h.tickAlive()
        // Frozen 8 m from the pad: the leg cannot complete, and its own bound ends the run rather
        // than leaving an aircraft hovering over nothing until the whole-mission cap.
        repeat(400) { h.tickAlive(500) }
        assertRefusedText(h, GuidedStatusTexts.REASON_LAND_PHASE_TIMEOUT)
    }

    // ------------------------------------------------------------------ the camera

    @Test
    fun `THE CAMERA - the belief must reach nadir before the sequence descends to the arm height`() {
        val h = Harness()
        h.gimbalDeaf = true
        h.startAtLandItem()
        h.settleOn(north = 0.0, east = 0.0, alt = ITEM_ALT)
        // The command went out, and the belief never followed. The sequence does NOT descend on a
        // camera nobody can vouch for — every fix the descent would fly is trigonometry on this angle.
        assertTrue(h.aimed.contains(TagDescentGuidance.NADIR_PITCH_DEG))
        assertFalse(
            h.messagesFor(EventCode.LAND_TAG_PHASE).any { it.startsWith("lowering") },
        )
        // ...and after the bounded wait it is refused by name, holding.
        repeat((PrecisionLand.NADIR_AIM_LIMIT_MS / 100).toInt() + 2) { h.tickAlive() }
        assertRefusedText(h, GuidedStatusTexts.REASON_LAND_PHASE_TIMEOUT)
        assertFalse(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
    }

    @Test
    fun `THE ROI - an ROI still in force is cleared when the sequence begins, or the camera never gets down`() {
        // `updateRoiCameraLocked` runs on every engaged tick, so an ROI left running would re-aim the
        // gimbal at its own depression angle forever and the nadir belief would never hold. big1.plan
        // clears its ROI at item 8; a plan that forgets must not deadlock.
        val h = Harness().startAtLandItem()
        assertEquals(
            Verdict.ACCEPTED,
            h.engine.roi(
                RoiCommand(
                    command = RoiCommand.MAV_CMD_DO_SET_ROI_LOCATION, isCommandInt = true, frame = 0,
                    latE7 = (latNorthOf(30.0) * 1e7).toInt(), lonE7 = (LON * 1e7).toInt(),
                )
            ),
        )
        h.tickAlive()
        assertTrue("the operator is owed the fact that their ROI ended", h.texts().contains(GuidedStatusTexts.ROI_CLEARED))
        assertTrue(h.messagesFor(EventCode.ROI_CLEARED).any { it.contains("tag landing") })
        // And the sequence completes: the camera reaches nadir and stays there.
        h.settleOn(north = 0.0, east = 0.0, alt = ITEM_ALT)
        h.settleOn(north = 0.0, east = 0.0, alt = PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        assertEquals(
            "the last camera command must be nadir, not the ROI's angle",
            TagDescentGuidance.NADIR_PITCH_DEG, h.aimed.last(), 1e-9,
        )
        assertTrue(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
    }

    // ------------------------------------------------------------------ the arm's own gates

    /**
     * **The sequence has no privilege the operator lacks** — the property this test has always
     * been about, restated for the acquisition descent.
     *
     * Since 2026-07-30 the gate is *polled* rather than attempted, so a blocked gate is no longer
     * an event: at 8 m over a pad the answer is "no fix" many times a second and that is the
     * feature working, not a fault. What must survive is that the gate's own words reach the
     * record and the operator — once, at the floor, where they are the reason the landing did not
     * happen. The representative conjunct is the same one: nothing latched this flight, which is
     * exactly landing16's state.
     */
    @Test
    fun `THE ARM DOOR - every gate the phone's arm enforces still binds, and a refusal holds`() {
        val h = Harness()
        h.latchedId = null
        h.startAtLandItem()
        h.settleOn(north = 0.0, east = 0.0, alt = ITEM_ALT)
        h.settleOn(north = 0.0, east = 0.0, alt = PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        h.settleOn(north = 0.0, east = 0.0, alt = PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M)
        h.waitOutTheHold()
        val refused = h.messagesFor(EventCode.LAND_TAG_REFUSED).last()
        assertTrue(
            "the refusal must carry the descent gate's own word: $refused",
            refused.contains(GuidedStatusTexts.REASON_NO_TAG),
        )
        assertFalse(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        // A refused arm holds. It does NOT report the mission complete, and it does not land by any
        // other means — there is no fallback landing on any path in this feature.
        assertTrue(h.sink.finished.isEmpty())
        assertEquals(
            listOf(MissionPauseCause.LAND_TAG_REFUSED to LAND_SEQ),
            h.sink.paused,
        )
        assertTrue(
            h.texts().contains(GuidedStatusTexts.tagLandRefused(GuidedStatusTexts.REASON_LAND_NO_ACQUIRE))
        )
        assertHoldingStill(h)
    }

    /**
     * **The lookahead is a prediction; the arm is the decision.** The acquisition phase asks the
     * gate with its own mission excused ([GuidedStickEngine] `descentGateLocked`'s
     * `ignoringOwnMission`), and that answer is never allowed to *be* the arm — the run is ended
     * and [GuidedStickEngine.armTagDescent] runs every gate again, from the same single owner the
     * phone door uses. So a world that changes in the gap must produce the ordinary refusal and a
     * hold, not a landing armed on a stale prediction.
     *
     * Reachable in the air on any tick: the recogniser runs on its own thread and a fix ages by
     * itself, which is exactly the gap this fixture opens by hand.
     */
    @Test
    fun `a fix that dies between the lookahead and the arm refuses and holds, never lands`() {
        val h = Harness()
        h.startAtLandItem()
        h.settleOn(north = 0.0, east = 0.0, alt = ITEM_ALT)
        // The next sense read after the poll — the arm's own — finds nothing.
        h.fixDiesAtSenseRead = h.senseReads + 2
        h.settleOn(north = 0.0, east = 0.0, alt = PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        assertFalse("a stale prediction must not arm a landing", h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        assertTrue(
            h.messagesFor(EventCode.TAG_DESCENT_DENIED).contains(GuidedStatusTexts.REASON_TAG_STALE)
        )
        assertTrue(h.messagesFor(EventCode.LAND_TAG_REFUSED).any { it.contains("arm") })
        assertEquals(listOf(MissionPauseCause.LAND_TAG_REFUSED to LAND_SEQ), h.sink.paused)
        assertTrue("a refused arm must not report the plan complete", h.sink.finished.isEmpty())
        assertTrue(h.texts().contains(GuidedStatusTexts.tagLandRefused(GuidedStatusTexts.REASON_LAND_ARM)))
    }

    @Test
    fun `a tag pipeline that goes quiet refuses at the floor the way it refuses the phone's arm`() {
        val h = Harness()
        h.startAtLandItem()
        h.settleOn(north = 0.0, east = 0.0, alt = ITEM_ALT)
        // The tag pipeline goes quiet on the way down: there is no fix at all, at any height, so the
        // acquisition descent finds nothing anywhere in its band.
        h.fixAtMs = null
        h.settleOn(north = 0.0, east = 0.0, alt = PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
        h.settleOn(north = 0.0, east = 0.0, alt = PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M)
        h.waitOutTheHold()
        val refused = h.messagesFor(EventCode.LAND_TAG_REFUSED).last()
        assertTrue(
            "the refusal must carry the descent gate's own word: $refused",
            refused.contains(GuidedStatusTexts.REASON_TAG_STALE),
        )
        assertEquals(listOf(MissionPauseCause.LAND_TAG_REFUSED to LAND_SEQ), h.sink.paused)
        assertHoldingStill(h)
    }

    // ------------------------------------------------------------- forward compatibility

    @Test
    fun `PARAM2 == 0 - a plan that does not ask to land holds, exactly as it did before this feature`() {
        // The forward-compatibility property, and the reason "Precision Land: Disabled" is QGC's
        // default: every plan authored before 2026-07-30 carries a zero there, and none of them may
        // descend. This route's last step is the HOLD one `MissionLaunch` builds for that case.
        val h = Harness()
        h.startAtLandItem(route(precision = false))
        h.settleOn(north = DRAWN_OFFSET_M, east = 0.0, alt = CRUISE_ALT)
        assertEquals(listOf(LAND_SEQ), h.sink.finished)
        assertTrue(h.texts().contains(GuidedStatusTexts.MISSION_DONE_HOLDING))
        // Nothing of this feature happened at all.
        assertFalse(h.codes().contains(EventCode.LAND_TAG_BEGUN))
        assertFalse(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        assertTrue(h.aimed.isEmpty())
        // And it held its arrival altitude rather than descending to any of it.
        assertTrue(h.cmds.mapNotNull { it.setpoint?.down }.all { it < 0.01 })
    }

    // ------------------------------------------------------------------ the ladder above

    @Test
    fun `an RC stick grab mid-sequence ends everything - the sequence is not a second authority`() {
        val h = Harness().startAtLandItem()
        h.tickAlive()
        assertTrue(h.codes().contains(EventCode.LAND_TAG_BEGUN))
        h.port.onRc!!(RcSticks(900, 0, 0, 0))
        h.tickAlive()
        h.tickAlive()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertNull(h.engine.missionCursorSeq())
        assertFalse(h.codes().contains(EventCode.TAG_DESCENT_ARMED))
    }

    // ------------------------------------------------------------------ helpers

    /** A refusal by reason word: the record, the sentence, the pause, the hold, and no arm. */
    private fun assertRefused(h: Harness, reason: String) {
        assertTrue(
            "expected $reason on the record, got ${h.messagesFor(EventCode.LAND_TAG_REFUSED)}",
            h.messagesFor(EventCode.LAND_TAG_REFUSED).any { it.contains(reason) },
        )
        assertRefusedText(h, reason)
    }

    /** The operator-facing half of a refusal, plus the hold and the absence of any landing. */
    private fun assertRefusedText(h: Harness, reason: String) {
        assertTrue(
            "expected the sentence for $reason, got ${h.texts()}",
            h.texts().contains(GuidedStatusTexts.tagLandRefused(reason)),
        )
        assertEquals(
            listOf(MissionPauseCause.LAND_TAG_REFUSED to LAND_SEQ),
            h.sink.paused,
        )
        assertTrue("a refused sequence must not report the plan complete", h.sink.finished.isEmpty())
        assertFalse("a refused sequence must never arm a descent", h.codes().contains(EventCode.TAG_DESCENT_ARMED))
        assertHoldingStill(h)
    }

    /** The aircraft is holding where it is: still engaged, commanding nothing. */
    private fun assertHoldingStill(h: Harness) {
        repeat(3) { h.tickAlive() }
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        val last = h.port.sent.last()
        assertEquals(0.0, last.pitch, 1e-9)
        assertEquals(0.0, last.roll, 1e-9)
        assertEquals(0.0, last.verticalThrottle, 1e-9)
    }
}
