package com.dimensional.mini4pro.telemetry

import io.dronefleet.mavlink.minimal.MavModeFlag
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The DJI→PX4 mode table.
 *
 * **Every expected `custom_mode` here is a decimal literal computed by hand**,
 * not `Px4Mode.packed(...)` re-evaluated. That is the whole point: the two
 * failure modes this table has are (a) the main/sub shifts swapped and (b) a row
 * pointing at the wrong PX4 mode, and both survive a test that asks the
 * implementation what it thinks the answer is. The arithmetic is shown in the
 * comment above each literal so a reader can redo it.
 *
 * The layout is `custom_mode = (main << 16) | (sub << 24)`
 * (`ref/qgroundcontrol/src/FirmwarePlugin/PX4/px4_custom_mode.h:39-51`), so:
 * `1 << 16 = 65536`, `1 << 24 = 16777216`.
 *
 * Mutation-checked around the 2026-07-26 restoration of `TRIPOD_GPS`/`GPS_NOVICE`
 * → `POSCTL_SLOW` — each breakage made deliberately in `Px4Mode.kt`, failing tests
 * counted, then reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | both rows put back on plain `POSCTL` (the 5.0.8-era downgrade) | 1 |
 *  | `SUB_POSCTL_SLOW = 3` instead of 2 | 3 |
 *  | `TRIPOD_GPS` → `POSCTL_ORBIT` (a precise-looking but wrong sub-mode) | 3 |
 *  | the `GPS_NOVICE` row deleted outright | 2 |
 *
 * The first is the one that matters: both `POSCTL` and `POSCTL_SLOW` are *true* of
 * Tripod, so no fact about the aircraft distinguishes them and a drift back to the
 * coarse word would read as a tidy-up. It now fails a named test instead.
 */
class Px4ModeTest {

    // ── the packed constants, hand-computed ───────────────────────────────────

    @Test
    fun packedConstantsMatchHandComputedWireValues() {
        // main 1 << 16
        assertEquals(65_536L, Px4Mode.MANUAL)
        // main 2 << 16
        assertEquals(131_072L, Px4Mode.ALTCTL)
        // main 3 << 16, sub 0
        assertEquals(196_608L, Px4Mode.POSCTL)
        // main 6 << 16
        assertEquals(393_216L, Px4Mode.OFFBOARD)
        // main 3 << 16 = 196608, sub 1 << 24 = 16777216 -> 16973824
        assertEquals(16_973_824L, Px4Mode.POSCTL_ORBIT)
        // 196608 + (2 << 24 = 33554432) -> 33751040
        assertEquals(33_751_040L, Px4Mode.POSCTL_SLOW)
        // main 4 << 16 = 262144, sub 2 << 24 = 33554432 -> 33816576
        assertEquals(33_816_576L, Px4Mode.AUTO_TAKEOFF)
        // 262144 + (3 << 24 = 50331648) -> 50593792
        assertEquals(50_593_792L, Px4Mode.AUTO_LOITER)
        // 262144 + (4 << 24 = 67108864) -> 67371008
        assertEquals(67_371_008L, Px4Mode.AUTO_MISSION)
        // 262144 + (5 << 24 = 83886080) -> 84148224
        assertEquals(84_148_224L, Px4Mode.AUTO_RTL)
        // 262144 + (6 << 24 = 100663296) -> 100925440
        assertEquals(100_925_440L, Px4Mode.AUTO_LAND)
        // 262144 + (8 << 24 = 134217728) -> 134479872
        assertEquals(134_479_872L, Px4Mode.AUTO_FOLLOW_TARGET)
    }

    /**
     * Catches the swapped-shift mistake directly: with `main` and `sub` the wrong
     * way round, `AUTO_RTL` would encode as `(5 << 16) | (4 << 24)` = 67436544,
     * which is a *valid-looking* value QGC would render as
     * `Unknown 209:67436544` — a silent mislabel, not a crash.
     */
    @Test
    fun mainAndSubBytesAreNotInterchangeable() {
        assertEquals(4, Px4Mode.mainMode(Px4Mode.AUTO_RTL))
        assertEquals(5, Px4Mode.subMode(Px4Mode.AUTO_RTL))
        assertNotEquals(Px4Mode.AUTO_RTL, Px4Mode.packed(5, 4))
        assertEquals(67_436_544L, Px4Mode.packed(5, 4))

        // The low 16 bits are `reserved` and must stay clear; anything there is a
        // private number no receiver can read.
        for (mode in Px4Mode.byDjiMode.values.toSet() + Px4Mode.UNMAPPED) {
            assertEquals("reserved bits set in $mode", 0L, mode and 0xFFFFL)
        }
    }

    /**
     * `PX4_CUSTOM_MAIN_MODE` starts at `MANUAL = 1` (`px4_custom_mode.h:5-17`), so
     * main mode 0 is not a member and cannot collide with any PX4 mode — which is
     * the entire reason [Px4Mode.UNMAPPED] is 0 rather than an invented number.
     */
    @Test
    fun unmappedIsZeroAndCollidesWithNoRealMode() {
        assertEquals(0L, Px4Mode.UNMAPPED)
        assertEquals(0, Px4Mode.mainMode(Px4Mode.UNMAPPED))
        assertFalse(Px4Mode.byDjiMode.values.contains(Px4Mode.UNMAPPED))
    }

    // ── the mapping rows ──────────────────────────────────────────────────────

    @Test
    fun returnToHome() {
        // QGC renders AUTO_RTL as "Return" (PX4FirmwarePlugin.cc:65).
        assertEquals(84_148_224L, Px4Mode.customMode("GO_HOME"))
        assertEquals(84_148_224L, Px4Mode.customMode("BACKUP_GO_HOME"))
    }

    @Test
    fun everyLandingModeIsAutoLand() {
        // QGC renders AUTO_LAND as "Land" (PX4FirmwarePlugin.cc:67).
        for (mode in listOf(
            "AUTO_LANDING", "ATTI_LANDING", "CONFIRM_LANDING",
            "BASE_LANDING", "BACKUP_LANDING",
        )) {
            assertEquals(mode, 100_925_440L, Px4Mode.customMode(mode))
        }
        // RTH is not a landing even though it ends in one — it must stay
        // distinguishable, because "Return" and "Land" are different instructions
        // to an operator deciding whether to take over.
        assertNotEquals(Px4Mode.customMode("GO_HOME"), Px4Mode.customMode("AUTO_LANDING"))
    }

    @Test
    fun everyTakeoffModeIsAutoTakeoff() {
        // QGC renders AUTO_TAKEOFF as "Takeoff" (PX4FirmwarePlugin.cc:70).
        for (mode in listOf(
            "AUTO_TAKE_OFF", "ASSISTED_TAKE_OFF", "TAKEOFF",
            "QUICKTAKEOFF_ASSIST", "PALM_LAUNCH",
        )) {
            assertEquals(mode, 33_816_576L, Px4Mode.customMode(mode))
        }
    }

    @Test
    fun waypointNavigationIsMission() {
        // QGC renders AUTO_MISSION as "Mission" (PX4FirmwarePlugin.cc:64).
        assertEquals(67_371_008L, Px4Mode.customMode("NAVI_GO"))
        assertEquals(67_371_008L, Px4Mode.customMode("NAVI_GO_NEW"))
    }

    /**
     * The M3 row. Unverified against hardware — whether virtual stick even
     * surfaces as a distinct `FCFlightMode` is open
     * (`docs/measurements/2026-07-25-ground-probe.md:99`). Asserted anyway so the
     * claim is explicit and a hardware result can contradict a named test rather
     * than a comment.
     */
    @Test
    fun virtualStickIsOffboard() {
        assertEquals(393_216L, Px4Mode.customMode("JOYSTICK"))
        assertEquals(Px4Mode.MAIN_OFFBOARD, Px4Mode.mainMode(Px4Mode.customMode("JOYSTICK")))
    }

    @Test
    fun normalGpsFlightIsPosition() {
        // QGC renders POSCTL_POSCTL as "Position" (PX4FirmwarePlugin.cc:58).
        // APAS is the only row with hardware evidence: measured parked on the
        // ground (docs/measurements/2026-07-25-key-sweep.md:47-51).
        for (mode in listOf(
            "APAS", "GPS_ATTI", "GPS_BRAKE", "GPS_HOMELOCK",
            "GPS_CL", "GPS_ATTI_WRISTBAND", "GPS_SPORT",
        )) {
            assertEquals(mode, 196_608L, Px4Mode.customMode(mode))
        }
    }

    /**
     * Sport keeps GPS position hold; PX4 MANUAL means no position *or* altitude
     * hold. Reporting MANUAL would tell an operator the aircraft drifts when the
     * sticks are centred, which is false in the dangerous direction.
     */
    @Test
    fun sportIsNotManual() {
        assertNotEquals(Px4Mode.MANUAL, Px4Mode.customMode("GPS_SPORT"))
        assertEquals(Px4Mode.customMode("GPS_ATTI"), Px4Mode.customMode("GPS_SPORT"))
    }

    /**
     * Tripod and Beginner are position control with a reduced speed envelope,
     * which is exactly PX4's POSCTL_SLOW — so that is what they send. QGC prints
     * "Position Slow" for `33_751_040` (`PX4FirmwarePlugin.cc:44`, `:60`),
     * measured against the binary in
     * `docs/measurements/2026-07-26-qgc-master-mode-sweep.md`.
     *
     * **The pin that matters is the second half.** These two rows spent
     * 2026-07-25 to 2026-07-26 downgraded to plain POSCTL, because QGC 5.0.8 had
     * no SLOW in `PX4_CUSTOM_SUB_MODE_POSCTL` and printed
     * "Unknown 81:33751040". Both mappings are *true* statements about the
     * aircraft, so nothing about the airframe can tell them apart and a silent
     * drift back to the coarse one would look like a cleanup. It is asserted as
     * a deliberate choice: reinstating the downgrade must fail this test and be
     * argued for, and the only argument is a target QGC whose
     * `_setModeEnumToModeStringMapping` lacks POSCTL_SLOW.
     */
    @Test
    fun speedLimitedModesAreReportedAsPositionSlowNotPlainPosition() {
        // 196608 + (2 << 24 = 33554432) -> 33751040, hand-computed as above.
        assertEquals(33_751_040L, Px4Mode.customMode("TRIPOD_GPS"))
        assertEquals(33_751_040L, Px4Mode.customMode("GPS_NOVICE"))
        assertEquals(Px4Mode.POSCTL_SLOW, Px4Mode.customMode("TRIPOD_GPS"))
        assertEquals(Px4Mode.POSCTL_SLOW, Px4Mode.customMode("GPS_NOVICE"))

        // Not the 5.0.8-era downgrade, and not confusable with ordinary GPS hold:
        // "Position" and "Position Slow" are different words on the toolbar.
        assertNotEquals(Px4Mode.POSCTL, Px4Mode.customMode("TRIPOD_GPS"))
        assertNotEquals(Px4Mode.POSCTL, Px4Mode.customMode("GPS_NOVICE"))
        assertNotEquals(Px4Mode.customMode("GPS_ATTI"), Px4Mode.customMode("TRIPOD_GPS"))

        // Same main mode, so it is still position control and still
        // pilot-authoritative — only the sub-mode carries the speed limit.
        assertEquals(Px4Mode.MAIN_POSCTL, Px4Mode.mainMode(Px4Mode.customMode("TRIPOD_GPS")))
        assertEquals(Px4Mode.SUB_POSCTL_SLOW, Px4Mode.subMode(Px4Mode.customMode("TRIPOD_GPS")))
    }

    @Test
    fun hotpointIsOrbit() {
        // QGC renders POSCTL_ORBIT as "Orbit" (PX4FirmwarePlugin.cc:59).
        assertEquals(16_973_824L, Px4Mode.customMode("GPS_HOTPOINT"))
    }

    /**
     * DJI ATTI holds altitude barometrically and does not hold position — which
     * is PX4 ALTCTL ("Altitude"), not STABILIZED (manual throttle, no altitude
     * hold) and emphatically not any POSCTL variant.
     *
     * RosettaDrone maps ATTI to ArduCopter LOITER
     * (`DroneModel.java:892`), and LOITER means GPS position hold. Asserting the
     * disagreement so the prior art cannot quietly be copied back in.
     */
    @Test
    fun attiFamilyIsAltitudeHoldNotPositionHold() {
        for (mode in listOf("ATTI", "ATTI_HOVER", "ATTI_CL", "ATTI_LIMITED")) {
            assertEquals(mode, 131_072L, Px4Mode.customMode(mode))
            assertNotEquals(mode, Px4Mode.POSCTL, Px4Mode.customMode(mode))
            assertNotEquals(mode, Px4Mode.packed(Px4Mode.MAIN_STABILIZED), Px4Mode.customMode(mode))
        }
    }

    @Test
    fun manualIsManual() {
        assertEquals(65_536L, Px4Mode.customMode("MANUAL"))
    }

    /**
     * Follow Me follows the operator's own controller position, which is what
     * PX4's FOLLOW_TARGET means. ActiveTrack follows a camera-selected subject
     * that may be anyone, so it must not borrow the words "Follow Me".
     *
     * QGC displays this value as "Follow Me" (`PX4FirmwarePlugin.cc:41`, `:66`;
     * measured in `docs/measurements/2026-07-26-qgc-master-mode-sweep.md`). It
     * did not on 5.0.8, which defined `AUTO_FOLLOW_TARGET` with precisely this
     * number and declared the string, then never connected them in
     * `_setModeEnumToModeStringMapping` — a QGC bug, not a disagreement about the
     * encoding, which is why the row was never changed to accommodate it. There
     * is no truthful coarser word to retreat to, and no Mini 4 Pro reaches this
     * row anyway.
     */
    @Test
    fun followMeIsMappedButVisionTrackingIsNot() {
        assertEquals(134_479_872L, Px4Mode.customMode("FOLLOW_ME"))
        assertEquals(Px4Mode.UNMAPPED, Px4Mode.customMode("ACTIVE_TRACK"))
        assertEquals(Px4Mode.UNMAPPED, Px4Mode.customMode("ACTIVE_TRACK_COURSE_LOCK"))
    }

    // ── what is deliberately absent ───────────────────────────────────────────

    /**
     * Nothing maps to AUTO_LOITER ("Hold"). DJI's hover/brake states are position
     * mode with full pilot authority, and its fly-to-a-point states are
     * translating — "Hold" would describe neither. The cost is real and is
     * recorded here so it is not rediscovered as a bug: QGC's `gotoFlightMode()`
     * and `isGuidedMode()` are both keyed on AUTO_LOITER
     * (`PX4FirmwarePlugin.cc:614`, `:634`), so a Go-to that validates by watching
     * the reported mode change to "Hold" will never validate.
     */
    @Test
    fun nothingIsReportedAsHold() {
        assertFalse(Px4Mode.byDjiMode.values.contains(Px4Mode.AUTO_LOITER))
        for (mode in listOf("GPS_BRAKE", "HOVER", "ATTI_HOVER", "CLICK_GO", "TAP_FLY")) {
            assertNotEquals(mode, Px4Mode.AUTO_LOITER, Px4Mode.customMode(mode))
        }
    }

    /**
     * PX4 TERMINATION means flight termination — outputs cut, chute deployed.
     * Nothing DJI reports means that, and it is the obvious wrong reach for the
     * degraded-control states, which is exactly why it is asserted absent.
     */
    @Test
    fun nothingIsReportedAsTermination() {
        val termination = Px4Mode.packed(Px4Mode.MAIN_TERMINATION)
        assertFalse(Px4Mode.byDjiMode.values.contains(termination))
        assertEquals(Px4Mode.UNMAPPED, Px4Mode.customMode("FAULT_TOLERANT"))
        assertEquals(Px4Mode.UNMAPPED, Px4Mode.customMode("ROLLOVER_RESCUE"))
    }

    /**
     * The canned DJI routines are autonomous but are not the operator's uploaded
     * mission, and "Mission" is the only autonomous-flight word PX4 has. With M4
     * about to make Plan-view missions literal, that confusion is not worth the
     * information.
     */
    @Test
    fun cannedRoutinesAreNotReportedAsMission() {
        for (mode in listOf(
            "CINEMATIC", "DRAW", "PANO", "QUICK_MOVIE", "TIME_LAPSE",
            "MASTER_SHOT", "DANCING", "JUMPING", "FIREFLY",
        )) {
            assertEquals(mode, Px4Mode.UNMAPPED, Px4Mode.customMode(mode))
        }
    }

    /** DJI's own "I do not know" must never acquire a name from us. */
    @Test
    fun djiUnknownAndNoReadingBothEncodeAsUnmapped() {
        assertEquals(Px4Mode.UNMAPPED, Px4Mode.customMode("UNKNOWN"))
        assertEquals(Px4Mode.UNMAPPED, Px4Mode.customMode(null))
        assertEquals(Px4Mode.UNMAPPED, Px4Mode.customMode(""))
        // A mode DJI adds in a future MSDK: unrecognised, therefore unnamed.
        assertEquals(Px4Mode.UNMAPPED, Px4Mode.customMode("SOMETHING_DJI_ADDED"))
        // Case matters — FCFlightMode.name is upper case, and a lower-case near
        // miss must not silently fall back to a mapped value by some other route.
        assertEquals(Px4Mode.UNMAPPED, Px4Mode.customMode("apas"))
    }

    // ── coverage: no mode left out by accident ────────────────────────────────

    /**
     * The full `FCFlightMode` constant list, in declaration order, from
     * `javap -cp dji-sdk-v5-aircraft-provided-5.18.0.jar
     * dji.sdk.keyvalue.value.flightcontroller.FCFlightMode`.
     *
     * **79 constants**, not the 78 quoted in some of our notes — the miscount is
     * off by one and this is the corrected list. The enum is undocumented (jar
     * only), so this literal is the only durable record of it inside the project.
     */
    private val fcFlightModeConstants = listOf(
        "MANUAL", "ATTI", "ATTI_CL", "ATTI_HOVER", "HOVER",
        "GPS_BRAKE", "GPS_ATTI", "GPS_CL", "GPS_HOMELOCK", "GPS_HOTPOINT",
        "ASSISTED_TAKE_OFF", "AUTO_TAKE_OFF", "AUTO_LANDING", "ATTI_LANDING", "NAVI_GO",
        "GO_HOME", "CLICK_GO", "JOYSTICK", "GPS_ATTI_WRISTBAND", "CINEMATIC",
        "ATTI_LIMITED", "DRAW", "FOLLOW_ME", "ACTIVE_TRACK", "TAP_FLY",
        "PANO", "FARMING", "FPV", "GPS_SPORT", "GPS_NOVICE",
        "CONFIRM_LANDING", "NOE", "GESTURE_CONTROL", "QUICK_MOVIE", "TRIPOD_GPS",
        "ACTIVE_TRACK_COURSE_LOCK", "MOTOR_START", "FIXED_WING", "APAS", "PALM_LAUNCH",
        "NAVI_GO_NEW", "TIME_LAPSE", "DANCING", "JUMPING", "ADSB_ACTION",
        "VISION_POI", "NAVI_SUBMODE_TA", "FAULT_TOLERANT", "PRE_MANUAL", "FLASHLIGHT",
        "FLASHLIGHT_SPORT", "FLASHLIGHT_ATTI", "ROLLOVER_RESCUE", "MASTER_SHOT", "TAKEOFF",
        "GO_TARGET_POINT", "FARM_WORK", "FIREFLY", "FIREFLY_EXIT", "COMMANDER_MODE",
        "BASE_LANDING", "BACKUP_GO_HOME", "BACKUP_LANDING", "TRANSPORT_DIVERT", "SPOTLIGHT_NORMAL",
        "SPOTLIGHT_TRIPOD", "SPOTLIGHT_SPORT", "TRANSPORT_HOIST_ASSIST", "QUICKTAKEOFF_ASSIST", "VISUAL_EXPLORATION",
        "FLYTO_LIVE_TARGET", "SDR_QUALITY_DETECT", "DEPARTURE_WAYLINE_TEST", "CALI_POWER_MODEL", "CABLE_FOLLOW",
        "CABLE_INSPECTION", "LOCK_YAW", "AUTO_EXPLORE", "UNKNOWN",
    )

    @Test
    fun theEnumListItselfIsWhatWeThinkItIs() {
        assertEquals(79, fcFlightModeConstants.size)
        assertEquals(79, fcFlightModeConstants.toSet().size)
    }

    /**
     * Every `FCFlightMode` constant is either mapped or explicitly reasoned about
     * as unmappable. This is the test that turns "we did not think about
     * `TRANSPORT_HOIST_ASSIST`" into a build failure rather than a silent
     * `Unknown` on an operator's toolbar.
     */
    @Test
    fun everyFcFlightModeIsEitherMappedOrDeliberatelyUnmapped() {
        val mapped = Px4Mode.byDjiMode.keys
        val unmapped = Px4Mode.deliberatelyUnmapped.keys

        val overlap = mapped intersect unmapped
        assertTrue("a mode is both mapped and declared unmapped: $overlap", overlap.isEmpty())

        val accountedFor = mapped + unmapped
        val missing = fcFlightModeConstants.toSet() - accountedFor
        assertTrue("FCFlightMode constants with no decision recorded: $missing", missing.isEmpty())

        val invented = accountedFor - fcFlightModeConstants.toSet()
        assertTrue("names in the table that are not FCFlightMode constants: $invented", invented.isEmpty())

        // 31 mapped + 48 reasoned-unmapped = 79. The counts are asserted so that
        // moving a row between the two lists is a visible edit, not a silent one.
        assertEquals(31, mapped.size)
        assertEquals(48, unmapped.size)
    }

    /** Every unmapped mode carries a reason, not an empty string. */
    @Test
    fun everyDeliberateOmissionHasAStatedReason() {
        for ((mode, reason) in Px4Mode.deliberatelyUnmapped) {
            assertTrue("no reason recorded for $mode", reason.length > 10)
        }
    }

    /** Every mapped mode lands on a `custom_mode` QGC can actually name. */
    @Test
    fun everyMappedModeIsInQgcsDisplayTable() {
        // The exact values QGC keys its display strings on
        // (PX4FirmwarePlugin.cc:50-73), hand-computed as above.
        val qgcKnows = setOf(
            65_536L, // Manual
            131_072L, // Altitude
            196_608L, // Position
            393_216L, // Offboard
            16_973_824L, // Orbit
            33_751_040L, // Position Slow
            33_816_576L, // Takeoff
            50_593_792L, // Hold
            67_371_008L, // Mission
            84_148_224L, // Return
            100_925_440L, // Land
            134_479_872L, // Follow Me
        )
        for ((dji, custom) in Px4Mode.byDjiMode) {
            assertTrue("$dji -> $custom is not in QGC's table", qgcKnows.contains(custom))
        }
    }

    // ── base_mode flags derived from the mapping ──────────────────────────────

    @Test
    fun autoModesClaimAutoAndNothingElse() {
        for (mode in listOf("GO_HOME", "AUTO_LANDING", "AUTO_TAKE_OFF", "NAVI_GO", "FOLLOW_ME")) {
            assertEquals(
                mode,
                listOf(MavModeFlag.MAV_MODE_FLAG_AUTO_ENABLED),
                Px4Mode.extraBaseModeFlags(Px4Mode.customMode(mode)),
            )
        }
    }

    @Test
    fun offboardAndOrbitClaimGuided() {
        assertEquals(
            listOf(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED),
            Px4Mode.extraBaseModeFlags(Px4Mode.customMode("JOYSTICK")),
        )
        assertEquals(
            listOf(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED),
            Px4Mode.extraBaseModeFlags(Px4Mode.customMode("GPS_HOTPOINT")),
        )
    }

    @Test
    fun pilotFlownAndUnmappedModesClaimNoAutonomy() {
        for (mode in listOf("APAS", "GPS_ATTI", "GPS_SPORT", "TRIPOD_GPS", "ATTI", "MANUAL")) {
            assertEquals(mode, emptyList<MavModeFlag>(), Px4Mode.extraBaseModeFlags(Px4Mode.customMode(mode)))
        }
        // An unknown mode claims the least capability available, never autonomy.
        for (mode in listOf("UNKNOWN", "FAULT_TOLERANT", "SOMETHING_DJI_ADDED", null)) {
            assertEquals(emptyList<MavModeFlag>(), Px4Mode.extraBaseModeFlags(Px4Mode.customMode(mode)))
        }
    }
}
