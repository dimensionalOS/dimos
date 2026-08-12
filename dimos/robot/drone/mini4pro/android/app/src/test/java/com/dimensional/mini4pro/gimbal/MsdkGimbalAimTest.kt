package com.dimensional.mini4pro.gimbal

import com.dimensional.mini4pro.command.ActionOutcome
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The DJI half of the gimbal, driven through a fake [GimbalPort] — every decision
 * [MsdkGimbalAim] makes about a real camera, made here against a recorded one.
 *
 * The suite is written to fail loudly for the mistakes that would leave an operator with a camera
 * that does not move and no idea why:
 *
 *  - **populating the yaw or roll axis of a `GimbalAngleRotation`**, which on this airframe is
 *    answered `SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW` and nothing happens
 *    ([#527](https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/527)). Several tests exist
 *    only for this, because the failure is invisible from the code: DJI's serialiser turns a null
 *    `Double` into `0.0`, so an axis is suppressed by its `*Ignored` flag or not at all
 *  - touching `KeyManager` while the SDK is unregistered or no product is connected
 *  - reporting a requested angle as if it were a measured one
 *  - **going quiet about a gimbal that is merely holding still.** `KeyGimbalAttitude` is
 *    change-driven, so a stabilised camera stops delivering; treating that as a dead feed cost
 *    QGroundControl its gimbal at connect time, measured 2026-07-26
 *  - **and the opposite**: describing a gimbal on an aircraft that is no longer there, or letting
 *    an angle measured on one link reappear on the next one
 *  - paraphrasing DJI's error name on its way to the operator
 *
 * Mutation-checked 2026-07-26 — each breakage was applied to the source, the whole gimbal suite
 * run, the failing tests counted, and the source restored. **Every one was caught.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `rollDeg = null` → `rollDeg = 0.0` in `aimPitch` | 2 |
 *  | `yawDeg = null` → `yawDeg = 0.0` in `aimPitch` | 2 |
 *  | `absolute = true` → `false` | 1 |
 *  | pitch negated on the way to the port | 6 |
 *  | availability check removed from `aimPitch` | 2 |
 *  | capability check removed from `aimPitch` | 1 |
 *  | capability refusal reported as `Refused` rather than `Unavailable` | 1 |
 *  | clamp dropped (request passed through unclamped) | 1 |
 *  | clamp invents a ±90/60 range when DJI reported none | 2 |
 *  | `ensureListening` no longer waits for `unavailableReason() == null` | 1 |
 *  | `ensureListening` removed from `reading()` | 11 |
 *  | attitude delivery stamped on *change* rather than on delivery | 1 |
 *  | `stop()` stops cancelling the subscriptions | 1 |
 *  | non-finite pitch passed through to the port | 1 |
 *
 * Re-run the same way that evening, when the aircraft refused the keep-fresh `get` and liveness
 * moved off this key onto DJI's aircraft-link state. Run across `MsdkGimbalAimTest`,
 * `GimbalEncoderTest` and `GimbalManagerTest` together, since the rule now spans them. **Every one
 * was caught.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the aircraft-link condition dropped from `isAdvertisable` | 2 |
 *  | the component-gone condition dropped from `isAdvertisable` | 2 |
 *  | a never-delivered attitude counts as advertisable | 6 |
 *  | link loss no longer forgets the reading | 2 |
 *  | link loss forgets but keeps the subscriptions | 2 |
 *  | the reading always claims a link | 2 |
 *  | `onLinkLost` re-forgets on every tick | 1 |
 *  | a delivery no longer stamps the age | 4 |
 */
class MsdkGimbalAimTest {

    // ------------------------------------------------------------------- rig

    /** One recorded `KeyRotateByAngle` call, in the port's own vocabulary. */
    private data class Rotation(
        val absolute: Boolean,
        val pitchDeg: Double?,
        val rollDeg: Double?,
        val yawDeg: Double?,
        val durationS: Double,
    )

    private class FakePort : GimbalPort {
        var unavailable: String? = null
        var performable = true
        var cancelled = 0
        val rotations = mutableListOf<Rotation>()

        /** The port stack's commanded record, as `CommandedGimbalPort` would surface it. */
        var commanded: Double? = null

        override fun commandedPitchDeg(): Double? = commanded

        var attitude: ((GimbalAngles?) -> Unit)? = null
        var range: ((GimbalLimits?) -> Unit)? = null
        var mode: ((String?) -> Unit)? = null
        var connection: ((Boolean?) -> Unit)? = null

        /** What the next rotation's callback should do, if anything. */
        var onNextRotation: ((onSuccess: () -> Unit, onFailure: (String) -> Unit) -> Unit)? = null

        override fun unavailableReason(): String? = unavailable

        override fun canRotateByAngle(): Boolean = performable

        override fun rotateByAngle(
            absolute: Boolean,
            pitchDeg: Double?,
            rollDeg: Double?,
            yawDeg: Double?,
            durationS: Double,
            onSuccess: () -> Unit,
            onFailure: (String) -> Unit,
        ) {
            rotations.add(Rotation(absolute, pitchDeg, rollDeg, yawDeg, durationS))
            onNextRotation?.invoke(onSuccess, onFailure)
        }

        var attitudeSubscriptions = 0

        override fun listenAttitude(onDelivery: (GimbalAngles?) -> Unit) {
            attitudeSubscriptions++
            attitude = onDelivery
        }

        override fun listenAttitudeRange(onDelivery: (GimbalLimits?) -> Unit) { range = onDelivery }
        override fun listenWorkMode(onDelivery: (String?) -> Unit) { mode = onDelivery }
        override fun listenConnection(onDelivery: (Boolean?) -> Unit) { connection = onDelivery }

        override fun cancelListens() {
            cancelled++
            attitude = null
            range = null
            mode = null
            connection = null
        }

        val subscribed: Boolean get() = attitude != null
    }

    private val port = FakePort()
    private val djiErrors = mutableListOf<String>()
    private val logs = mutableListOf<String>()

    /** Injected monotonic clock. Nothing in this suite sleeps; time is a variable. */
    private var nowMs = 0L

    private val aim = MsdkGimbalAim(
        port = port,
        reportAsyncDjiError = { djiErrors.add(it) },
        log = { logs.add(it) },
        nowMs = { nowMs },
    )

    // --------------------------------------------------------- the #527 handling

    @Test
    fun `a pitch command never populates the yaw axis`() {
        aim.aimPitch(-90.0)

        val rotation = port.rotations.single()
        assertNull(
            "a yaw of 0.0 is what makes this airframe answer " +
                "SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW, and DJI's serialiser writes 0.0 for a " +
                "null — so the axis must be null here and suppressed by yawIgnored at the port",
            rotation.yawDeg,
        )
    }

    @Test
    fun `a pitch command never populates the roll axis`() {
        aim.aimPitch(-90.0)
        assertNull(port.rotations.single().rollDeg)
    }

    @Test
    fun `no requested angle, however unusual, can put a number on yaw or roll`() {
        // The failure this guards against is a future "improvement" that reads #527's title as
        // "never send exactly 0.0" and starts nudging the axes. Every angle here goes through the
        // same unconditional nulls.
        listOf(-90.0, -45.0, 0.0, 0.001, 30.0, 60.0, 89.9).forEach { aim.aimPitch(it) }

        assertEquals(7, port.rotations.size)
        assertTrue(port.rotations.all { it.yawDeg == null })
        assertTrue(port.rotations.all { it.rollDeg == null })
    }

    @Test
    fun `the pitch axis is the one axis that is populated`() {
        aim.aimPitch(-45.0)
        assertEquals(-45.0, port.rotations.single().pitchDeg!!, 0.0)
    }

    // ------------------------------------------------------------ the command itself

    @Test
    fun `the rotation is absolute, because a MAVLink pitch is an absolute angle`() {
        // MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW's param1 is "pitch angle", not a delta. A relative
        // rotation would make QGC's Tilt-90 button tilt a further 90° from wherever it already is,
        // every press, until the gimbal hit its stop.
        aim.aimPitch(-90.0)
        assertTrue(port.rotations.single().absolute)
    }

    @Test
    fun `negative is down, with no sign flip anywhere`() {
        // DJI: "positive is rotated upward, negative downward". QGC's Tilt 90 sends -90 and its
        // Center sends 0. The conventions agree, so the correct amount of conversion is none —
        // and a helpful inversion added later would point the camera at the sky.
        aim.aimPitch(-90.0)
        aim.aimPitch(0.0)
        aim.aimPitch(30.0)
        assertEquals(listOf(-90.0, 0.0, 30.0), port.rotations.map { it.pitchDeg })
    }

    @Test
    fun `every rotation carries the documented duration`() {
        aim.aimPitch(-30.0)
        assertEquals(MsdkGimbalAim.ROTATION_DURATION_S, port.rotations.single().durationS, 0.0)
    }

    @Test
    fun `a dispatched aim reports Requested, never a success`() {
        assertTrue(aim.aimPitch(-30.0) is ActionOutcome.Requested)
    }

    // ----------------------------------------------------------------- refusals

    @Test
    fun `nothing is asked of DJI while the SDK is unregistered`() {
        port.unavailable = "SDK_NOT_REGISTERED"

        val outcome = aim.aimPitch(-30.0)

        assertEquals("SDK_NOT_REGISTERED", (outcome as ActionOutcome.Unavailable).reason)
        assertTrue(port.rotations.isEmpty())
        assertFalse("and no subscription is made either", port.subscribed)
    }

    @Test
    fun `nothing is asked of DJI while no product is connected`() {
        port.unavailable = "NO_PRODUCT"
        assertEquals("NO_PRODUCT", (aim.aimPitch(-30.0) as ActionOutcome.Unavailable).reason)
        assertTrue(port.rotations.isEmpty())
    }

    @Test
    fun `an unperformable key fails closed with DJI's own word for the flag`() {
        port.performable = false

        val outcome = aim.aimPitch(-30.0)

        // Unavailable, not Refused: canPerformAction is a constant baked into the key in our own
        // process, so no gimbal was consulted and no decision may be attributed to one.
        assertEquals(MsdkGimbalAim.CANNOT_PERFORM_ACTION, (outcome as ActionOutcome.Unavailable).reason)
        assertTrue(port.rotations.isEmpty())
        assertFalse("an unperformable gimbal leaves no subscription behind", port.subscribed)
    }

    @Test
    fun `a non-finite pitch never reaches DJI`() {
        listOf(Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY).forEach {
            assertEquals(
                MsdkGimbalAim.PITCH_NOT_FINITE,
                (aim.aimPitch(it) as ActionOutcome.Unavailable).reason,
            )
        }
        assertTrue(port.rotations.isEmpty())
    }

    @Test
    fun `DJI's error name reaches the operator verbatim`() {
        port.onNextRotation = { _, onFailure -> onFailure("SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW") }

        aim.aimPitch(-90.0)

        assertEquals(listOf("SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW"), djiErrors)
    }

    @Test
    fun `an accepted rotation says nothing to the operator`() {
        // The gimbal's confirmation is the attitude feed, five times a second, from the aircraft.
        // Unlike Return and Land there is no silent-channel problem to compensate for.
        port.onNextRotation = { onSuccess, _ -> onSuccess() }
        aim.aimPitch(-90.0)
        assertTrue(djiErrors.isEmpty())
    }

    // ------------------------------------------------------------------ clamping

    @Test
    fun `a request outside DJI's reported range is clamped to it`() {
        deliverLimits(GimbalLimits(pitchMinDeg = -90.0, pitchMaxDeg = 60.0))

        aim.aimPitch(-140.0)
        aim.aimPitch(120.0)

        assertEquals(listOf(-90.0, 60.0), port.rotations.map { it.pitchDeg })
    }

    @Test
    fun `a request inside DJI's reported range is untouched`() {
        deliverLimits(GimbalLimits(pitchMinDeg = -90.0, pitchMaxDeg = 60.0))
        aim.aimPitch(-33.5)
        assertEquals(-33.5, port.rotations.single().pitchDeg!!, 0.0)
    }

    @Test
    fun `with no reported range nothing is clamped and DJI gets to refuse for itself`() {
        // The alternative is inventing an envelope. DJI's own refusal names the reason; our guess
        // at the limits would silently produce a different angle from the one asked for.
        aim.aimPitch(-140.0)
        assertEquals(-140.0, port.rotations.single().pitchDeg!!, 0.0)
    }

    @Test
    fun `a half-reported range clamps nothing`() {
        deliverLimits(GimbalLimits(pitchMinDeg = -90.0, pitchMaxDeg = null))
        aim.aimPitch(-140.0)
        assertEquals(-140.0, port.rotations.single().pitchDeg!!, 0.0)
    }

    // ------------------------------------------------------------------ readings

    @Test
    fun `nothing is reported before DJI has said anything`() {
        val reading = aim.reading()
        assertNull(reading.pitchDeg)
        assertNull(reading.attitudeAgeMs)
        assertFalse(reading.isAdvertisable())
    }

    @Test
    fun `the reported attitude is DJI's, not the one we asked for`() {
        subscribe()
        port.attitude!!(GimbalAngles(pitchDeg = -12.0, rollDeg = 0.5, yawDeg = 100.0))

        // Ask for something completely different and check the report does not move.
        aim.aimPitch(-90.0)

        val reading = aim.reading()
        assertEquals(-12.0, reading.pitchDeg!!, 0.0)
        assertEquals(0.5, reading.rollDeg!!, 0.0)
        assertEquals(100.0, reading.yawDeg!!, 0.0)
    }

    @Test
    fun `age is stamped on every delivery, including an unchanged one`() {
        subscribe()
        port.attitude!!(GimbalAngles(pitchDeg = -12.0))
        nowMs += 4_000
        assertEquals(4_000L, aim.reading().attitudeAgeMs)

        // DJI re-sends unchanged values, and a re-send is a live feed rather than a stale one.
        port.attitude!!(GimbalAngles(pitchDeg = -12.0))
        assertEquals(0L, aim.reading().attitudeAgeMs)
        assertTrue(aim.reading().isAdvertisable())
    }

    @Test
    fun `an age is reported but never used to withdraw the advertisement`() {
        // This assertion is the inverse of the one that used to be here, and the reversal is the
        // whole 2026-07-26 lesson: on a change-driven key, a large age means "it has not moved".
        // The age is still reported, because it is true and it is worth logging.
        subscribe()
        port.attitude!!(GimbalAngles(pitchDeg = -12.0))
        nowMs += 60_000

        val reading = aim.reading()
        assertEquals(60_000L, reading.attitudeAgeMs)
        assertEquals(-12.0, reading.pitchDeg!!, 0.0)
        assertTrue("age alone withdraws nothing", reading.isAdvertisable())
    }

    @Test
    fun `a null attitude delivery is DJI saying the component is gone`() {
        subscribe()
        port.attitude!!(GimbalAngles(pitchDeg = -12.0))
        port.attitude!!(null)

        val reading = aim.reading()
        assertNull(reading.pitchDeg)
        assertFalse(reading.isAdvertisable())
        // Still a fresh fact: the delivery happened, it just carried nothing.
        assertEquals(0L, reading.attitudeAgeMs)
    }

    @Test
    fun `the work mode crosses the seam as DJI's own name`() {
        subscribe()
        port.mode!!("YAW_FOLLOW")
        assertEquals("YAW_FOLLOW", aim.reading().workMode)
    }

    @Test
    fun `a disconnect is reported`() {
        subscribe()
        port.attitude!!(GimbalAngles(pitchDeg = -12.0))
        port.connection!!(false)
        assertFalse(aim.reading().isAdvertisable())
    }

    // ------------------------------------------------------------- subscriptions

    @Test
    fun `KeyManager is never touched before registration completes`() {
        // Subscribing before registration silently does nothing — no error, no callback — so an
        // early subscription is indistinguishable from a gimbal that never reports.
        port.unavailable = "SDK_NOT_REGISTERED"
        aim.reading()
        assertFalse(port.subscribed)

        port.unavailable = null
        aim.reading()
        assertTrue(port.subscribed)
    }

    @Test
    fun `the subscription is made once, from whichever call comes first`() {
        repeat(20) { aim.reading() }
        aim.aimPitch(-30.0)
        assertEquals(1, port.attitudeSubscriptions)
    }

    @Test
    fun `stopping cancels the subscriptions and forgets everything DJI said`() {
        subscribe()
        port.attitude!!(GimbalAngles(pitchDeg = -12.0))
        port.connection!!(true)

        aim.stop()

        assertEquals(1, port.cancelled)
        val reading = aim.reading()
        assertNull(reading.pitchDeg)
        assertNull(reading.connected)
        assertFalse(reading.isAdvertisable())
    }

    // ------------------------------------------- a change-driven key and its liveness
    //
    // Measured twice on 2026-07-26, and both measurements are in these tests. First:
    // `KeyGimbalAttitude` only delivers on change, so a motionless gimbal stopped delivering for
    // six minutes, the age-based staleness rule withdrew the advertisement, and QGC's first six
    // discovery probes were answered UNSUPPORTED (2026-07-26-gimbal-first-aim.md). Then: the
    // keep-fresh `get` written to fix that was refused by the aircraft with
    // REQUEST_HANDLER_NOT_FOUND, every time (2026-07-26-gimbal-keep-fresh-get.md).
    //
    // So liveness cannot come from this key at all, and these tests hold the replacement in place:
    // it comes from DJI's aircraft-link state, and a reading belongs to the link it was measured
    // on.

    @Test
    fun `a gimbal that never moves again stays advertisable`() {
        // The bug, in one test. A stabilised camera delivers nothing, forever, and that is not a
        // reason to tell QGroundControl there is no gimbal.
        subscribe()
        port.attitude!!(GimbalAngles(pitchDeg = -12.0))
        port.connection!!(true)

        nowMs += 10 * 60_000

        val reading = aim.reading()
        assertTrue("ten minutes of stillness is not a dead gimbal", reading.isAdvertisable())
        assertEquals(-12.0, reading.pitchDeg!!, 0.0)
        assertEquals("and the age is still reported honestly", 600_000L, reading.attitudeAgeMs)
    }

    @Test
    fun `no aircraft link means no advertisement, however recent the angle`() {
        subscribe()
        port.attitude!!(GimbalAngles(pitchDeg = -12.0))
        assertTrue(aim.reading().isAdvertisable())

        port.unavailable = "NO_PRODUCT"

        val reading = aim.reading()
        assertFalse(reading.isAdvertisable())
        assertEquals(false, reading.aircraftLinked)
    }

    @Test
    fun `an angle measured on one link cannot outlive it`() {
        // Without this, an aircraft unplugged and replaced would inherit the last aircraft's
        // camera angle the instant it connected — and with no age gate left, inherit it
        // indefinitely.
        subscribe()
        port.attitude!!(GimbalAngles(pitchDeg = -12.0))
        port.connection!!(true)

        port.unavailable = "NO_PRODUCT"
        aim.reading()
        port.unavailable = null

        val reading = aim.reading()
        assertNull("the angle went with the link", reading.pitchDeg)
        assertNull(reading.connected)
        assertFalse(reading.isAdvertisable())
        assertEquals("and the reading says the link is back", true, reading.aircraftLinked)
    }

    @Test
    fun `losing the link drops the subscriptions, and regaining it makes new ones`() {
        // The re-subscription is the point: getOnce = true is the only thing that can produce an
        // attitude before the gimbal next moves, so a reconnect must ask for it again.
        subscribe()
        assertEquals(1, port.attitudeSubscriptions)

        port.unavailable = "NO_PRODUCT"
        aim.reading()
        assertEquals(1, port.cancelled)
        assertFalse(port.subscribed)

        // Still gone, and we do not cancel a second time for the same loss.
        aim.reading()
        assertEquals(1, port.cancelled)

        port.unavailable = null
        aim.reading()
        assertEquals(2, port.attitudeSubscriptions)
    }

    @Test
    fun `the reading carries the link state DJI reported at that instant`() {
        assertEquals(true, aim.reading().aircraftLinked)

        port.unavailable = "SDK_NOT_REGISTERED"
        assertEquals(false, aim.reading().aircraftLinked)
    }

    @Test
    fun `nothing is advertised before the first delivery, link or no link`() {
        // A link is not a gimbal. Until DJI says where the camera points there is no honest
        // quaternion to put in GIMBAL_DEVICE_ATTITUDE_STATUS.
        val reading = aim.reading()
        assertEquals(true, reading.aircraftLinked)
        assertNull(reading.pitchDeg)
        assertFalse(reading.isAdvertisable())
    }

    // ---------------------------------------------------------- the believed pitch

    @Test
    fun `believedPitch prefers the port's commanded record and says it was not reported`() {
        subscribe()
        port.attitude!!(GimbalAngles(pitchDeg = -33.0))
        port.commanded = -90.0
        assertEquals(PitchBelief(-90.0, reported = false), aim.believedPitch())
    }

    @Test
    fun `with nothing commanded believedPitch falls back to the delivered attitude and says so`() {
        subscribe()
        port.attitude!!(GimbalAngles(pitchDeg = -90.0))
        assertEquals(PitchBelief(-90.0, reported = true), aim.believedPitch())
    }

    @Test
    fun `with neither source believedPitch is null - never a zero`() {
        subscribe()
        assertNull(aim.believedPitch())
    }

    @Test
    fun `the real port chain carries an accepted rotate into the belief`() {
        // End to end inside the JVM: MsdkGimbalAim over a real CommandedGimbalPort over the fake
        // DJI port, which is exactly Bridge's stack minus the recorder. The commanded half of the
        // belief must be the success-stamped record — an ask DJI refuses must not be believed.
        val chained = MsdkGimbalAim(
            port = CommandedGimbalPort(port),
            reportAsyncDjiError = { djiErrors.add(it) },
            nowMs = { nowMs },
        )
        port.onNextRotation = { _, onFailure -> onFailure("SDK_SERVICE_GIMBAL_BUSY") }
        chained.aimPitch(-90.0)
        assertNull("a refused rotate is not a belief", chained.believedPitch())

        port.onNextRotation = { onSuccess, _ -> onSuccess() }
        chained.aimPitch(-90.0)
        assertEquals(PitchBelief(-90.0, reported = false), chained.believedPitch())
    }

    @Test
    fun `link loss forgets the reported belief but not the commanded record`() {
        // The reported angle is a measurement and belongs to the link it was made on; the
        // commanded record is this bridge's own act and survives until Bridge.stop() drops the
        // whole stack.
        subscribe()
        port.attitude!!(GimbalAngles(pitchDeg = -33.0))
        port.unavailable = "NO_PRODUCT"
        aim.reading() // the tick that notices the link is gone
        assertNull(aim.believedPitch())

        port.commanded = -90.0
        assertEquals(PitchBelief(-90.0, reported = false), aim.believedPitch())
    }

    // ------------------------------------------------------------------ helpers

    /** Gets past the lazy subscription, the way the telemetry tick does. */
    private fun subscribe() {
        aim.reading()
    }


    private fun deliverLimits(limits: GimbalLimits) {
        subscribe()
        port.range!!(limits)
    }
}
