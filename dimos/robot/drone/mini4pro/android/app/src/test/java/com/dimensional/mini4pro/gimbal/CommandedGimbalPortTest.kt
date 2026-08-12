package com.dimensional.mini4pro.gimbal

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **The commanded camera angle, which is the only one `vision/` is allowed to see.**
 *
 * `KeyGimbalAttitude` is change-driven: hold the camera still — the whole of a nadir approach — and
 * DJI stops delivering it. [GimbalReading]'s own KDoc records that this was measured wrong twice in
 * one day and that six minutes of a flight record contain no attitude at all while the camera was in
 * perfect health. So a detector asking *"is the camera pointing down right now?"* off the reading
 * gets "I do not know" precisely when the answer is "yes, and it has been for a while".
 *
 * ## What is pinned here
 *
 * The four rules this decorator has, each with its own test: the angle is remembered **on DJI's
 * acceptance** and not at the ask, a refusal does not move it, a silent call does not move it, and a
 * *relative* rotate makes it unknown rather than letting it drift. Not mutation-measured — it was
 * written after the harness had run, and its rules are each asserted by exactly one test, so the
 * counts would all be 1 and would say nothing the file does not.
 */
class CommandedGimbalPortTest {

    private class FakePort : GimbalPort {
        var lastAbsolute: Boolean? = null
        var lastPitch: Double? = null
        /** Set by the test before a call: null accepts, non-null refuses with that message. */
        var refuseWith: String? = null
        /** When true, neither callback fires — measured DJI behaviour, four times on 2026-07-26. */
        var silent = false
        var cancelled = false

        override fun unavailableReason(): String? = null
        override fun canRotateByAngle(): Boolean = true

        override fun rotateByAngle(
            absolute: Boolean,
            pitchDeg: Double?,
            rollDeg: Double?,
            yawDeg: Double?,
            durationS: Double,
            onSuccess: () -> Unit,
            onFailure: (String) -> Unit,
        ) {
            lastAbsolute = absolute
            lastPitch = pitchDeg
            when {
                silent -> Unit
                refuseWith != null -> onFailure(refuseWith!!)
                else -> onSuccess()
            }
        }

        override fun listenAttitude(onDelivery: (GimbalAngles?) -> Unit) = Unit
        override fun listenAttitudeRange(onDelivery: (GimbalLimits?) -> Unit) = Unit
        override fun listenWorkMode(onDelivery: (String?) -> Unit) = Unit
        override fun listenConnection(onDelivery: (Boolean?) -> Unit) = Unit
        override fun cancelListens() { cancelled = true }
    }

    private fun rig(): Pair<FakePort, CommandedGimbalPort> {
        val inner = FakePort()
        val port = CommandedGimbalPort(inner)
        port.nowNanos = { 12_345L }
        return inner to port
    }

    private fun CommandedGimbalPort.aim(
        pitch: Double?,
        absolute: Boolean = true,
        onFailure: (String) -> Unit = {},
    ) = rotateByAngle(absolute, pitch, null, null, 0.5, onSuccess = {}, onFailure = onFailure)

    @Test
    fun nothingIsRememberedBeforeAnythingHasBeenCommanded() {
        val (_, port) = rig()
        assertNull(port.pitchDeg)
        assertNull(port.acceptedAtNanos)
    }

    @Test
    fun anAcceptedAbsoluteRotateIsRemembered() {
        val (_, port) = rig()
        port.aim(-90.0)
        assertEquals(-90.0, port.pitchDeg!!, 1e-12)
        assertEquals(12_345L, port.acceptedAtNanos)
    }

    /**
     * **A refused rotate left the camera where it was.** Remembering the angle we failed to reach
     * would be exactly the wrong number for a consumer that is about to do trigonometry with it —
     * `vision/TagWorld` would compute a fix from a pointing the gimbal never took.
     */
    @Test
    fun arefusedRotateDoesNotMoveTheRememberedAngle() {
        val (inner, port) = rig()
        port.aim(-90.0)
        inner.refuseWith = "GIMBAL_BUSY"
        var reported: String? = null
        port.aim(-45.0) { reported = it }
        assertEquals("the caller must still see the refusal", "GIMBAL_BUSY", reported)
        assertEquals("a refusal must not move the remembered angle", -90.0, port.pitchDeg!!, 1e-12)
    }

    /**
     * **A `performAction` that never calls back** is measured behaviour on this airframe, four times
     * in one session (`docs/measurements/2026-07-26-m2-first-command.md`). The angle is remembered on
     * acceptance, so silence leaves it exactly where it was.
     */
    @Test
    fun acallThatNeverAnswersDoesNotMoveTheRememberedAngle() {
        val (inner, port) = rig()
        port.aim(-90.0)
        inner.silent = true
        port.aim(-30.0)
        assertEquals(-90.0, port.pitchDeg!!, 1e-12)
    }

    /**
     * **A relative rotate clears it rather than drifting.** An increment cannot be integrated without
     * the reported angle this class exists to avoid, so the commanded angle becomes *unknown*, which
     * `TagWorld.fix` refuses on. Nothing in this project sends one today.
     */
    @Test
    fun arelativeRotateMakesTheAngleUnknownRatherThanWrong() {
        val (_, port) = rig()
        port.aim(-90.0)
        port.aim(10.0, absolute = false)
        assertNull("an increment is not an angle", port.pitchDeg)
        assertNull(port.acceptedAtNanos)
    }

    @Test
    fun anullOrNonFinitePitchIsNotRemembered() {
        val (_, port) = rig()
        port.aim(-90.0)
        port.aim(null)
        assertEquals("a null pitch is 'this axis unchanged', not an angle", -90.0, port.pitchDeg!!, 1e-12)
        port.aim(Double.NaN)
        assertEquals(-90.0, port.pitchDeg!!, 1e-12)
    }

    /**
     * The record on the [GimbalPort] seam itself, where `MsdkGimbalAim.believedPitch` reads it —
     * the one override of the interface's default null.
     */
    @Test
    fun theSeamSurfacesTheSameRecordThePropertyHolds() {
        val (_, port) = rig()
        assertNull(port.commandedPitchDeg())
        port.aim(-90.0)
        assertEquals(-90.0, port.commandedPitchDeg()!!, 1e-12)
        port.aim(10.0, absolute = false)
        assertNull("the relative-rotate unknown crosses the seam too", port.commandedPitchDeg())
    }

    /** Every argument reaches the inner port untouched — it is an observer, not a policy. */
    @Test
    fun everyArgumentIsForwardedUnchanged() {
        val (inner, port) = rig()
        port.aim(-63.5)
        assertEquals(true, inner.lastAbsolute)
        assertEquals(-63.5, inner.lastPitch!!, 1e-12)
        port.cancelListens()
        assertTrue(inner.cancelled)
    }
}
