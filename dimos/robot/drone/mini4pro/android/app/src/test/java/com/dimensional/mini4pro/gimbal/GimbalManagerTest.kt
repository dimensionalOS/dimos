package com.dimensional.mini4pro.gimbal

import com.dimensional.mini4pro.command.ActionOutcome
import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.CommandInterlock
import com.dimensional.mini4pro.handshake.HandshakeResponder
import com.dimensional.mini4pro.mavlink.StatusTextSink
import io.dronefleet.mavlink.common.CommandAck
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.GimbalDeviceAttitudeStatus
import io.dronefleet.mavlink.common.GimbalManagerInformation
import io.dronefleet.mavlink.common.GimbalManagerSetAttitude
import io.dronefleet.mavlink.common.GimbalManagerStatus
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavResult
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.Statustext
import io.dronefleet.mavlink.util.EnumValue
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The MAVLink half of the gimbal, driven through a **real** `HandshakeResponder` with the frames
 * QGroundControl actually sends, so the acknowledgement path under test is the production one.
 *
 * Every wire fact asserted here is read from QGC `da14fad28`'s source with the file and line named
 * at the assertion, and none of it has been measured against a running QGC — the operator's
 * QGroundControl is live on 14550 with a real aircraft and the aircraft is powered down.
 * `docs/gimbal.md` lists what a measurement would still settle. Nothing is guessed.
 *
 * The suite is written to fail loudly for the mistakes that would matter:
 *
 *  - a gimbal advertised before there is one, or after the feed has stopped
 *  - a command left unacknowledged, which in QGC's queue silently swallows the next ~1.2 s of
 *    commands to the same component (`MavCommandQueue.cc:274-286`, `:452-457`)
 *  - a refusal reported to QGC as a success
 *  - an operator locked out of their own camera by a `primary_control` we invented
 *  - ten DJI calls a second for one drag
 *  - the *last* setpoint of a drag being dropped, leaving the camera short of where the operator
 *    let go
 *
 * Mutation-checked 2026-07-26 — each breakage was applied to the source, the whole gimbal suite
 * run, the failing tests counted, and the source restored. **Every one was caught.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` handler not registered | 21 |
 *  | `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE` handler not registered | 3 |
 *  | `Refused` acked `ACCEPTED` | 1 |
 *  | every outcome acked `ACCEPTED` | 6 |
 *  | `CANNOT_PERFORM_ACTION` acked `TEMPORARILY_REJECTED` rather than `UNSUPPORTED` | 1 |
 *  | coalescing replaced with a plain drop (pending setpoint discarded) | 1 |
 *  | coalescing removed entirely (every command straight to DJI) | 3 |
 *  | `flushPending` never called from `tick` | 1 |
 *  | the yaw announcement removed | 2 |
 *  | the rate announcement removed | 1 |
 *  | `announce` de-duplication removed | 1 |
 *  | `CONFIGURE` param −3 (release) treated as a literal id | 1 |
 *  | `STATUS_PERIOD_MS` dropped to the attitude cadence | 1 |
 *  | `reset()` stops clearing the control holder | 1 |
 *  | a command for another gimbal device performed anyway | 1 |
 *  | the joystick rate message silently ignored | 1 |
 */
class GimbalManagerTest {

    // ------------------------------------------------------------------- rig

    private val gcsSysId = HandshakeResponder.DEFAULT_GCS_SYSTEM_ID
    private val gcsCompId = HandshakeResponder.DEFAULT_GCS_COMPONENT_ID

    private var nowMs = 0L
    private val sent = mutableListOf<Any>()
    private val logs = mutableListOf<String>()

    private val responder = HandshakeResponder(send = { sent.add(it) }, nowMs = { nowMs })

    private val manager = GimbalManager(
        announcer = Announcer(StatusTextSink { sent.add(it) }),
        log = { logs.add(it) },
        nowMs = { nowMs },
        timeBootMs = { nowMs },
    ).also { it.attachTo(responder) }

    /** A gimbal that is present, level and freshly reported. */
    private var reading = GimbalReading(
        pitchDeg = 0.0,
        rollDeg = 0.0,
        yawDeg = 0.0,
        workMode = "YAW_FOLLOW",
        connected = true,
        attitudeAgeMs = 0L,
        aircraftLinked = true,
    )

    private val aimed = mutableListOf<Double>()
    private var outcome: (Double) -> ActionOutcome = { ActionOutcome.Requested }

    private val fakeAim = object : GimbalAim {
        /** What [believedPitch] surfaces — set by the forwarding test, null everywhere else. */
        var belief: PitchBelief? = null

        override fun aimPitch(pitchDeg: Double): ActionOutcome {
            aimed.add(pitchDeg)
            return outcome(pitchDeg)
        }

        override fun reading(): GimbalReading = reading

        override fun believedPitch(): PitchBelief? = belief
    }

    private val statusTexts: List<Statustext> get() = sent.filterIsInstance<Statustext>()
    private val acks: List<CommandAck> get() = sent.filterIsInstance<CommandAck>()

    private fun attachAim() {
        manager.aim = fakeAim
    }

    /**
     * QGC's gimbal angle command, exactly as `GimbalController::sendPitchBodyYaw` builds it
     * (`GimbalController.cc:477-504`): degrees in param1/param2, NaN rates, flags in param5,
     * device id in param7.
     */
    private fun pitchYaw(
        pitchDeg: Float,
        yawDeg: Float = 0f,
        pitchRate: Float = Float.NaN,
        yawRate: Float = Float.NaN,
        deviceId: Float = GimbalEncoder.GIMBAL_DEVICE_ID.toFloat(),
    ): CommandLong = CommandLong.builder()
        .targetSystem(1)
        .targetComponent(1)
        .command(MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)
        .param1(pitchDeg)
        .param2(yawDeg)
        .param3(pitchRate)
        .param4(yawRate)
        // ROLL_LOCK | PITCH_LOCK | YAW_IN_VEHICLE_FRAME, the value QGC computes at :481-483.
        .param5(44f)
        .param6(0f)
        .param7(deviceId)
        .build()

    private fun configure(
        primarySysid: Float,
        primaryCompid: Float,
        deviceId: Float = GimbalEncoder.GIMBAL_DEVICE_ID.toFloat(),
    ): CommandLong = CommandLong.builder()
        .targetSystem(1)
        .targetComponent(1)
        .command(MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE)
        .param1(primarySysid)
        .param2(primaryCompid)
        .param3(-1f)
        .param4(-1f)
        .param7(deviceId)
        .build()

    private fun requestMessage(id: Int): CommandLong = CommandLong.builder()
        .targetSystem(1)
        .targetComponent(1)
        .command(MavCmd.MAV_CMD_REQUEST_MESSAGE)
        .param1(id.toFloat())
        .build()

    private fun deliver(payload: Any) = responder.onMessage(payload, gcsSysId, gcsCompId)

    /** Moves past both the rate window and the announcement window. */
    private fun quietPeriod() {
        nowMs += GimbalManager.ANNOUNCE_REPEAT_MS + 1
    }

    // ------------------------------------------------------------- advertising

    @Test
    fun `nothing is advertised until DJI reports a gimbal`() {
        attachAim()
        reading = GimbalReading()

        assertEquals(emptyList<Any>(), manager.tick())
        deliver(requestMessage(GimbalEncoder.MESSAGE_ID_GIMBAL_MANAGER_INFORMATION))
        // UNSUPPORTED, from the provider returning null. QGC re-asks — six heartbeat-driven
        // attempts plus three more once any gimbal message arrives — so "not yet" is not "never".
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.single().result().entry())
        assertTrue(sent.none { it is GimbalManagerInformation })
    }

    @Test
    fun `with no aim attached at all the bridge emits exactly what it did before`() {
        assertEquals(emptyList<Any>(), manager.tick())
        deliver(requestMessage(GimbalEncoder.MESSAGE_ID_GIMBAL_MANAGER_INFORMATION))
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.single().result().entry())
    }

    @Test
    fun `GIMBAL_MANAGER_INFORMATION is served on request once a gimbal exists`() {
        attachAim()
        deliver(requestMessage(GimbalEncoder.MESSAGE_ID_GIMBAL_MANAGER_INFORMATION))

        assertEquals(MavResult.MAV_RESULT_ACCEPTED, acks.single().result().entry())
        val info = sent.filterIsInstance<GimbalManagerInformation>().single()
        assertEquals(GimbalEncoder.GIMBAL_DEVICE_ID, info.gimbalDeviceId())
    }

    @Test
    fun `both streamed messages go out unprompted, at their own cadences`() {
        // QGC sends no SET_MESSAGE_INTERVAL that we honour, and its own request path is easy to
        // miss entirely (`_checkComplete` runs only from the three message handlers), so anything
        // it needs has to be streamed. It also needs all three before it registers a gimbal at
        // all (`GimbalController.cc:329-343`).
        attachAim()

        val first = manager.tick()
        assertEquals(1, first.filterIsInstance<GimbalDeviceAttitudeStatus>().size)
        assertEquals(1, first.filterIsInstance<GimbalManagerStatus>().size)

        // 5 Hz attitude, 1 Hz status: four more ticks give four attitudes and no status.
        var attitudes = 0
        var statuses = 0
        repeat(4) {
            nowMs += GimbalManager.ATTITUDE_PERIOD_MS
            manager.tick().forEach {
                if (it is GimbalDeviceAttitudeStatus) attitudes++
                if (it is GimbalManagerStatus) statuses++
            }
        }
        assertEquals(4, attitudes)
        assertEquals(0, statuses)

        nowMs += GimbalManager.ATTITUDE_PERIOD_MS
        assertEquals(1, manager.tick().filterIsInstance<GimbalManagerStatus>().size)
    }

    @Test
    fun `a feed that stops is not what stops the advertisement — a lost aircraft is`() {
        attachAim()
        assertTrue(manager.tick().isNotEmpty())

        // A minute with no delivery, on a change-driven key, is a camera that has not moved.
        reading = reading.copy(attitudeAgeMs = 60_000)
        nowMs += 1_000
        assertTrue(manager.tick().isNotEmpty())

        reading = reading.copy(aircraftLinked = false)
        nowMs += 1_000
        assertEquals(emptyList<Any>(), manager.tick())
    }

    // --------------------------------------------- MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW

    @Test
    fun `QGC's Tilt 90 button points the camera straight down`() {
        // GimbalIndicator.qml:188 — `sendPitchBodyYaw(-90, 0, true)`. This is Ivan's "down", and
        // it is one press of a button QGC shows unconditionally once a gimbal registers.
        attachAim()
        deliver(pitchYaw(pitchDeg = -90f))

        assertEquals(listOf(-90.0), aimed)
        assertEquals(MavResult.MAV_RESULT_ACCEPTED, acks.single().result().entry())
    }

    @Test
    fun `QGC's Center button points the camera forward`() {
        // GimbalController.cc:424-431 — `sendPitchBodyYaw(0, 0, true)`. Ivan's "forward".
        attachAim()
        deliver(pitchYaw(pitchDeg = 0f))

        assertEquals(listOf(0.0), aimed)
        assertEquals(MavResult.MAV_RESULT_ACCEPTED, acks.single().result().entry())
    }

    @Test
    fun `the pitch is read from param1 in degrees, unscaled and unflipped`() {
        attachAim()
        listOf(-90f, -45f, 0f, 30f, 55.5f).forEachIndexed { i, p ->
            nowMs += GimbalManager.MIN_ROTATE_INTERVAL_MS
            deliver(pitchYaw(pitchDeg = p))
            assertEquals(p.toDouble(), aimed[i], 1e-6)
        }
    }

    @Test
    fun `every command is acknowledged exactly once`() {
        // Not a courtesy. MavCommandQueue refuses to send a command that is still awaiting an ack
        // to the same component and only clears the entry after a 1200 ms sweep
        // (MavCommandQueue.cc:274-286, :452-457), so an unacknowledged command silently swallows
        // the next ~1.2 s of them — a drag control that moves once and stops.
        attachAim()
        deliver(pitchYaw(pitchDeg = -30f))
        assertEquals(1, acks.size)
        assertEquals(
            EnumValue.of(MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW).value(),
            acks.single().command().value(),
        )
    }

    @Test
    fun `a NaN pitch asks for nothing and says so`() {
        // MAVLink's "leave this axis alone". Nothing to do, and nothing went wrong.
        attachAim()
        deliver(pitchYaw(pitchDeg = Float.NaN, yawDeg = Float.NaN))

        assertTrue(aimed.isEmpty())
        assertEquals(MavResult.MAV_RESULT_ACCEPTED, acks.single().result().entry())
        assertTrue(statusTexts.isEmpty())
    }

    @Test
    fun `a command for a gimbal we do not manage is denied rather than performed on ours`() {
        attachAim()
        deliver(pitchYaw(pitchDeg = -90f, deviceId = 4f))

        assertTrue(aimed.isEmpty())
        assertEquals(MavResult.MAV_RESULT_DENIED, acks.single().result().entry())
    }

    @Test
    fun `a broadcast device id is ours too`() {
        attachAim()
        deliver(pitchYaw(pitchDeg = -90f, deviceId = 0f))
        assertEquals(listOf(-90.0), aimed)
    }

    // ------------------------------------------------------------------ refusals

    @Test
    fun `a DJI refusal is never acknowledged as a success, and carries DJI's own word`() {
        attachAim()
        outcome = { ActionOutcome.Refused("SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW") }

        deliver(pitchYaw(pitchDeg = -90f))

        assertEquals(MavResult.MAV_RESULT_DENIED, acks.single().result().entry())
        // 39 bytes on its own, so the framing has to go and DJI's string survives intact.
        assertTrue(
            statusTexts.any { it.text() == "SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW" },
        )
    }

    @Test
    fun `an unavailable gimbal is temporarily rejected and explained`() {
        attachAim()
        outcome = { ActionOutcome.Unavailable("NO_PRODUCT") }

        deliver(pitchYaw(pitchDeg = -90f))

        assertEquals(MavResult.MAV_RESULT_TEMPORARILY_REJECTED, acks.single().result().entry())
        assertTrue(statusTexts.any { it.text() == "Gimbal failed: NO_PRODUCT" })
    }

    @Test
    fun `a key DJI declares unperformable is UNSUPPORTED, not merely rejected`() {
        // QGC renders each result as a different sentence (MavCommandQueue.cc:466-479).
        // "not supported" is a statement about the whole capability; "rejected" is about this
        // attempt. A demoted key is the former.
        attachAim()
        outcome = { ActionOutcome.Unavailable(MsdkGimbalAim.CANNOT_PERFORM_ACTION) }

        deliver(pitchYaw(pitchDeg = -90f))

        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.single().result().entry())
    }

    @Test
    fun `an aim that throws is a failure the operator hears about`() {
        attachAim()
        outcome = { throw IllegalStateException("port not wired") }

        deliver(pitchYaw(pitchDeg = -90f))

        assertEquals(MavResult.MAV_RESULT_TEMPORARILY_REJECTED, acks.single().result().entry())
        assertTrue(statusTexts.any { it.text() == "Gimbal failed: port not wired" })
    }

    @Test
    fun `a command with no DJI layer behind it says so`() {
        // manager.aim stays null: the bridge is stopped, or was never started.
        deliver(pitchYaw(pitchDeg = -90f))

        assertEquals(MavResult.MAV_RESULT_TEMPORARILY_REJECTED, acks.single().result().entry())
        assertTrue(statusTexts.any { it.text() == "Gimbal failed: NO_AIRCRAFT_LINK" })
    }

    @Test
    fun `an asynchronous DJI error reaches the operator, and a blank one does not`() {
        manager.reportAsyncDjiError("SDK_SERVICE_GIMBAL_ROTATE_ROLL_NOT_ALLOW")
        manager.reportAsyncDjiError("   ")

        assertEquals(1, statusTexts.size)
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, statusTexts.single().severity().entry())
    }

    // --------------------------------------------------------------- the yaw truth

    @Test
    fun `a command carrying a yaw tells the operator the yaw did not happen`() {
        // QGC's Center and Tilt-90 both hard-code yaw = 0, and QGC reads our HAS_YAW_AXIS
        // capability bit nowhere, so this STATUSTEXT is the operator's only route to knowing that
        // half of what they pressed is not available on this airframe.
        attachAim()
        deliver(pitchYaw(pitchDeg = -90f, yawDeg = 0f))

        assertTrue(statusTexts.any { it.text() == GimbalStatusTexts.YAW_UNAVAILABLE })
        // And the pitch still happened.
        assertEquals(listOf(-90.0), aimed)
    }

    @Test
    fun `a command with no yaw at all says nothing about yaw`() {
        attachAim()
        deliver(pitchYaw(pitchDeg = -90f, yawDeg = Float.NaN))
        assertTrue(statusTexts.none { it.text() == GimbalStatusTexts.YAW_UNAVAILABLE })
    }

    @Test
    fun `the yaw sentence is not repeated for every tick of a drag`() {
        attachAim()
        repeat(20) {
            nowMs += 100
            deliver(pitchYaw(pitchDeg = -10f - it, yawDeg = 5f))
        }
        // 2 s of dragging at QGC's 10 Hz, inside one ANNOUNCE_REPEAT_MS window.
        assertEquals(1, statusTexts.count { it.text() == GimbalStatusTexts.YAW_UNAVAILABLE })

        quietPeriod()
        deliver(pitchYaw(pitchDeg = -30f, yawDeg = 5f))
        assertEquals(2, statusTexts.count { it.text() == GimbalStatusTexts.YAW_UNAVAILABLE })
    }

    // ---------------------------------------------------------------- rate refusal

    @Test
    fun `a pitch rate in the command is refused out loud`() {
        attachAim()
        deliver(pitchYaw(pitchDeg = Float.NaN, yawDeg = Float.NaN, pitchRate = 20f))
        assertTrue(statusTexts.any { it.text() == GimbalStatusTexts.RATE_UNSUPPORTED })
    }

    @Test
    fun `the joystick rate message is refused out loud rather than ignored`() {
        // GIMBAL_MANAGER_SET_ATTITUDE (282), QGC's joystick path — a message, not a command, so
        // there is nothing to acknowledge and STATUSTEXT is the only channel. A control that does
        // nothing and says nothing is the failure this layer exists to prevent.
        attachAim()
        val taken = manager.onInbound(rateMessage(pitchRateRadS = 0.35f), gcsSysId, gcsCompId)

        assertTrue(taken)
        assertTrue(statusTexts.any { it.text() == GimbalStatusTexts.RATE_UNSUPPORTED })
        assertTrue("a message is never acked", acks.isEmpty())
        assertTrue(aimed.isEmpty())
    }

    @Test
    fun `a zero rate is a stop and needs no words`() {
        attachAim()
        manager.onInbound(rateMessage(pitchRateRadS = 0f), gcsSysId, gcsCompId)
        assertTrue(statusTexts.isEmpty())
    }

    @Test
    fun `a rate message addressed elsewhere is not ours`() {
        attachAim()
        val elsewhere = GimbalManagerSetAttitude.builder()
            .targetSystem(7)
            .targetComponent(1)
            .gimbalDeviceId(GimbalEncoder.GIMBAL_DEVICE_ID)
            .angularVelocityX(Float.NaN)
            .angularVelocityY(0.35f)
            .angularVelocityZ(Float.NaN)
            .build()

        assertFalse(manager.onInbound(elsewhere, gcsSysId, gcsCompId))
        assertTrue(statusTexts.isEmpty())
    }

    @Test
    fun `an unrelated payload is not ours`() {
        assertFalse(manager.onInbound(Statustext.builder().text("hello").build(), gcsSysId, gcsCompId))
        assertFalse(manager.onInbound(null, gcsSysId, gcsCompId))
    }

    private fun rateMessage(pitchRateRadS: Float): GimbalManagerSetAttitude =
        GimbalManagerSetAttitude.builder()
            .targetSystem(1)
            .targetComponent(1)
            .gimbalDeviceId(GimbalEncoder.GIMBAL_DEVICE_ID)
            .q(listOf(Float.NaN, Float.NaN, Float.NaN, Float.NaN))
            .angularVelocityX(Float.NaN)
            .angularVelocityY(pitchRateRadS)
            .angularVelocityZ(Float.NaN)
            .build()

    // ------------------------------------------------------------- rate limiting

    @Test
    fun `a 10 Hz drag does not become 10 DJI calls a second`() {
        attachAim()
        // One second of QGC's on-screen drag: 10 commands, 100 ms apart, each a new angle.
        repeat(10) {
            deliver(pitchYaw(pitchDeg = -10f - it))
            nowMs += 100
        }
        assertTrue("expected about 5 calls, got ${aimed.size}", aimed.size <= 6)
        assertTrue(aimed.isNotEmpty())
    }

    @Test
    fun `the last setpoint of a drag is always delivered`() {
        // The failure this prevents is the camera stopping short of where the operator let go,
        // which is this project's characteristic shape: a control that stops halfway while the
        // display says otherwise. A plain "drop anything inside the window" filter has exactly
        // that bug.
        attachAim()
        deliver(pitchYaw(pitchDeg = -10f))
        nowMs += 50
        deliver(pitchYaw(pitchDeg = -20f))
        nowMs += 50
        deliver(pitchYaw(pitchDeg = -30f))

        assertEquals("only the first went straight through", listOf(-10.0), aimed)

        // The operator lets go. The tick delivers the angle they stopped at.
        nowMs += GimbalManager.MIN_ROTATE_INTERVAL_MS
        manager.tick()
        assertEquals(listOf(-10.0, -30.0), aimed)
    }

    @Test
    fun `a coalesced setpoint is acknowledged rather than left pending`() {
        // MAV_RESULT_IN_PROGRESS is forbidden here: QGC keeps such a command pending forever and
        // then refuses to send that command id again at all
        // (HandshakeResponder.registerCommandHandler's KDoc, MavCommandQueue.cc:497-506).
        attachAim()
        deliver(pitchYaw(pitchDeg = -10f))
        nowMs += 50
        deliver(pitchYaw(pitchDeg = -20f))

        assertEquals(2, acks.size)
        assertTrue(acks.all { it.result().entry() == MavResult.MAV_RESULT_ACCEPTED })
    }

    @Test
    fun `the first command after a quiet period is never deferred`() {
        // So that a genuinely unavailable gimbal is discovered synchronously and refused with a
        // truthful ack. Only the middle of a fast drag can be answered optimistically.
        attachAim()
        outcome = { ActionOutcome.Unavailable("NO_PRODUCT") }

        deliver(pitchYaw(pitchDeg = -90f))

        assertEquals(listOf(-90.0), aimed)
        assertEquals(MavResult.MAV_RESULT_TEMPORARILY_REJECTED, acks.single().result().entry())
    }

    @Test
    fun `the same angle inside the window is not re-commanded`() {
        attachAim()
        deliver(pitchYaw(pitchDeg = -30f))
        nowMs += 10
        deliver(pitchYaw(pitchDeg = -30.05f))

        assertEquals(listOf(-30.0), aimed)
        nowMs += GimbalManager.MIN_ROTATE_INTERVAL_MS
        manager.tick()
        assertEquals("and nothing was queued either", listOf(-30.0), aimed)
    }

    // ------------------------------------------- MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE

    @Test
    fun `nobody holds control until QGC asks`() {
        attachAim()
        val status = manager.tick().filterIsInstance<GimbalManagerStatus>().single()
        assertEquals(0, status.primaryControlSysid())
        assertEquals(0, status.primaryControlCompid())
    }

    @Test
    fun `acquiring control is accepted and reported back`() {
        // QGC sends this unprompted before the operator's first command, with showError = true —
        // anything but ACCEPTED raises a modal dialog every time they touch the camera. And the
        // echo matters: if GIMBAL_MANAGER_STATUS ever names a controller that is not this QGC,
        // `_tryGetGimbalControl` blocks every operator action behind a popup
        // (GimbalController.cc:179-180, :346-352).
        attachAim()
        deliver(configure(primarySysid = 255f, primaryCompid = 190f))

        assertEquals(MavResult.MAV_RESULT_ACCEPTED, acks.single().result().entry())
        val status = manager.tick().filterIsInstance<GimbalManagerStatus>().single()
        assertEquals(255, status.primaryControlSysid())
        assertEquals(190, status.primaryControlCompid())
    }

    @Test
    fun `releasing control clears it rather than storing minus three`() {
        attachAim()
        deliver(configure(primarySysid = 255f, primaryCompid = 190f))
        nowMs += GimbalManager.STATUS_PERIOD_MS
        deliver(configure(primarySysid = -3f, primaryCompid = -3f))

        val status = manager.tick().filterIsInstance<GimbalManagerStatus>().single()
        assertEquals(0, status.primaryControlSysid())
        assertEquals(0, status.primaryControlCompid())
    }

    @Test
    fun `minus one leaves the control holder alone`() {
        attachAim()
        deliver(configure(primarySysid = 255f, primaryCompid = 190f))
        nowMs += GimbalManager.STATUS_PERIOD_MS
        deliver(configure(primarySysid = -1f, primaryCompid = -1f))

        val status = manager.tick().filterIsInstance<GimbalManagerStatus>().single()
        assertEquals(255, status.primaryControlSysid())
        assertEquals(190, status.primaryControlCompid())
    }

    @Test
    fun `minus two means the sender itself`() {
        attachAim()
        deliver(configure(primarySysid = -2f, primaryCompid = -2f))

        val status = manager.tick().filterIsInstance<GimbalManagerStatus>().single()
        assertEquals(gcsSysId, status.primaryControlSysid())
        assertEquals(gcsCompId, status.primaryControlCompid())
    }

    @Test
    fun `a reset forgets who held control`() {
        // A stale primary controller surviving into the next link would lock the next operator
        // out of their own camera behind QGC's "Request Gimbal Control?" popup.
        attachAim()
        deliver(configure(primarySysid = 255f, primaryCompid = 190f))
        manager.reset()

        val status = manager.tick().filterIsInstance<GimbalManagerStatus>().single()
        assertEquals(0, status.primaryControlSysid())
    }

    @Test
    fun `a reset drops any coalesced setpoint`() {
        attachAim()
        deliver(pitchYaw(pitchDeg = -10f))
        nowMs += 50
        deliver(pitchYaw(pitchDeg = -30f))
        manager.reset()

        nowMs += GimbalManager.MIN_ROTATE_INTERVAL_MS
        manager.tick()
        assertEquals("the pending -30 must not survive the link that asked for it", listOf(-10.0), aimed)
    }

    // ------------------------------------------------------------------ interlock

    @Test
    fun `aiming the camera is deliberately not gated by the command interlock`() {
        // This is a decision, argued in GimbalManager's KDoc and recorded in docs/gimbal.md as
        // Ivan's to overturn: the interlock means "this bridge can move an aircraft", a gimbal
        // cannot move one, and gating the routine act of aiming a camera behind the switch that
        // arms Return and Land would leave that switch on for most of every flight.
        //
        // The test is here so that reversing the decision is a deliberate act with a failing test
        // attached, rather than something that drifts either way unnoticed.
        val interlock = CommandInterlock()
        assertFalse("the interlock is off, as it is at every process start", interlock.enabled)

        attachAim()
        deliver(pitchYaw(pitchDeg = -90f))

        assertEquals(listOf(-90.0), aimed)
        assertEquals(MavResult.MAV_RESULT_ACCEPTED, acks.single().result().entry())
    }

    @Test
    fun `no gimbal message can switch the command interlock on`() {
        val interlock = CommandInterlock()
        attachAim()
        deliver(pitchYaw(pitchDeg = -90f))
        deliver(configure(primarySysid = 255f, primaryCompid = 190f))
        manager.onInbound(rateMessage(pitchRateRadS = 1f), gcsSysId, gcsCompId)
        deliver(requestMessage(GimbalEncoder.MESSAGE_ID_GIMBAL_MANAGER_INFORMATION))

        assertFalse(interlock.enabled)
    }

    // ------------------------------------------------------------------- severity

    @Test
    fun `every gimbal sentence is at a severity QGroundControl actually shows`() {
        attachAim()
        outcome = { ActionOutcome.Refused("SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW") }
        deliver(pitchYaw(pitchDeg = -90f, yawDeg = 0f))
        quietPeriod()
        deliver(pitchYaw(pitchDeg = Float.NaN, pitchRate = 20f))

        assertTrue(statusTexts.isNotEmpty())
        // QGC surfaces only EMERGENCY/ALERT/CRITICAL/ERROR (StatusTextHandler.cc:18-24); anything
        // below is filed silently in a list nobody opens.
        assertTrue(
            statusTexts.all { it.severity().entry() == MavSeverity.MAV_SEVERITY_ERROR },
        )
    }

    @Test
    fun `a served GIMBAL_MANAGER_INFORMATION reaches the wire and not just the ack`() {
        attachAim()
        deliver(requestMessage(GimbalEncoder.MESSAGE_ID_GIMBAL_MANAGER_INFORMATION))
        assertNotNull(sent.filterIsInstance<GimbalManagerInformation>().singleOrNull())
    }

    // ------------------------------------------------------------ the believed pitch

    @Test
    fun `believedPitch surfaces the aim's own belief untouched - one owner, no second resolution`() {
        // This class must add nothing: the resolution is MsdkGimbalAim's (where both sources
        // live) through PitchBelief.of (the one implementation). A manager that recomputed or
        // adjusted it would be the second copy the single-owner rule forbids.
        attachAim()
        fakeAim.belief = PitchBelief(-90.0, reported = true)
        assertEquals(PitchBelief(-90.0, reported = true), manager.believedPitch())
    }

    @Test
    fun `believedPitch with no aim attached is null - a stopped bridge believes nothing`() {
        assertNull(manager.believedPitch())
    }
}
