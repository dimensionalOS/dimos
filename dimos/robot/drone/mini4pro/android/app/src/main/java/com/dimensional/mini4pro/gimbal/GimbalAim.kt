package com.dimensional.mini4pro.gimbal

import com.dimensional.mini4pro.command.ActionOutcome

/**
 * The one thing a QGroundControl operator may ask this bridge to do to the camera: **point it at a
 * pitch angle**.
 *
 * **This file contains no DJI code and must never contain any**, for the same reason
 * `command/FlightActions.kt` doesn't: the MSDK is `compileOnly` and is not on the unit-test
 * classpath (`docs/architecture.md`), so anything on the DJI side of a seam cannot be tested. The
 * shape here is deliberately the same as M2's — a DJI-free interface, every decision above it,
 * one thin DJI class below a second seam ([GimbalPort]) — because a reader who has understood
 * `command/` should not have to learn a second architecture to read this one.
 *
 * ## Why pitch, and only pitch
 *
 * Not a scoping convenience. **The Mini 4 Pro's gimbal refuses yaw and roll through the MSDK**,
 * and it refuses them in the most confusing way available: with the axis set to exactly `0.0`.
 * `docs/mini4pro-constraints.md` records the symptom; the issue itself
 * ([Mobile-SDK-Android-V5#527](https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/527), read
 * 2026-07-26) records the mechanism:
 *
 * > `GimbalAngleRotation` with `pitch = -90, yaw = 0.0, roll = 0.0, mode = ABSOLUTE_ANGLE` … "works
 * > as expected with a Mini 3 and a Mini 3 Pro. However with the Mini 4 Pro I get
 * > `SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW` and `SDK_SERVICE_GIMBAL_ROTATE_ROLL_NOT_ALLOW`."
 *
 * The reporter's own workaround — *"setting `yawIgnored` and `rollIgnored` to true prevents the
 * error"* — is the whole of our handling, and it is enforced in [MsdkGimbalAim] rather than in the
 * DJI class, so a test can hold it in place. DJI posted no reply.
 *
 * Read the title of that issue carefully before "improving" this. It says *when yaw or roll is set
 * to 0.0*, which invites the fix "then never send 0.0, send 0.001". **That fix does not work and
 * cannot work**, for two independent reasons:
 *
 *  1. The refusal is not about the number. `SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW` is DJI saying
 *     the *axis* may not be rotated on this airframe; `0.0` is simply the value the reporter
 *     happened to want. A nudge to `0.001` is still a yaw rotation.
 *  2. **`0.0` is not something we can decline to send.** Verified in the 5.18.0 bytecode:
 *     `GimbalAngleRotation.toBytes` serialises all ten fields positionally through
 *     `ByteStreamHelper.doubleToBytes(byte[], Double, int)`, whose first act is
 *     `if (arg1 == null) arg1 = Double.valueOf(0.0d)`. A `null` yaw therefore goes on the wire as
 *     `0.0`, indistinguishably from a deliberate one. **The `*Ignored` booleans are the only thing
 *     that can suppress an axis**, and "just leave it null" is a bug that looks like a fix.
 *
 * **How to re-read that bytecode, because the obvious reading is wrong.**
 * `dji-sdk-v5-aircraft-provided-5.18.0.jar` is a **compile-only ABI stub**: every method body has
 * a `return <default>` *prepended* at offsets 0-1, with the real implementation left behind it as
 * unreachable dead code. So `javap -c` on any MSDK method shows a two-instruction lie followed by
 * the truth, and "javap says it returns 0" is a linking artefact rather than shipped behaviour.
 * **The claim above is read from offset 2 onward** — `doubleToBytes` offsets 2-10 are the null
 * check, 11-37 the little-endian `ByteBuffer.putDouble` — i.e. from the real body. Anything in
 * this package that cites an MSDK *method body* says which form it is quoting.
 *
 * The key **declarations** are a different and safer kind of evidence: they are `DJIKeyInfo`
 * instance fields set by a builder chain in a static initialiser, so they are data rather than a
 * stubbed body. They are also independently corroborated — DJI's own offline HTML reference for
 * `GimbalKey` prints the same chain, `canGet(false).canSet(false).canListen(false)
 * .canPerformAction(true)` for `KeyRotateByAngle`, which matches what the bytecode parse found.
 *
 * So this bridge advertises no yaw axis and no roll axis to QGroundControl
 * ([GimbalEncoder.CAP_FLAGS]) and never populates either one toward DJI. A QGC control that
 * carries a yaw anyway — its Center and Tilt-90 buttons both hard-code `yaw = 0` — gets the pitch
 * performed and the operator gets told, in words, that the yaw was not
 * ([GimbalStatusTexts.YAW_UNAVAILABLE]).
 *
 * ## What a correct implementation must do
 *
 * The same four rules as `FlightActions`, and for the same reasons:
 *
 *  1. **Return the truth about the call you made.** [ActionOutcome.Requested] means *"I called the
 *     MSDK and it did not refuse on the spot"*. It is reused from `command/` deliberately: there
 *     is no `Success` case in this project's vocabulary, because nothing on this side of the seam
 *     can know whether the gimbal moved. The honest evidence is `GIMBAL_DEVICE_ATTITUDE_STATUS`
 *     built from `KeyGimbalAttitude` — what the gimbal *is* doing, never what was asked.
 *  2. **Do not block.** [aimPitch] is called from the MAVLink receive thread and from the
 *     telemetry thread's tick. Start the action and return.
 *  3. **Pass DJI's word through untouched.** [ActionOutcome.Refused] carries the MSDK error name
 *     verbatim — `SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW` is a string an operator can search for
 *     and our paraphrase of it is not.
 *  4. **Never invent an outcome.** A stub that throws reaches the operator as a failure
 *     ([GimbalManager] catches and reports); it can never become an `ACCEPTED`.
 *
 * ## What it must *not* do
 *
 * Nothing here is automatic. Every call originates in an operator pressing something in
 * QGroundControl. There is no recentre-on-connect, no level-on-takeoff, no restore-on-reconnect —
 * `docs/decisions/2026-07-25-m2-command-safety.md` §Q4 forbids unrequested commands to the
 * aircraft, and a gimbal command is a command to the aircraft even though it cannot fly it.
 */
interface GimbalAim {

    /**
     * Point the camera at [pitchDeg] degrees, absolute, in the gimbal's own frame.
     *
     * **Sign convention: negative is down.** This is DJI's (`GimbalAngleRotation.setPitch`: *"The
     * positive number is rotated upward, while the negative number is rotated downward"*) and it
     * is also QGroundControl's — its Tilt-90 button sends `param1 = -90`
     * (`src/Toolbar/GimbalIndicator.qml:188`) and its Center button sends `0`
     * (`GimbalController.cc:424-431`). The two agree, so **no sign flip happens anywhere in this
     * package**, and the absence is deliberate: a sign inversion here aims the camera at the sky
     * when the operator asked for the ground, and it is exactly the class of silent unit error
     * `docs/architecture.md` built the seam to make testable.
     *
     * Called at most once per [GimbalManager.MIN_ROTATE_INTERVAL_MS]; QGC's on-screen drag emits
     * at 10 Hz (`src/FlyView/OnScreenGimbalController.qml:62-73`) and [GimbalManager] coalesces
     * that to the latest setpoint rather than handing DJI ten commands a second.
     */
    fun aimPitch(pitchDeg: Double): ActionOutcome

    /**
     * Where the gimbal actually is, as last reported by DJI. Never a setpoint, never an echo of
     * [aimPitch] — see [GimbalReading].
     *
     * Read on the telemetry thread at 5 Hz, so implementations must be cheap and thread-safe.
     */
    fun reading(): GimbalReading

    /**
     * **The pitch this bridge believes the camera holds** — [PitchBelief]'s resolution, and the
     * single owner of it: commanded wins ([GimbalPort.commandedPitchDeg], the success-stamped
     * record), the last-reported attitude is the fallback, null means neither exists and is
     * never a zero. Every consumer that treats a pitch as a claim about where the camera points
     * — `TagWorld.fix`'s geometry, the descent arm gate's nadir check, the Zenoh `tf` camera
     * edge — reads this and nothing else, so no two of them can disagree; divergence is
     * structurally impossible because there is one implementation to diverge from.
     *
     * Deliberately not on [GimbalReading], whose "only measurements, never requests" rule this
     * would break: a belief is a resolution *between* a request and a measurement, and it says
     * which it chose.
     *
     * The exempt readers, named so the sweep is checkable: `Recorder.gimbalSource` and the
     * landing gimbal watchdog (`GuidedStickEngine.gimbalReportedPitchDeg`) read the raw
     * *reported* samples on purpose — they are measurements **of the gimbal**, not claims about
     * pointing, and the watchdog in particular exists to catch DJI moving the camera against
     * our own commanded −90°, which this belief (commanded wins) would mask by construction.
     */
    fun believedPitch(): PitchBelief?
}

/**
 * What DJI last told us about the gimbal. **Only measurements, never requests.**
 *
 * This project has already been bitten by the difference — on 2026-07-26 we asked DJI for
 * `AUTO_RTL`, DJI landed instead, and reporting `AUTO_LAND` was the correct thing to do. The same
 * rule applies to a camera angle, and it is enforced structurally: there is no field here that
 * could hold a requested angle, and [MsdkGimbalAim] never writes to this type from [GimbalAim
 * .aimPitch]'s argument. Every value in it arrives from a `KeyManager` callback.
 *
 * All fields nullable, and `null` means **no reading** rather than zero — the house rule from
 * `docs/architecture.md`. A zeroed gimbal attitude is a confidently level camera, which is a
 * specific and wrong claim.
 *
 * @param pitchDeg gimbal pitch, degrees, negative down. `KeyGimbalAttitude`.
 * @param rollDeg gimbal roll, degrees.
 * @param yawDeg gimbal yaw, degrees, **in the north-east-down earth frame** — DJI's own
 *   documentation for `KeyGimbalAttitude` says so in as many words: *"The yaw angle uses the north
 *   east down coordinate system. If you need to get the yaw angle of the gimbal relative to the
 *   nose of the aircraft, please call KeyYawRelativeToAircraftHeading."* That sentence is why
 *   [GimbalEncoder] sets `GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME` and not the vehicle-frame bit.
 * @param workMode DJI's `GimbalMode` enum **name**, verbatim (`FREE`, `FPV`, `YAW_FOLLOW`,
 *   `UNKNOWN`), unmapped — the same idiom `AircraftState.flightMode` uses, so the enum→meaning
 *   decision stays above the DJI seam where a test can reach it.
 * @param connected DJI's `GimbalKey.KeyConnection`. `false` means the gimbal component is gone and
 *   we must stop advertising one.
 * @param limits the gimbal's own reported travel, or null if DJI has not said.
 * @param attitudeAgeMs how long ago [pitchDeg] and friends were *delivered* (not changed) — the
 *   staleness model from `docs/decisions/2026-07-25-per-field-staleness.md`. Null when never
 *   delivered. **Reported, but no longer a gate**: on this key an old age means "the camera has
 *   not moved", not "the feed is dead" — see [isAdvertisable].
 * @param aircraftLinked whether the MSDK reported an aircraft link at the instant this reading was
 *   taken — `GimbalPort.unavailableReason() == null`, read fresh, never cached. Null means nobody
 *   asked. This is the liveness signal that [attitudeAgeMs] cannot be for a change-driven key, and
 *   it is a fact about DJI's own lifecycle rather than about any one key.
 */
data class GimbalReading(
    val pitchDeg: Double? = null,
    val rollDeg: Double? = null,
    val yawDeg: Double? = null,
    val workMode: String? = null,
    val connected: Boolean? = null,
    val limits: GimbalLimits? = null,
    val attitudeAgeMs: Long? = null,
    val aircraftLinked: Boolean? = null,
) {

    /**
     * Whether there is a gimbal to tell a ground station about at all.
     *
     * Three conditions, and each one is a thing we would otherwise be asserting without evidence:
     *
     *  - **The aircraft link is up right now** ([aircraftLinked]). This is the liveness condition,
     *    and it is deliberately *not* the age of the last attitude — see below.
     *  - **DJI has not said the component is gone.** `connected == false` is an explicit "no
     *    gimbal"; `null` is "not asked yet", which is not the same and does not block — the
     *    attitude arriving is itself evidence a gimbal exists.
     *  - **We have a pitch reading**, delivered during this link. `GIMBAL_DEVICE_ATTITUDE_STATUS`
     *    is a required part of the advertisement (QGC drops a gimbal that never sends one —
     *    `GimbalController.cc:329-343`) and it carries a quaternion. With no reading there is no
     *    honest quaternion to put in it. [MsdkGimbalAim] forgets the reading when the link drops,
     *    so a remembered angle can never outlive the aircraft it was measured on.
     *
     * ## Why an age is not one of the conditions any more
     *
     * It was, until 2026-07-26. `attitudeAgeMs < STALE_MS` was the rule, `STALE_MS` was 5 s, and
     * it was **measured wrong twice in one day**
     * (`docs/measurements/2026-07-26-gimbal-first-aim.md`,
     * `docs/measurements/2026-07-26-gimbal-keep-fresh-get.md`).
     *
     * `KeyGimbalAttitude` is **change-driven**: a stabilised, motionless gimbal stops delivering
     * altogether — six minutes of silence in the record, while the camera was in perfect health.
     * The age rule withdrew the advertisement on every gap, and QGC's first six discovery probes
     * were answered `UNSUPPORTED` because of it. **For this key an old age means "it has not
     * moved", not "the feed is dead"**, and those are different facts that no amount of waiting
     * separates.
     *
     * The first attempt at a fix kept the age rule and fed it: poll a `get` when the listener goes
     * quiet, so an unchanged gimbal produces deliveries anyway. **The aircraft refused.** DJI
     * answered every `getValue(KeyGimbalAttitude, callback)` with `REQUEST_HANDLER_NOT_FOUND`
     * (DJI's own gloss: *"not support"*) on a connected aircraft whose gimbal was delivering
     * listener updates at the same time. `canGet` in the key metadata is a statement about the
     * SDK's client-side key table, not a promise that the device serves the request.
     *
     * So the age cannot be kept fresh, and a fresh age cannot be faked:
     *
     *  - The **cache read** (`getValue(key)` without a callback) returns bytes and nothing else —
     *    `JNIKeyValue.native_get_sync` has no timestamp in its signature, and no MSDK API exposes
     *    when a cached value was written. Reading it and stamping "now" would be inventing
     *    liveness, which is the exact lie the staleness rule existed to prevent.
     *  - Re-subscribing would produce a delivery, because `getOnce = true` falls back to that same
     *    ageless cache read when the device get fails (`PushRecorder$1.onFailed`, read from the
     *    5.18.0 bytecode at offset 2). Same lie, one layer down.
     *
     * What is left is an honest liveness signal from somewhere other than this key, and there is
     * one: **DJI's own aircraft-link state**, maintained continuously by the SDK lifecycle and
     * read fresh on every tick. A gimbal that is connected, on an aircraft that is linked, with an
     * attitude DJI delivered during this link, is a gimbal we can honestly describe — that is what
     * a change-driven key *means*: the last delivered value is the current value until DJI says
     * otherwise.
     *
     * **What this gives up, plainly.** If the attitude push channel alone dies while the link and
     * the gimbal both still report healthy, we will go on publishing the last delivered angle with
     * no way to notice. Nothing in the MSDK distinguishes that from a camera holding still. The
     * trade is deliberate: the failure being given up on is speculative and has never been
     * observed, and the one being fixed was measured, certain, and cost the operator their gimbal
     * on every connect. An operator retains the check that matters — commanding a pitch and
     * watching whether the readout follows.
     *
     * Deliberately *not* a condition: whether the aircraft is flying, armed, or in any mode. A
     * camera on the ground still points somewhere.
     */
    fun isAdvertisable(): Boolean =
        aircraftLinked == true &&
            connected != false &&
            pitchDeg != null

    /*
     * There was a `STALE_MS = 5000` here until 2026-07-26, and the reason it is gone rather than
     * merely bigger is written out in [isAdvertisable]. The short version, so nobody reinstates it
     * from memory: it made an advertisement conditional on a delivery cadence this key does not
     * have, no number can distinguish "not moved" from "dead", and the two measurements that
     * settled it are dated in `docs/measurements/`.
     *
     * What withdrawing the advertisement is still for, unchanged: **QGroundControl never
     * de-registers a gimbal** — no removal code and no liveness timer anywhere in `src/Gimbal/`
     * (verified against QGC `da14fad28`) — so the indicator stays on screen showing the last angle
     * it heard. We cannot fix that from here. What we can do is stop being the source of the
     * claim, which is the distinction `HomeEventGate` draws: a bridge that has nothing to say says
     * nothing. The link and component conditions in [isAdvertisable] are that rule, resting on
     * facts DJI actually maintains.
     */
}

/**
 * The gimbal's own reported travel limits, degrees, from `GimbalKey.KeyGimbalAttitudeRange`.
 *
 * Used for exactly one thing: clamping a requested pitch to something DJI has said it can reach.
 * **Not used to invent limits** — when DJI has not reported a range, nothing is clamped and a
 * request DJI cannot honour comes back as DJI's own refusal, which is more informative than our
 * guess at the envelope.
 *
 * It is published in `GIMBAL_MANAGER_INFORMATION` too, but that is nearly decorative: QGC reads
 * `pitch_min`/`pitch_max`/`roll_*`/`yaw_*` **nowhere** (verified across `src/Gimbal/` at
 * `da14fad28`). Publishing them honestly costs nothing and helps any other ground station.
 */
data class GimbalLimits(
    val pitchMinDeg: Double? = null,
    val pitchMaxDeg: Double? = null,
    val rollMinDeg: Double? = null,
    val rollMaxDeg: Double? = null,
    val yawMinDeg: Double? = null,
    val yawMaxDeg: Double? = null,
) {

    /**
     * [pitchDeg] brought inside the reported pitch range, or unchanged if either end is unknown.
     *
     * Returns null when nothing was clamped, so a caller can tell "inside the envelope" from "moved
     * to the edge" and say so — [GimbalStatusTexts.clamped].
     */
    fun clampPitch(pitchDeg: Double): Double? {
        val min = pitchMinDeg
        val max = pitchMaxDeg
        if (min == null || max == null || min > max) return null
        val clamped = pitchDeg.coerceIn(min, max)
        return if (clamped == pitchDeg) null else clamped
    }
}
