package com.dimensional.mini4pro.vision

/**
 * **The on-board tag recogniser**: frames in on somebody else's thread, sightings out on ours, and
 * nothing in between that can make the video wait.
 *
 * Everything it is built from is separately testable and separately argued — [LatestFrame] for the
 * queue, [RateCap] for the sampling, [TagArming] for when it may run, [TagLatch] for what counts as
 * evidence, [TagWorld] for the geometry. This class is the wiring and the thread, and it is
 * deliberately the least interesting file in the package.
 *
 * ## The three safety properties, and where each one actually lives
 *
 * | property | enforced by | what breaks without it |
 * |---|---|---|
 * | the frame callback never waits on the detector | [LatestFrame.offer] — a lock held across reference assignments only, never across the copy and never across a detect | MSDK's decoded delivery stalls. **Measured**: 100 ms held in that callback takes it from 24.0 to 9.7 fps |
 * | the video passthrough is never touched | that MSDK dispatches the two on different threads | the ground station's video stalls. **Measured on the aircraft, three runs**: the encoded stream held 24.0 fps throughout the 100 ms block |
 * | the 25 Hz setpoint loop is never touched | this class owning one daemon thread and sharing no lock with `guided/` | an aircraft flies on late setpoints |
 *
 * The middle row is the one that was inference until 2026-07-28 and is now a measurement. It is
 * also the one no unit test can hold: it is a property of MSDK's threading, so it is pinned by
 * `vision/FrameListenerProbeTest` on hardware and stated here.
 *
 * ## What runs where
 *
 * **The frame source's thread** (`LIVE_VIEWDispatcher_17`, measured) does exactly four things: read
 * one volatile boolean, ask [RateCap] whether this frame is due, copy 2.07 MB, and return. Measured
 * cost 0.19–0.48 ms a frame, against a 41.5 ms frame interval.
 *
 * **The worker thread** does the detect, the geometry, the latch and the tap. It also re-evaluates
 * the arming rule every loop, which is why [LatestFrame.take] has a timeout: with the detector
 * disarmed no frames arrive, and a worker that only woke on a frame could never notice it was time
 * to arm again.
 *
 * ## Cost, measured on the aircraft rather than reasoned about
 *
 * | state | cores | note |
 * |---|---|---|
 * | bridge running, no frame listener | 0.43 | the floor everything else is a delta from |
 * | attached, disarmed (frames arriving, rejected) | 0.49 | **0.06 cores** — the cost of being ready |
 * | armed, 2 threads, 10 Hz cap | 1.11 | **0.68 cores** over the floor |
 * | armed, 2 threads, uncapped at 24 fps | 2.00 | what the cap is buying |
 * | armed, 8 threads, uncapped | 2.70 | a third of the phone |
 *
 * That 0.06-versus-0.68 gap is the whole argument for [TagArming] existing rather than the detector
 * simply always running.
 */
class TagRecogniser(
    private val source: FrameSource,
    /**
     * Makes the detector, **called on the worker thread**.
     *
     * A factory rather than an instance because `AprilTagDetector` is not thread-safe by design — it
     * reuses one native image across frames so that a detect allocates nothing — and the cleanest
     * way to guarantee one thread ever touches it is for one thread to have made it.
     */
    private val detectorFactory: () -> TagDetector,
    /** The aircraft, as the arming rule sees it. Read on the worker thread, never in a callback. */
    private val flight: () -> FlightView,
    /** Where the aircraft was, for the geometry. Read once per detected frame. */
    private val pose: () -> CameraPose,
    /** Every sighting goes here, on the way past. See [TagRecogniser]'s wiring in `Bridge`. */
    private val onSighting: (TagSighting.Sighting, TagFix?, Boolean) -> Unit,
    /** `SystemClock.elapsedRealtimeNanos` in the app, hand-cranked in tests. */
    private val nowNanos: () -> Long,
    private val config: Config = Config(),
    private val log: (String) -> Unit = {},
) : TagSighting, FrameListener {

    data class Config(
        val capHz: Double = RateCap.DEFAULT_HZ,
        /**
         * How old [latest] may be before it reports nothing.
         *
         * **One second**, which is ten missed detections at the cap. Long enough that the measured
         * gaps in a good band (92 % per-frame at 7–8 m) never blank the readout, short enough that
         * an operator glancing at the screen after the tag has left the frame sees "not seen"
         * rather than a stale position. It is a *display and consumer* bound, not a control one:
         * a controller must use [TagSighting.Sighting.ageMillisAt] and decide for itself.
         */
        val staleMillis: Long = 1_000,
        /**
         * The printed marker's side, metres. The cross-check range's input ([TagWorld]) and,
         * via `Bridge`'s detector factory, the pose solve's scale — one number, one marker.
         */
        val tagSizeM: Double = 0.075,
        /**
         * **The session's one camera model** — every projection this recogniser computes and,
         * via `Bridge`'s detector factory, the JNI solve's intrinsics, so the two cannot
         * disagree about the camera (`CameraCalibration`'s KDoc has the argument). Resolved
         * once per session in `Bridge.startTagRecogniser` from the drop-in file or the
         * [CameraCalibration.ASSUMED] prior, and recorded either way.
         */
        val calibration: CameraCalibration = CameraCalibration.ASSUMED,
        val minSightings: Int = TagLatch.DEFAULT_MIN_SIGHTINGS,
        val latchWindowNanos: Long = TagLatch.DEFAULT_WINDOW_NANOS,
        /** How long the worker waits for a frame before re-checking the arming rule. */
        val idleWaitMillis: Long = 250,
    )

    /** The operator's override. Read from the frame callback, so volatile and nothing more. */
    @Volatile
    var mode: TagArm = TagArm.AUTO

    /** Whether the arming rule currently permits work. The only thing the frame callback branches on. */
    @Volatile
    private var armed: Boolean = false
        private set

    /** The last arming decision's reason, for the status line. */
    @Volatile
    var why: String = "not started"
        private set

    private val mailbox = LatestFrame()
    private val cap = RateCap(config.capHz)
    private val latch = TagLatch(config.minSightings, config.latchWindowNanos)

    @Volatile private var armingState = TagArmingState()
    @Volatile private var newest: TagSighting.Sighting? = null

    /**
     * The newest **placeable** sighting's world fix, for [latestFix]. Only a frame that
     * produced a fix replaces it — a frame whose sighting could not be placed (pose missing,
     * camera off nadir) leaves the previous fix standing with its own older [TagFix.atNanos].
     * That is the honest shape for a consumer running a staleness ladder: the age it computes
     * is "how old is the newest place the camera could put the tag", which goes on growing
     * through unplaceable frames exactly as it does through frames with no tag at all, and the
     * ladder fires on it either way. Cleared with the flight, beside the latch.
     */
    @Volatile private var newestFix: TagFix? = null
    @Volatile private var worker: Thread? = null

    /** Frames the detector actually looked at, and how many held a tag. */
    @Volatile var detected: Long = 0L; private set
    @Volatile var hits: Long = 0L; private set

    /**
     * Attach to the frame source and start the worker. Returns null on success or a sentence saying
     * why not — the same shape [FrameSource.start] uses and for the same reason: every failure here
     * is one an operator can act on.
     *
     * Idempotent.
     */
    fun start(): String? {
        if (worker != null) return null
        val failure = source.start(this)
        if (failure != null) {
            why = failure
            return failure
        }
        worker = Thread(::runWorker, "tag-detect").apply { isDaemon = true; start() }
        log("tag recogniser started, cap ${config.capHz} Hz")
        return null
    }

    /**
     * Detach, stop the worker, and forget the flight.
     *
     * Ordered detach-first so that no frame can arrive after the mailbox is closed; a frame arriving
     * then would be dropped anyway, but "cannot happen" beats "is handled".
     */
    fun stop() {
        val w = worker ?: return
        worker = null
        armed = false
        why = "stopped"
        source.stop()
        mailbox.close()
        w.interrupt()
        w.join(2_000)
        newest = null
        newestFix = null
        log("tag recogniser stopped after $detected detections, $hits with a tag")
    }

    // ───────────────────────────────────────────────────── the producer side

    /**
     * **MSDK's decode thread.** Everything here is measured in tenths of a millisecond and nothing
     * here can wait — see the class KDoc's table.
     */
    override fun onLuma(data: ByteArray, offset: Int, width: Int, height: Int, atNanos: Long) {
        if (!armed) return
        if (!cap.admit(atNanos)) return
        mailbox.offer(data, offset, width, height, atNanos)
    }

    // ───────────────────────────────────────────────────── the consumer side

    private fun runWorker() {
        val detector = try {
            detectorFactory()
        } catch (e: Throwable) {
            why = "detector unavailable: ${e.message}"
            log(why)
            return
        }
        try {
            while (!Thread.currentThread().isInterrupted && worker != null) {
                stepArming()
                val frame = mailbox.take(config.idleWaitMillis) ?: continue
                // **Contained, always.** A detector fault is an evidence problem; it must never
                // become a flight problem, which is the rule `record/Tap` sets for the whole
                // project and this thread sits closer to the flight than most.
                val found = try {
                    detector.detect(frame.luma, frame.width, frame.height)
                } catch (e: Throwable) {
                    log("detector threw, frame skipped: $e")
                    Found.NOTHING
                }
                detected++
                publish(found, frame.width, frame.height, frame.atNanos)
            }
        } finally {
            runCatching { detector.close() }
        }
    }

    /** The arming rule, on the worker's clock. Also resets the latch when a new flight begins. */
    private fun stepArming() {
        val decision = try {
            TagArming.step(mode, armingState, flight(), latch.isLatched(), nowNanos())
        } catch (e: Throwable) {
            // A state read that throws must not stop the worker. Disarm and say so: refusing to
            // spend CPU is the safe direction, and a silent continue would leave the detector armed
            // on a rule that is no longer being evaluated.
            log("arming rule threw, disarming: $e")
            why = "arming rule failed"
            armed = false
            return
        }
        armingState = decision.state
        if (decision.newFlight) {
            latch.reset()
            cap.reset()
            newest = null
            newestFix = null
            log("new flight — tag latch cleared")
        }
        if (decision.armed != armed) log("tag detector ${if (decision.armed) "armed" else "disarmed"}: ${decision.why}")
        armed = decision.armed
        why = decision.why
    }

    private fun publish(found: Found, width: Int, height: Int, atNanos: Long) {
        val best = found.largest
        if (best == null) {
            latch.observedNothing()
            return
        }
        hits++
        val cameraPose = try {
            pose()
        } catch (e: Throwable) {
            log("pose read threw, sighting recorded without a fix: $e")
            CameraPose(null, null, null, null, null)
        }
        // The range along the optical axis, from the tag's apparent size. Carried on the sighting
        // AND on the fix for cross-checking, and deliberately **not** what the world fix uses —
        // see TagWorld's KDoc for why altitude is the better-grounded of the two. Since landing07
        // the cross-check has its consumer: the landing commit gate refuses when this number and
        // the barometer disagree (TagFix.sizeRangeM's KDoc).
        val sizeRange = TagWorld.rangeFromSize(
            best.longestEdgePixels, width, config.tagSizeM, config.calibration,
        )
        val range = sizeRange ?: 0.0
        val (x, y, z) = TagWorld.cameraFrame(
            best.centreX, best.centreY, width, height, range, config.calibration,
        )
        val sighting = TagSighting.Sighting(
            tagId = best.id,
            x = x, y = y, z = z,
            atNanos = atNanos,
            pixelSize = best.longestEdgePixels,
            // Not negotiable and not a placeholder: the focal length is a fit, the principal point
            // is an assumption and distortion is unmeasured. `docs/apriltag-landing-recording.md`.
            metric = false,
            hamming = best.hamming,
            decisionMargin = best.decisionMargin,
            centreX = best.centreX,
            centreY = best.centreY,
            imageWidth = width,
            imageHeight = height,
            corners = best.corners,
            // Raw, ungated — the record carries every solve so the gates stay measurable from
            // flight data. Belief is TagPose.trusted's question, asked at publication.
            solve = best.solve,
        )
        val fix = TagWorld.fix(
            best, width, height, cameraPose, atNanos, config.calibration, sizeRangeM = sizeRange,
        )
        val justLatched = latch.observe(best, fix, atNanos)
        newest = sighting
        if (fix != null) newestFix = fix
        if (justLatched) {
            log("tag ${best.id} latched for this flight" + (fix?.let {
                " at N%+.2f E%+.2f from %.1f m".format(it.northM, it.eastM, it.fromHeightM)
            } ?: " (position unknown)"))
        }
        // **Last, and outside everything.** The record is the primary evidence and must not depend
        // on Zenoh being up — but it also must not be able to take the worker down. Contained here
        // rather than in the tap, because a tap that had to be trusted is the shape this project
        // has already been bitten by.
        try {
            onSighting(sighting, fix, justLatched)
        } catch (e: Throwable) {
            log("sighting tap threw: $e")
        }
    }

    // ─────────────────────────────────────────────────────────── the readers

    override fun latest(): TagSighting.Sighting? {
        val s = newest ?: return null
        // Stale is reported as nothing, not as an old value. "The camera saw a tag a second ago" and
        // "the camera can see a tag" are different claims and only one of them is a sensor.
        return if (s.ageMillisAt(nowNanos()) > config.staleMillis) null else s
    }

    override fun latestFix(): TagFix? = newestFix

    override fun latched(): Latched? = latch.latched()

    /** What the queue has been doing, for the status line and the record's session summary. */
    fun counters(): Counters = Counters(
        armed = armed,
        why = why,
        mailbox = mailbox.counters(),
        capAdmitted = cap.admitted,
        capRefused = cap.refused,
        detected = detected,
        hits = hits,
    )

    data class Counters(
        val armed: Boolean,
        val why: String,
        val mailbox: LatestFrame.Counters,
        val capAdmitted: Long,
        val capRefused: Long,
        val detected: Long,
        val hits: Long,
    )
}
