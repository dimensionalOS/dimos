package com.dimensional.mini4pro.video

/**
 * The 1 Hz control loop's decision, as a pure function.
 *
 * [VideoStreamer] used to make these choices inline, which meant the ordering
 * rules that matter most — registration before anything, tear down before
 * waiting, back off before retrying, give up on a `startStream` that never calls
 * back — could only be checked with an aircraft on the bench. They are the rules
 * most likely to be broken by a later edit and the most expensive to debug in a
 * hardware window, so they live here where JVM tests can reach them.
 *
 * [VideoStreamer] keeps only the part that genuinely needs the MSDK: reading the
 * inputs and carrying out the [Decision].
 */
object VideoLoop {

    /** Backoff after a failed `startStream()`; DJI's failures here are often transient. */
    const val RETRY_AFTER_FAILURE_MS = 5_000L

    /**
     * **How long SERVING with no new frame means the passthrough has stalled**, rather than a
     * hiccup worth waiting out.
     *
     * Measured, not chosen. Inter-frame gaps across three recorded flights — landing17 (16 233
     * frames), landing18 (12 794) and church01 (26 106), 55 133 gaps in total:
     *
     * | flight | median | p99.9 | **max** |
     * |---|---|---|---|
     * | church01 | 0.017 s | 0.057 s | **0.413 s** |
     * | landing17 | 0.017 s | 0.069 s | **0.928 s** |
     * | landing18 | 0.017 s | 0.130 s | **1.043 s** |
     *
     * So the worst gap a *healthy* feed has ever produced on this airframe is **1.043 s**, and this
     * threshold sits at 2.9× that. It is the same size as `VideoStatus.FIRST_FRAME_GRACE_S`, which
     * was sized independently for the aircraft's first keyframe after a source switch — two
     * different questions landing on the same number is a mild corroboration, not a coincidence to
     * lean on.
     *
     * The cost of firing early is one restart, about a second of video. The cost of firing late is
     * what happened on 2026-07-30: the feed stopped after 8 frames and nobody noticed for 110 s,
     * with the tag detector blind for all of it because it eats the same stream.
     */
    const val STALL_AFTER_MS = 3_000L

    /**
     * **The progressive part of the retry**, Ivan's ask: *"should we try and restart the video feed
     * if it fails a few times with kind of progressives or something"*.
     *
     * Doubling from [RETRY_AFTER_FAILURE_MS], capped at [RETRY_CAP_MS]: 5 s, 10 s, 20 s, 40 s, 60 s,
     * 60 s… The ladder exists because the two failure populations want opposite things. A one-off
     * stall wants a fast retry — the first one is 5 s and usually ends it. A feed that is genuinely
     * unavailable (camera in playback, aircraft powering down, a source that will not switch) wants
     * us to stop hammering the SDK every 5 s for the rest of a flight, because each attempt is a
     * `setValue(KeyCameraVideoStreamSource)` and a listener re-registration on an aircraft that has
     * other things to do.
     *
     * **It never gives up.** There is no attempt cap, deliberately: a feed that comes back after
     * four minutes is still worth having, and an operator who has stopped getting video has no way
     * to ask for another try except restarting the bridge by hand — which is precisely the manual
     * step this replaces. Capping the *interval* bounds the cost; capping the *count* would not
     * bound anything, it would just decide in advance that the flight is over.
     *
     * @param consecutiveFailures how many failures since the last healthy stretch; 0 for the first.
     */
    fun retryDelayMs(consecutiveFailures: Int): Long {
        if (consecutiveFailures <= 0) return RETRY_AFTER_FAILURE_MS
        // Shift rather than pow, and clamp the shift before it can overflow a Long.
        val doublings = consecutiveFailures.coerceAtMost(16)
        val delay = RETRY_AFTER_FAILURE_MS shl doublings
        return if (delay <= 0 || delay > RETRY_CAP_MS) RETRY_CAP_MS else delay
    }

    /** The longest we ever wait between restart attempts. A minute of silence is enough. */
    const val RETRY_CAP_MS = 60_000L

    /**
     * How long a stream must serve *with frames arriving* before its failure history is forgiven.
     *
     * Without this the ladder is a one-way ratchet: a session that stalls at minute 2 and again at
     * minute 40 would treat the second one as a repeat and wait 10 s, having been perfectly healthy
     * for 38 minutes in between. Sized well above [STALL_AFTER_MS] so a feed that flickers back for
     * one frame and dies again does not reset the ladder and start hammering.
     */
    const val HEALTHY_RESET_MS = 30_000L

    /** Grace before believing `isStreaming() == false` right after a successful start. */
    const val STREAMING_GRACE_MS = 4_000L

    /**
     * How long to wait for `startStream`'s completion callback before treating the
     * attempt as lost.
     *
     * `LiveStreamManager.startStream` hands the work to `DJIExecutor` and then to
     * `MRTCManager.startLiveStream` (`javap -c
     * dji.v5.manager.datacenter.livestream.LiveStreamManager`), so the callback is
     * asynchronous with no documented bound. Without a timeout a callback that
     * never arrives parks the loop in [VideoPhase.STARTING] for the rest of the
     * session with no retry — the exact silence this class exists to prevent.
     */
    const val START_TIMEOUT_MS = 20_000L

    /**
     * Everything the decision depends on, sampled once per tick.
     *
     * @param wanted [VideoStreamer.start] has been called and not cancelled.
     * @param serving `startStream` reported success and we have not torn down.
     * @param startInFlight `startStream` was called and has not called back.
     * @param attemptStartedAtMs elapsed-realtime when the in-flight `startStream`
     *   was issued. Only meaningful when [startInFlight].
     * @param nextAttemptAtMs earliest elapsed-realtime for the next attempt.
     * @param servingSinceMs elapsed-realtime when [serving] became true.
     * @param streamingReported `ILiveStreamManager.isStreaming()` as last polled.
     *   Only meaningful when [serving].
     */
    data class Input(
        val wanted: Boolean,
        val registered: Boolean,
        val aircraftConnected: Boolean,
        val cameraAvailable: Boolean,
        val serving: Boolean,
        val startInFlight: Boolean,
        val nowMs: Long,
        val attemptStartedAtMs: Long = 0,
        val nextAttemptAtMs: Long = 0,
        val servingSinceMs: Long = 0,
        val streamingReported: Boolean = false,
        /** Frames the tap has ever seen this session. 0 means the feed has never proved itself. */
        val framesSeen: Long = 0,
        /** When the last frame arrived, on the same clock as [nowMs]. 0 when none has. */
        val lastFrameAtMs: Long = 0,
        /** Consecutive failures with no healthy stretch between them; drives [retryDelayMs]. */
        val consecutiveFailures: Int = 0,
    )

    sealed interface Decision {
        /**
         * Non-null when the live stream must be torn down before anything else.
         * The string is the reason, and it is logged — "it stopped" without a
         * reason is what makes this path expensive to debug.
         */
        val tearDown: String? get() = null
    }

    /** Nobody wants video. */
    data object Stop : Decision

    /** A precondition is missing. [gate] says which; [tearDown] may be set. */
    data class Wait(val gate: VideoGate, override val tearDown: String? = null) : Decision

    /** All preconditions met, no attempt in flight, backoff elapsed. */
    data object BringUp : Decision

    /** Preconditions met but the backoff after a failure has not elapsed. */
    data object AwaitRetry : Decision

    /** `startStream` is in flight and still within [START_TIMEOUT_MS]. */
    data object AwaitStart : Decision

    /** Serving and healthy; just refresh the counters. */
    data object Poll : Decision

    /** Give up on this attempt, report [message], and retry after the backoff. */
    data class Fail(val message: String, override val tearDown: String? = null) : Decision

    /**
     * The gate order is fixed and load-bearing:
     *
     *  1. **registration** — touching the MSDK earlier silently does nothing.
     *  2. **aircraft** — no product, no camera stream to serve.
     *  3. **camera** — MSDK has to have reported the camera in.
     *
     * A missing gate also tears the stream down rather than leaving a stale
     * `serving` flag, because otherwise the gate coming back would never
     * re-trigger a bring-up.
     */
    fun decide(input: Input): Decision {
        if (!input.wanted) return Stop

        val live = input.serving || input.startInFlight

        if (!input.registered) {
            // Nothing can have been started yet: nothing may touch the SDK before
            // registration, so there is deliberately no teardown here.
            return Wait(VideoGate.WAIT_REGISTRATION)
        }
        if (!input.aircraftConnected) {
            return Wait(VideoGate.WAIT_AIRCRAFT, if (live) "aircraft disconnected" else null)
        }
        if (!input.cameraAvailable) {
            return Wait(VideoGate.WAIT_CAMERA, if (live) "camera no longer reported" else null)
        }

        if (input.startInFlight) {
            return if (input.nowMs - input.attemptStartedAtMs >= START_TIMEOUT_MS) {
                Fail(
                    "startStream never called back within ${START_TIMEOUT_MS / 1000}s",
                    tearDown = "startStream timed out",
                )
            } else {
                AwaitStart
            }
        }

        if (input.serving) {
            val settled = input.nowMs - input.servingSinceMs > STREAMING_GRACE_MS
            if (settled && !input.streamingReported) {
                // startStream() said success, MSDK now says it is not streaming.
                return Fail(
                    "MSDK reports isStreaming=false ${STREAMING_GRACE_MS / 1000}s " +
                        "after a successful startStream",
                )
            }
            // **The stall.** Frames were arriving and stopped, with every other signal still
            // saying everything is fine: aircraft connected, camera reported, listener registered,
            // phase SERVING. Nothing above catches it, because nothing above looks at frames.
            //
            // Deliberately gated on `framesSeen > 0`. A feed that has never delivered anything is
            // a different fault with a different cure — `VideoStatus.verdict` already names it
            // ("SERVING but zero frames … suspect the lens/source"), and restarting on it would
            // loop forever against a camera in playback mode. This rule only fires where the
            // passthrough has already *proved* it works and then went quiet, which is the shape a
            // restart actually fixes.
            val silentFor = input.nowMs - input.lastFrameAtMs
            if (input.framesSeen > 0 && input.lastFrameAtMs > 0 && silentFor >= STALL_AFTER_MS) {
                return Fail(
                    "no frames for ${silentFor / 1000}s after ${input.framesSeen} — " +
                        "the passthrough stalled",
                    tearDown = "video stalled",
                )
            }
            return Poll
        }

        return if (input.nowMs >= input.nextAttemptAtMs) BringUp else AwaitRetry
    }
}
