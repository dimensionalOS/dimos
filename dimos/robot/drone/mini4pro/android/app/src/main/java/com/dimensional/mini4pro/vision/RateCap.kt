package com.dimensional.mini4pro.vision

/**
 * **How often the detector is allowed to look**, decided on the producer's side.
 *
 * The aircraft sends 24–56 frames a second and detection costs 35–110 ms of one to three cores.
 * Detection is therefore *sampled*, never per-frame, and this is the thing that samples it.
 *
 * ## Why the cap is here and not in the consumer
 *
 * A frame rejected by [admit] costs one clock read and one comparison. A frame accepted and then
 * discarded by a worker has already cost the 2.07 MB copy out of MSDK's buffer — 0.4 ms measured on
 * the aircraft, which is small but is paid on the frame-delivery thread, the one thread this design
 * promises never to make anything wait on. Deciding early is free; deciding late is not.
 *
 * ## Why 10 Hz
 *
 * Ivan's cap, and the measurements support it from both ends. At 2 threads the detector costs 35 ms
 * a frame on the scene measured in flight and ~86 ms on the cluttered terrace the offline comparison
 * used, so 100 ms is a period the expensive case still fits inside. And a 25 Hz control loop reading
 * a sensor that updates at 10 Hz is already spending most ticks on a value it has seen before;
 * paying three cores to make that 20 Hz would buy nothing a controller could use.
 *
 * ## The interval is a floor, not a schedule
 *
 * This admits a frame when at least [minIntervalNanos] have passed since the last one it admitted —
 * it does not try to hit 10.0 Hz exactly by catching up. Catching up is how a rate limiter turns a
 * momentary stall into a burst, and a burst of detections is precisely what the CPU budget cannot
 * afford. The achieved rate is therefore *at most* the nominal one and, with frames arriving every
 * 41 ms, lands at whatever multiple of the frame interval fits — 41.6 ms × 3 = 8.0 Hz for a 10 Hz
 * cap on a 24 fps stream. **That is expected**, and it is why the rate is reported from counters
 * rather than from the setting.
 *
 * Pure, and deliberately not thread-safe: it is called from exactly one thread, the frame source's,
 * and a lock on the video path to protect one long would be a worse trade than an occasional
 * double-admit could ever be. Nothing here is shared.
 */
class RateCap(hz: Double) {

    /** Nanoseconds that must pass between two admitted frames. */
    val minIntervalNanos: Long = if (hz <= 0.0) 0L else (1_000_000_000.0 / hz).toLong()

    private var lastNanos = Long.MIN_VALUE

    var admitted: Long = 0L
        private set

    var refused: Long = 0L
        private set

    /**
     * True if a frame arriving at [nowNanos] may be worked on.
     *
     * The first frame is always admitted: a cap is a limit on how often, not a delay before
     * starting, and a landing that had to wait 100 ms for its first look would be paying for
     * nothing.
     */
    fun admit(nowNanos: Long): Boolean {
        if (minIntervalNanos <= 0L) {
            admitted++
            return true
        }
        // `Long.MIN_VALUE` as "never" rather than 0: `elapsedRealtimeNanos` is small just after a
        // boot, and a subtraction against 0 there would be a real comparison rather than a
        // first-frame one. Compared explicitly so no arithmetic can overflow into a wrong answer.
        if (lastNanos != Long.MIN_VALUE && nowNanos - lastNanos < minIntervalNanos) {
            refused++
            return false
        }
        lastNanos = nowNanos
        admitted++
        return true
    }

    /** Forget the last admission, so the next frame is treated as the first. */
    fun reset() {
        lastNanos = Long.MIN_VALUE
    }

    companion object {
        /** Ivan's cap. See the class KDoc for the two measurements that agree with it. */
        const val DEFAULT_HZ = 10.0
    }
}
