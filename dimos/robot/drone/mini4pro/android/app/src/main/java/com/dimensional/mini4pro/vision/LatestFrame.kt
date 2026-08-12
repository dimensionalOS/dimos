package com.dimensional.mini4pro.vision

/**
 * **A mailbox of depth one that drops the oldest and never makes the producer wait.**
 *
 * This is the single most important class in `vision/`, and it is small on purpose. The phone is the
 * pilot: it holds the abort ladder, it owns the 25 Hz setpoint stream, and it forwards the video a
 * pilot is flying on. Detection costs 40–110 ms a frame against frames arriving every 18 ms. The
 * only shape in which those two facts coexist is one where the expensive side runs on its own thread
 * and the cheap side can *always* return immediately, whatever the expensive side is doing.
 *
 * `record/Recorder` reaches the same shape by the same argument, and `Tap`'s contract states it as a
 * rule for the whole project: *"never block — a full queue drops the entry and records the drop; it
 * does not stall a 25 Hz setpoint loop."* This is that rule for pixels.
 *
 * ## The three properties, and what each one is defending against
 *
 * 1. **[offer] never blocks on the consumer.** It takes a lock, but only ever across a handful of
 *    reference assignments — never across the copy, and never across a detect. There is no path by
 *    which a slow detector can hold the frame callback. This is the property that keeps a detector
 *    from stalling the video.
 * 2. **Depth is one, and the *newest* wins.** A landing controller wants the most recent view of the
 *    world, not the oldest one still queued. A deeper queue would convert a CPU shortfall into
 *    latency — the failure mode where the detector is confidently reporting where the tag was two
 *    seconds ago — and latency is exactly what a control law cannot see. Dropping is the honest
 *    response to being too slow, and it is counted.
 * 3. **No *buffer* is allocated per frame.** Two of them, reused. A 1920×1080 luminance plane is
 *    2.07 MB; at 55 fps that would be 114 MB/s of garbage, on a device where a GC pause lands in the
 *    same process as the abort ladder. A four-field [Frame] header *is* allocated per accepted frame
 *    and is deliberately not pooled — at the 10 Hz cap that is ten small objects a second, and
 *    pooling it would need a third object whose lifetime nobody could state in one sentence.
 *
 * ## Why exactly two buffers is provably enough
 *
 * One producer, one consumer. At any instant a buffer can be in exactly one of three places: held by
 * the consumer (at most one, the frame it was last handed), sitting in the slot (at most one), or
 * being filled by the producer (at most one, and only inside [offer]). With two buffers the producer
 * can always get one, because when the free list is empty the slot must be occupied — and the slot's
 * buffer is the one it is entitled to take, since taking it *is* dropping the oldest. So [offer]
 * never fails for want of a buffer, and the pool never grows.
 *
 * [take] recycles the frame it previously handed out rather than making the consumer release it.
 * A release that has to be remembered is a release that eventually is not, and the consequence here
 * would be a silent allocation per frame.
 */
class LatestFrame {

    /**
     * One frame, valid until the **next** [take] on the same mailbox.
     *
     * [luma] is the mailbox's own buffer, not a copy — that is the whole point of the pool. A
     * consumer that wants to keep pixels past the next take must copy them, and nothing in this
     * project does.
     */
    class Frame internal constructor(
        @JvmField internal var buffer: ByteArray,
        var width: Int,
        var height: Int,
        /**
         * When the frame arrived, `SystemClock.elapsedRealtimeNanos()`, stamped in the source's
         * callback. **Not when it was taken** — the difference between the two is queue latency, and
         * a sighting's honesty about its own age depends on this being the earlier of the two.
         */
        var atNanos: Long,
    ) {
        /** The luminance plane, `width * height` bytes from index 0. */
        val luma: ByteArray get() = buffer
    }

    private val lock = Object()

    /** Buffers nobody is using. See the class KDoc for why two is enough and why it never grows. */
    private val free = ArrayDeque<ByteArray>(2)

    /** The one frame waiting to be taken, or null. */
    private var slot: Frame? = null

    /** The frame [take] last handed out, recycled on the next [take]. */
    private var lent: Frame? = null

    private var closed = false

    /** Frames handed to [offer]. */
    @Volatile var offered: Long = 0L
        private set

    /**
     * Frames that were displaced by a newer one before anybody looked at them.
     *
     * **Expected to be large and that is the design working**, not a fault: the aircraft sends 52–56
     * frames a second and the detector consumes at most 10, so roughly four frames in five are meant
     * to be dropped. It is counted because the ratio is the honest measure of how far behind the
     * detector is running, and because a drop count of *zero* would mean the producer was being made
     * to wait, which is the thing this class exists to prevent.
     */
    @Volatile var dropped: Long = 0L
        private set

    /** Frames actually handed to a consumer. */
    @Volatile var taken: Long = 0L
        private set

    /**
     * Copy `width * height` bytes from [data] at [offset] into the mailbox, displacing anything
     * already waiting.
     *
     * **Called from the frame source's callback thread and must never block on the consumer.**
     * Returns true if the frame was accepted; false only when the mailbox is closed or the arguments
     * do not describe a readable plane.
     */
    fun offer(data: ByteArray, offset: Int, width: Int, height: Int, atNanos: Long): Boolean {
        val need = width * height
        if (need <= 0 || offset < 0 || offset + need > data.size) return false

        // Phase one: get a buffer to fill. Under the lock, but only across list operations.
        val target: ByteArray
        synchronized(lock) {
            if (closed) return false
            offered++
            val reused = free.removeLastOrNull()
            target = if (reused != null && reused.size == need) {
                reused
            } else if (reused != null) {
                // Geometry changed. The old buffer is the wrong size and is not kept: a resolution
                // change invalidates far more than a buffer, and holding a stale one to save an
                // allocation on a once-a-session event would be an optimisation of nothing.
                ByteArray(need)
            } else {
                // Free list empty ⇒ the slot is occupied (see the KDoc's counting argument). Taking
                // its buffer *is* dropping the oldest, which is the policy.
                val stale = slot
                slot = null
                if (stale != null) {
                    dropped++
                    if (stale.buffer.size == need) stale.buffer else ByteArray(need)
                } else {
                    // Reachable only if the pool has not been seeded yet — the very first offer.
                    ByteArray(need)
                }
            }
        }

        // Phase two: the copy, **outside the lock**. This is the expensive part of the callback
        // (~0.3 ms at 1080p on the measured path) and holding a lock across it would let a consumer
        // that happened to be between takes contend with the video thread for no reason.
        System.arraycopy(data, offset, target, 0, need)

        // Phase three: install, displacing whatever arrived while we were copying.
        synchronized(lock) {
            if (closed) {
                free.addLast(target)
                return false
            }
            slot?.let {
                dropped++
                free.addLast(it.buffer)
            }
            slot = Frame(target, width, height, atNanos)
            (lock as Object).notifyAll()
        }
        return true
    }

    /**
     * The newest frame, waiting up to [timeoutMs] for one, or null if none arrived or the mailbox
     * closed.
     *
     * Recycles the frame handed out by the previous call, so the consumer holds exactly one frame at
     * a time and never has to remember to give it back. The returned frame's [Frame.luma] stays
     * valid until the next call to this method.
     */
    fun take(timeoutMs: Long): Frame? {
        synchronized(lock) {
            lent?.let { free.addLast(it.buffer); lent = null }
            if (closed) return null
            if (slot == null && timeoutMs > 0) {
                // A timed wait rather than an untimed one so a worker can notice interruption and
                // shutdown without needing anything to wake it deliberately.
                runCatching { (lock as Object).wait(timeoutMs) }
                    .onFailure { Thread.currentThread().interrupt(); return null }
            }
            val f = slot ?: return null
            slot = null
            lent = f
            taken++
            return f
        }
    }

    /**
     * Wake any waiting consumer and refuse further frames.
     *
     * Idempotent. A mailbox is closed when the recogniser stops, and a consumer blocked in [take]
     * has to come out of it or the worker thread outlives the session that owned it.
     */
    fun close() {
        synchronized(lock) {
            if (closed) return
            closed = true
            slot = null
            lent = null
            free.clear()
            (lock as Object).notifyAll()
        }
    }

    /** For the status line and the tests. Cheap, and consistent within one call. */
    fun counters(): Counters = synchronized(lock) { Counters(offered, dropped, taken) }

    data class Counters(val offered: Long, val dropped: Long, val taken: Long)
}
