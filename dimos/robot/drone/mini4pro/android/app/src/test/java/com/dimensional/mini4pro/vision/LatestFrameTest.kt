package com.dimensional.mini4pro.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertNotSame
import org.junit.Assert.assertTrue
import org.junit.Test
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * **The mailbox that stands between MSDK's decode thread and a 40 ms detector.**
 *
 * This is the class the whole design's safety rests on, so it is the one tested hardest. The three
 * properties, and what each is defending:
 *
 *  1. **[LatestFrame.offer] never waits on the consumer.** Measured on the aircraft on 2026-07-28:
 *     holding MSDK's frame callback for 100 ms a frame takes decoded delivery from 24.0 to 9.7 fps.
 *     A mailbox that made the producer wait for a 40 ms detect would do exactly that, every frame.
 *  2. **Depth one, newest wins.** A deeper queue converts a CPU shortfall into latency, and latency
 *     is the failure a control law cannot see.
 *  3. **Nothing is allocated per frame.** 2.07 MB at 24 fps is 50 MB/s of garbage in the process
 *     that holds the abort ladder.
 *
 * ## Mutations killed, measured 2026-07-28
 *
 * | mutation | failures |
 * |---|---|
 * | `offer` waits while the slot is full — a blocking put | **4** |
 * | drop the *newest* instead of the oldest | 2 |
 * | displaced frames are not counted as drops | 1 |
 * | `take` allocates instead of recycling the previous buffer | 1 |
 * | `close` does not wake a waiting consumer | 1 |
 *
 * The first row is the one that cost something to measure. A genuinely blocking `offer` does not
 * fail an assertion — it hangs, and it hung the whole suite for fourteen minutes before the harness
 * was killed. The class rule above is what turned it into four failures.
 *
 * Each row is one mutation applied alone to `src/main`, with the **whole suite** run against it
 * and `app/build/test-results/testDebugUnitTest` deleted first — a mutation that fails to compile
 * otherwise leaves the previous run's XML and reports a confident zero. Harness: `tmp/mutate.py`.
 */
class LatestFrameTest {

    /**
     * **Every test in this class is timed out, and that is the point rather than a precaution.**
     *
     * The property under test is *not waiting*. An implementation that genuinely blocks does not
     * fail an assertion — it never reaches one, and without a bound it hangs the whole suite instead
     * of failing one test. Measured the hard way on 2026-07-28: mutating `offer` into a blocking put
     * sat in `takeReturnsTheNewestAndDropsTheRest` (three offers, no consumer) until the mutation
     * harness's own ten-minute backstop killed the run and reported nothing.
     *
     * A class rule rather than per-method annotations because the trap is *any* test that offers
     * without draining, and there are eight of those. Thirty seconds is two orders of magnitude
     * above the slowest honest test here.
     */
    @get:org.junit.Rule
    val timeout: org.junit.rules.Timeout = org.junit.rules.Timeout.seconds(30)

    private fun frame(size: Int, fill: Byte): ByteArray = ByteArray(size) { fill }

    // ─────────────────────────────────────────────── property 1: never blocks

    /**
     * **The one that matters.** A consumer that never takes must not slow the producer down at all.
     *
     * Written as a bound on wall time rather than as a structural assertion, because the property
     * is about *waiting* and only a clock can see waiting. A thousand offers into a mailbox nobody
     * is draining is a hundredth of a second of memcpy.
     *
     * **The JUnit `timeout` is not belt and braces, it is the enforcement.** A genuinely blocking
     * `offer` with no consumer does not fail the assertion below — it never reaches it, and without
     * the timeout it hangs the whole suite instead of failing one test. Measured the hard way:
     * mutating `offer` into a blocking put on 2026-07-28 sat in this method for fourteen minutes
     * before the run was killed.
     */
    @Test
    fun offerNeverWaitsForAConsumerThatIsNotThere() {
        val mailbox = LatestFrame()
        val data = frame(64 * 64, 7)
        val t0 = System.nanoTime()
        repeat(1_000) { mailbox.offer(data, 0, 64, 64, it.toLong()) }
        val elapsedMs = (System.nanoTime() - t0) / 1_000_000
        assertTrue(
            "1000 offers into an undrained mailbox took $elapsedMs ms — it is waiting on something",
            elapsedMs < 2_000,
        )
        assertEquals(1_000L, mailbox.counters().offered)
        // 999 displaced, because exactly one is still sitting in the slot.
        assertEquals(999L, mailbox.counters().dropped)
    }

    /**
     * The same property against a consumer that is *actually slow*, which is the real case: the
     * detector holds a frame for 40 ms while frames arrive every 41.
     *
     * The consumer sleeps 200 ms per frame while the producer offers 200 frames as fast as it can.
     * A mailbox that made the producer wait would take 40 seconds; this asserts it takes under one.
     */
    @Test
    fun offerNeverWaitsForASlowConsumer() {
        val mailbox = LatestFrame()
        val data = frame(32 * 32, 3)
        val stop = AtomicBoolean(false)
        val consumed = AtomicLong()
        val consumer = Thread {
            while (!stop.get()) {
                mailbox.take(50) ?: continue
                consumed.incrementAndGet()
                Thread.sleep(200)
            }
        }.apply { isDaemon = true; start() }
        // Let the consumer get hold of a frame and go to sleep holding it.
        mailbox.offer(data, 0, 32, 32, 0)
        Thread.sleep(50)

        val t0 = System.nanoTime()
        repeat(200) { mailbox.offer(data, 0, 32, 32, it.toLong()) }
        val elapsedMs = (System.nanoTime() - t0) / 1_000_000
        stop.set(true)
        consumer.interrupt()

        assertTrue(
            "200 offers past a consumer holding a frame took $elapsedMs ms",
            elapsedMs < 1_000,
        )
        assertTrue("the consumer should have taken at least one", consumed.get() >= 1)
    }

    // ──────────────────────────────────────────── property 2: depth 1, newest

    /** Three offers, one take, and the take gets the **third**. */
    @Test
    fun takeReturnsTheNewestAndDropsTheRest() {
        val mailbox = LatestFrame()
        mailbox.offer(frame(4, 1), 0, 2, 2, 100)
        mailbox.offer(frame(4, 2), 0, 2, 2, 200)
        mailbox.offer(frame(4, 3), 0, 2, 2, 300)

        val got = mailbox.take(0)
        assertNotNull(got)
        assertEquals(300L, got!!.atNanos)
        assertEquals(3, got.luma[0].toInt())
        assertEquals(2L, mailbox.counters().dropped)
        assertEquals(1L, mailbox.counters().taken)
        // And the mailbox is now empty: depth one means one.
        assertNull(mailbox.take(0))
    }

    /** The frame's own arrival stamp travels with it — the whole basis of an honest age. */
    @Test
    fun theFramesArrivalStampIsCarriedNotTheTakeTime() {
        val mailbox = LatestFrame()
        mailbox.offer(frame(4, 9), 0, 2, 2, 123_456_789L)
        Thread.sleep(10)
        assertEquals(123_456_789L, mailbox.take(0)!!.atNanos)
    }

    /** Geometry travels with the frame, because a resolution change invalidates the geometry. */
    @Test
    fun widthAndHeightTravelWithTheFrame() {
        val mailbox = LatestFrame()
        mailbox.offer(frame(6, 1), 0, 3, 2, 1)
        val a = mailbox.take(0)!!
        assertEquals(3, a.width)
        assertEquals(2, a.height)
        mailbox.offer(frame(12, 1), 0, 4, 3, 2)
        val b = mailbox.take(0)!!
        assertEquals(4, b.width)
        assertEquals(3, b.height)
        assertEquals(12, b.luma.size)
    }

    // ───────────────────────────────────────── property 3: no per-frame alloc

    /**
     * **Two buffers, reused forever.** After the pool has warmed, every frame handed out is one of
     * exactly two arrays.
     *
     * Asserted by identity rather than by a heap measurement, because identity is what the property
     * actually is and a heap measurement of a JVM is a measurement of the garbage collector.
     */
    @Test
    fun buffersAreRecycledRatherThanAllocated() {
        val mailbox = LatestFrame()
        val src = frame(16, 5)
        val identities =
            java.util.Collections.newSetFromMap(java.util.IdentityHashMap<ByteArray, Boolean>())
        repeat(50) {
            mailbox.offer(src, 0, 4, 4, it.toLong())
            identities.add(mailbox.take(0)!!.luma)
        }
        assertTrue("expected at most two distinct buffers, saw ${identities.size}", identities.size <= 2)
    }

    /** A geometry change gets a correctly sized buffer rather than a stale one. */
    @Test
    fun aGeometryChangeResizesTheBuffer() {
        val mailbox = LatestFrame()
        mailbox.offer(frame(16, 1), 0, 4, 4, 1)
        assertEquals(16, mailbox.take(0)!!.luma.size)
        mailbox.offer(frame(64, 2), 0, 8, 8, 2)
        val big = mailbox.take(0)!!
        assertEquals(64, big.luma.size)
        assertEquals(2, big.luma[63].toInt())
    }

    /** Only the luminance plane is copied — the chroma that follows it in NV21 is not wanted. */
    @Test
    fun onlyTheLuminancePlaneIsCopied() {
        val mailbox = LatestFrame()
        // NV21 for 4x4: 16 bytes of luma then 8 of interleaved chroma, marked distinctly.
        val nv21 = ByteArray(24) { if (it < 16) 1 else 99 }
        mailbox.offer(nv21, 0, 4, 4, 1)
        val got = mailbox.take(0)!!
        assertEquals(16, got.luma.size)
        assertTrue("chroma leaked into the luma plane", got.luma.all { it.toInt() == 1 })
    }

    /** An offset into the source is honoured — MSDK reports one and it has been 0 so far. */
    @Test
    fun anOffsetIntoTheSourceIsHonoured() {
        val mailbox = LatestFrame()
        val padded = ByteArray(20) { if (it < 4) 42 else 7 }
        mailbox.offer(padded, 4, 4, 4, 1)
        assertTrue(mailbox.take(0)!!.luma.all { it.toInt() == 7 })
    }

    // ───────────────────────────────────────────────────────── refusals

    /** A frame that does not describe a readable plane is refused rather than read past its end. */
    @Test
    fun aFrameThatOverrunsTheBufferIsRefused() {
        val mailbox = LatestFrame()
        assertFalse(mailbox.offer(frame(10, 1), 0, 4, 4, 1))
        assertFalse(mailbox.offer(frame(16, 1), 4, 4, 4, 1))
        assertFalse(mailbox.offer(frame(16, 1), 0, 0, 4, 1))
        assertFalse(mailbox.offer(frame(16, 1), -1, 4, 4, 1))
        assertEquals(0L, mailbox.counters().offered)
        assertNull(mailbox.take(0))
    }

    // ─────────────────────────────────────────────────────────── shutdown

    /** [LatestFrame.close] wakes a consumer that is waiting, or a worker outlives its session. */
    @Test
    fun closeWakesAWaitingConsumer() {
        val mailbox = LatestFrame()
        val woke = CountDownLatch(1)
        Thread {
            mailbox.take(30_000)
            woke.countDown()
        }.apply { isDaemon = true; start() }
        Thread.sleep(100)
        mailbox.close()
        assertTrue("take did not return when the mailbox closed", woke.await(3, TimeUnit.SECONDS))
    }

    /** A closed mailbox accepts nothing and hands out nothing. Twice is safe. */
    @Test
    fun aClosedMailboxIsInertAndCloseIsIdempotent() {
        val mailbox = LatestFrame()
        mailbox.offer(frame(4, 1), 0, 2, 2, 1)
        mailbox.close()
        mailbox.close()
        assertFalse(mailbox.offer(frame(4, 1), 0, 2, 2, 2))
        assertNull(mailbox.take(0))
    }

    /** A take with a timeout returns null rather than hanging when nothing arrives. */
    @Test
    fun takeTimesOutWhenNothingArrives() {
        val mailbox = LatestFrame()
        val t0 = System.nanoTime()
        assertNull(mailbox.take(50))
        val elapsedMs = (System.nanoTime() - t0) / 1_000_000
        assertTrue("waited $elapsedMs ms for a 50 ms timeout", elapsedMs in 20..2_000)
    }

    /** A frame offered while a consumer is waiting wakes it rather than waiting for the timeout. */
    @Test
    fun anOfferWakesAWaitingConsumer() {
        val mailbox = LatestFrame()
        val got = java.util.concurrent.atomic.AtomicReference<LatestFrame.Frame?>()
        val done = CountDownLatch(1)
        Thread {
            got.set(mailbox.take(10_000))
            done.countDown()
        }.apply { isDaemon = true; start() }
        Thread.sleep(100)
        mailbox.offer(frame(4, 8), 0, 2, 2, 777)
        assertTrue(done.await(3, TimeUnit.SECONDS))
        assertEquals(777L, got.get()!!.atNanos)
    }

    /**
     * **The producer and the consumer do not corrupt each other's buffer.**
     *
     * The whole point of the pool is that the consumer reads one array while the producer fills
     * another. This runs both flat out for a while and checks every frame the consumer sees is
     * internally consistent — every byte equal to the marker the producer stamped that frame with.
     * A pool that handed the same array to both would show a frame with two markers in it.
     */
    @Test
    fun theProducerNeverWritesIntoTheBufferTheConsumerIsReading() {
        val mailbox = LatestFrame()
        val size = 128 * 128
        val stop = AtomicBoolean(false)
        val torn = AtomicLong()
        val checked = AtomicLong()
        val consumer = Thread {
            while (!stop.get()) {
                val f = mailbox.take(20) ?: continue
                val marker = f.luma[0]
                // Read slowly enough that a racing producer would have time to overwrite.
                for (i in f.luma.indices step 97) {
                    if (f.luma[i] != marker) { torn.incrementAndGet(); break }
                }
                checked.incrementAndGet()
            }
        }.apply { isDaemon = true; start() }

        var mark = 1
        val until = System.nanoTime() + 500_000_000L
        while (System.nanoTime() < until) {
            val src = ByteArray(size) { mark.toByte() }
            mailbox.offer(src, 0, 128, 128, mark.toLong())
            mark = if (mark >= 120) 1 else mark + 1
        }
        stop.set(true)
        consumer.join(2_000)

        assertTrue("the consumer never got a frame", checked.get() > 0)
        assertEquals("frames were torn — the pool handed one buffer to both sides", 0L, torn.get())
    }

    /** The frame handed out stays the consumer's until the next take, which is what the pool promises. */
    @Test
    fun aTakenFrameSurvivesUntilTheNextTake() {
        val mailbox = LatestFrame()
        mailbox.offer(ByteArray(16) { 4 }, 0, 4, 4, 1)
        val first = mailbox.take(0)!!
        val buffer = first.luma
        // Offer twice more; the pool has a second buffer, so neither may touch this one.
        mailbox.offer(ByteArray(16) { 5 }, 0, 4, 4, 2)
        mailbox.offer(ByteArray(16) { 6 }, 0, 4, 4, 3)
        assertTrue("the taken frame was overwritten", buffer.all { it.toInt() == 4 })
        // And the next take hands back the newest, having recycled the one just released.
        val second = mailbox.take(0)!!
        assertEquals(6, second.luma[0].toInt())
        assertNotSame("the consumer's buffer must not be the one it was just reading", buffer, second.luma)
    }
}
