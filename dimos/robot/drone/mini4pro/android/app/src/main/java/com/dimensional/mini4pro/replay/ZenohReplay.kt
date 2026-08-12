package com.dimensional.mini4pro.replay

import com.dimensional.mini4pro.zenoh.Gate
import com.dimensional.mini4pro.zenoh.LcmTime
import com.dimensional.mini4pro.zenoh.OdomDatum
import com.dimensional.mini4pro.zenoh.Withheld
import com.dimensional.mini4pro.zenoh.ZenohChannel
import com.dimensional.mini4pro.zenoh.ZenohEmission

/**
 * Drives [ZenohEmission] over a replayed flight and reports **what it published and what it
 * withheld, per channel, with the reason** — the offline half of "record all the messages coming
 * from a drone so that we can replay them into Zenoh".
 *
 * There is no Zenoh session here and no publisher, and there does not need to be: since
 * 2026-07-27 the live one exists (`zenoh/ZenohPublisher`) and **both drive the same decision
 * object**. What used to be this file's own gating logic now lives in [ZenohEmission], which is
 * what makes the counts below a statement about the live publisher rather than a parallel
 * re-derivation of it. This class is the timeline, the tally and the datum rule; the judgement is
 * shared.
 *
 * ## Why the reasons are worth as much as the counts
 *
 * [ZenohTelemetryEncoder][com.dimensional.mini4pro.zenoh.ZenohTelemetryEncoder] returns null
 * rather than a zero-filled message, so a channel that publishes nothing and a channel that has no
 * data look identical from the outside. On the reference flight that difference is the finding:
 * `odom` is withheld for 78 % of the samples, and the reason is not a missing altitude — the
 * aircraft was at a perfectly well-known 29.3 m the whole time — but a **stale** one, because
 * `KeyAltitude` arrives in bursts every five or six seconds against `Signal.ALTITUDE`'s 1 s limit.
 * Counts alone would say "the encoder is dropping messages"; reasons say which limit, on which
 * signal, from which key.
 *
 * ## The one thing a replay decides that a live publisher decides differently
 *
 * [takeoffDatum] scans the whole record for the first sample with the motors on. **A live
 * publisher cannot do that** — it has no later samples — so `zenoh/ZenohTelemetryPump` latches the
 * origin causally instead, and its KDoc states the deviation. Everything else in this file is a
 * loop and a tally.
 */
object ZenohReplay {

    /** One channel's tally over a whole replay. */
    data class ChannelResult(
        val channel: ZenohChannel,
        val published: Int,
        val reasons: Map<Withheld, Int>,
    ) {
        val samples: Int get() = published + reasons.filterKeys { it != Withheld.PUBLISHED }.values.sum()
        val withheld: Int get() = samples - published
        fun fraction(): Double = if (samples == 0) 0.0 else published.toDouble() / samples
    }

    /**
     * A whole replay's outcome: the datum that was recorded, the per-channel tallies, and the
     * encoded frames when the caller asked for them.
     */
    class Result(
        val datum: OdomDatum?,
        /** The sample index and time the datum was taken at, or null when none was. */
        val datumAtSeconds: Double?,
        val channels: Map<ZenohChannel, ChannelResult>,
        val sampleCount: Int,
        /** `(tSeconds, channel, bytes)` in emission order — empty unless [Options.encodeFrames]. */
        val frames: List<Frame>,
    ) {
        operator fun get(channel: ZenohChannel): ChannelResult =
            channels[channel] ?: ChannelResult(channel, 0, emptyMap())
    }

    /** One encoded LCM frame, ready for a publisher that now exists. */
    data class Frame(val tSeconds: Double, val channel: ZenohChannel, val bytes: ByteArray) {
        override fun equals(other: Any?): Boolean =
            other is Frame && tSeconds == other.tSeconds && channel == other.channel &&
                bytes.contentEquals(other.bytes)

        override fun hashCode(): Int =
            (31 * tSeconds.hashCode() + channel.hashCode()) * 31 + bytes.contentHashCode()
    }

    data class Options(
        /**
         * Encode each published message to LCM bytes and keep them in [Result.frames].
         *
         * Off by default: a 371 s flight at 5 Hz is ~8 000 frames and several megabytes, and a
         * test that only wants the counts should not pay for them.
         */
        val encodeFrames: Boolean = false,
    )

    /**
     * The takeoff datum for this flight, or null when the record never held a fix good enough to
     * anchor a local frame.
     *
     * "Takeoff" is read off the record rather than assumed: the first sample at which DJI said the
     * motors were on, which on the reference flight is t=26.6 s, three seconds before `isFlying`
     * and while the aircraft is still on the pad — which is exactly where an `odom` origin
     * belongs. If the motors never came on (a ground session), the first sample with a usable fix
     * is used instead, so a bench replay still has a frame.
     *
     * **This is the one rule the live publisher cannot copy**, because both passes look ahead. See
     * the class doc.
     */
    fun takeoffDatum(samples: List<ReplaySample>): Pair<OdomDatum, Double>? {
        for (s in samples) {
            if (s.state.motorsOn != true) continue
            OdomDatum.atTakeoff(s.state)?.let { return it to s.tSeconds }
        }
        for (s in samples) {
            OdomDatum.atTakeoff(s.state)?.let { return it to s.tSeconds }
        }
        return null
    }

    /**
     * Runs the encoder over [samples].
     *
     * The stamp on every message is the sample's own wall-clock instant, from the record's header
     * — not the replay machine's clock. A frame written to disk here therefore carries the time
     * the aircraft was actually there, which is what makes the output a fixture rather than a
     * recording of when it was replayed.
     */
    fun run(samples: List<ReplaySample>, options: Options = Options()): Result {
        val found = takeoffDatum(samples)
        val datum = found?.first
        val tally = ZenohChannel.entries.associateWith { HashMap<Withheld, Int>() }
        val frames = ArrayList<Frame>(if (options.encodeFrames) samples.size * 6 else 0)

        for (sample in samples) {
            val stamp = sample.unixMillis?.let { LcmTime.ofEpochSeconds(it / 1000.0) } ?: LcmTime.ZERO
            for (e in ZenohEmission.emit(sample.state, datum, stamp, encode = options.encodeFrames)) {
                val m = tally.getValue(e.channel)
                m[e.reason] = (m[e.reason] ?: 0) + 1
                e.bytes?.let { frames.add(Frame(sample.tSeconds, e.channel, it)) }
            }
        }

        val channels = ZenohChannel.entries.associateWith { ch ->
            val m = tally.getValue(ch)
            ChannelResult(ch, m[Withheld.PUBLISHED] ?: 0, m.toMap())
        }
        return Result(datum, found?.second, channels, samples.size, frames)
    }

    // ── the gates, per sample, for the coverage report and its tests ─────────
    //
    // Thin wrappers over [ZenohEmission]'s, taking a [ReplaySample] so callers that already hold
    // one need not unwrap it. There is no second copy of the rule behind them.

    fun positionReason(sample: ReplaySample, gate: Gate = Gate.FRESH): Withheld =
        ZenohEmission.positionReason(sample.state, gate)

    fun altitudeReason(sample: ReplaySample, gate: Gate = Gate.FRESH): Withheld =
        ZenohEmission.altitudeReason(sample.state, gate)

    fun attitudeReason(sample: ReplaySample, gate: Gate = Gate.FRESH): Withheld =
        ZenohEmission.attitudeReason(sample.state, gate)

    /**
     * [gate] is accepted and ignored: velocity has no held form, deliberately, and the parameter
     * stays so that all four gates read the same at every call site. [ZenohEmission.velocityReason]
     * has the argument.
     */
    @Suppress("UNUSED_PARAMETER")
    fun velocityReason(sample: ReplaySample, gate: Gate = Gate.FRESH): Withheld =
        ZenohEmission.velocityReason(sample.state)
}
