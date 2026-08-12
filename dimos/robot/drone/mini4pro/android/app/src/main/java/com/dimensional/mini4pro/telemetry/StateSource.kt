package com.dimensional.mini4pro.telemetry

/**
 * **Where a downstream's `AircraftState` comes from** — the aircraft, or a file.
 *
 * Ivan's instruction, verbatim: *"replay should just pass downstream the same way actual drone
 * data is, just replay reads from a file, should be a clean implementation."* This is that seam,
 * and it is one object because there were already **two** copies of the live read — `Bridge`'s
 * private `aircraftState()` and `ZenohBus.sample`'s own line, identical to the character. Two
 * copies of a rule is one too many, and a replay that had to be wired into both would have been
 * wired into one.
 *
 * Nothing here knows what a recording is. It holds two suppliers and answers *which one a given
 * sink reads*; the replay controller decides what the recording's supplier returns and the UI
 * decides whether one is installed at all. No Android, no DJI, no clock: the whole of it is a
 * JVM test.
 *
 * ## The two halves of the fan-out, and why only one of them is switchable
 *
 * The bridge's `aircraftState()` used to feed everything. It does not any more, and the split is
 * the safety property of this whole feature:
 *
 * - **Outbound** — MAVLink telemetry, the Zenoh pump — reads [read], which honours a replay.
 *   These paths describe an aircraft to somebody else. Describing a recorded one is the feature.
 * - **Control** — `guided/`, `mission/`, the command dispatcher, the tag detector, the mission
 *   upload's AMSL datum — reads [liveState], which **cannot** see a replay. There is no
 *   parameter, no flag and no ordering that makes it return recorded data; the feed is simply
 *   not consulted.
 *
 * That is the structural half. The behavioural half is `replay/ReplayAdmission`, which refuses
 * to publish at all while an aircraft is connected or the interlock is on — so the two halves
 * are not merely separated, they are never both live. `replay/ReplayPublishTest` asserts the
 * structural one directly: with a feed installed and publishing on **both** sinks, [liveState]
 * still returns the live value.
 *
 * ## Why a held global rather than a constructor parameter
 *
 * The consumers are two long-lived loops on two threads — `Bridge.tick` at 5 Hz and
 * `ZenohBus`'s sampler at 5 Hz — started before any recording is opened and not restarted when
 * one is. `ZenohBus.gimbalSource` and `Recorder.gimbalSource` are the same shape for the same
 * reason, and this file follows them deliberately rather than inventing a third pattern.
 */
object StateSource {

    /** A downstream that describes the aircraft to somebody else. Each is switched separately. */
    enum class Sink {
        /** The MAVLink telemetry stream, and therefore QGroundControl's picture. */
        MAVLINK,

        /** The Zenoh bus, and therefore DiMOS. */
        ZENOH,
    }

    /** What a sink is reading right now. For the screen, and for the record's own event lines. */
    enum class Kind { LIVE, REPLAY }

    /**
     * A recording, offered to zero, one or both sinks.
     *
     * @param read the cursor's current state, or null when the recording has not reached a
     *   sample yet. Null becomes an all-null [AircraftState] rather than a held previous one:
     *   before the first sample there is no reading, and this project's oldest rule is that no
     *   reading is null and never a stale value wearing a fresh face.
     */
    class Feed(
        /** The recording's own name, for the banner, the announcement and the record. */
        val name: String,
        val toMavlink: Boolean,
        val toZenoh: Boolean,
        val read: () -> AircraftState?,
    ) {
        /** True when at least one sink is being fed. A feed with neither is a picture only. */
        val publishing: Boolean get() = toMavlink || toZenoh

        fun feeds(sink: Sink): Boolean = when (sink) {
            Sink.MAVLINK -> toMavlink
            Sink.ZENOH -> toZenoh
        }
    }

    /**
     * The aircraft, installed once by `Bridge`. Null until then, and **never cleared**.
     *
     * Not cleared on `Bridge.stop()` because it is a pure read-through — it retains nothing, and
     * it already answers a stopped or unregistered SDK with the same all-null state a caller
     * would have built for itself. Clearing it would only open a window in which a live read
     * silently became an empty one for a different reason.
     */
    @Volatile
    var live: (() -> AircraftState)? = null

    @Volatile
    private var feed: Feed? = null

    /** The recording currently offered to any sink, or null. */
    fun feed(): Feed? = feed

    /** True while at least one sink is reading a recording. The banner's condition. */
    val publishing: Boolean get() = feed?.publishing == true

    /**
     * Installs, replaces or removes the recording. Null ends it.
     *
     * Idempotent and instantaneous: the next tick of either loop reads the new answer. There is
     * no handshake to get wrong because there is no state to hand over — a sink that changes
     * source mid-stream simply describes a different aircraft from the next message on, which is
     * exactly what a subscriber sees when a real aircraft is switched off and another switched
     * on.
     */
    fun install(f: Feed?) {
        feed = f
    }

    /** Whether [sink] is currently reading a recording. */
    fun kind(sink: Sink): Kind =
        if (feed?.feeds(sink) == true) Kind.REPLAY else Kind.LIVE

    /**
     * What [sink] should describe: the recording when one is installed for it, the aircraft
     * otherwise.
     *
     * An all-null state is the answer when there is nothing to read — no live supplier, or a
     * cursor that has not reached a sample. That is the same answer both call sites gave before
     * this object existed, and the encoders turn it into sentinels and withheld messages rather
     * than into zeros.
     */
    fun read(sink: Sink): AircraftState {
        val f = feed
        if (f != null && f.feeds(sink)) return f.read() ?: AircraftState()
        return liveState()
    }

    /**
     * **The aircraft, always.** The command path's read, and the one that cannot be replayed.
     *
     * Deliberately not `read(sink)` with a third sink value: a `Sink.COMMAND` would be a thing
     * somebody could pass a feed for. This has no parameter at all, so there is nothing to get
     * wrong at a call site and nothing to add to a `Feed` that would change what it returns.
     */
    fun liveState(): AircraftState = live?.invoke() ?: AircraftState()
}
