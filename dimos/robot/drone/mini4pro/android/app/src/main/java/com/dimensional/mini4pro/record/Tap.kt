package com.dimensional.mini4pro.record

import io.dronefleet.mavlink.MavlinkMessage

/**
 * Who is on the other end of a boundary. Two peers, and the whole point of naming them is that
 * this project spent weeks treating only one of them as a boundary at all.
 */
enum class Peer {
    /** A ground station: QGroundControl today, a DiMOS Zenoh node next. */
    GCS,

    /** The aircraft, through the MSDK. */
    AIRCRAFT,
}

/** Which way the traffic is going, from this app's point of view. */
enum class Way { OUT, IN }

/**
 * **Every boundary at which this app exchanges traffic with something outside itself**, and the
 * flight-record entry each one lands in.
 *
 * This enum is the registry Ivan's requirement turns on:
 *
 * > *"Recorder should be a general thing that all messages pass through. Structurally, we
 * > shouldn't be able to hook up something to ground control or Zenoh without it passing through
 * > a recording system."*
 *
 * ## What it buys, and what it does not
 *
 * [entryKind] is an **exhaustive `when` with no `else`**, so a new channel does not compile until
 * somebody has decided which line of the record it produces. That closes the "added a channel and
 * forgot the log" half. The other half — *added a transport and never declared a channel* — cannot
 * be closed by a type, because the missing thing is an absence; it is closed by
 * `RecordingSeamTest`, which walks the source tree, finds every file that touches a socket or the
 * MSDK, and fails when one appears that this enum does not account for.
 *
 * ## The proof that this was needed
 *
 * The gimbal had been aiming a real camera for weeks and **not one gimbal reading existed in any
 * flight record** (`replay/ReplayCoverage.GIMBAL`). Nobody decided that. It is what happens when
 * recording is something you remember to call rather than something traffic passes through. The
 * same shape was live on the aircraft-outbound side: `stick_cmd` recorded the 25 Hz setpoint
 * stream in detail while *"we asked DJI to take off and DJI answered
 * `CONTROL_AUTH_HAS_NO_CONTROL_AUTH`"* existed only as prose in an `event`, if at all.
 *
 * ## Volume, stated because this seam makes adding traffic easy
 *
 * A record is already ~8 MB per six minutes. The two channels added with this enum are both
 * deliberately cheap and neither is a stream:
 *
 * | channel | cadence | measured/estimated cost |
 * |---|---|---|
 * | [AIRCRAFT_GIMBAL] | on change past a 0.5° deadband, ≤5 Hz, 1 Hz heartbeat | ~60 B/line; ≤1.4 % of the file at the 5 Hz ceiling, ~0.3 % in practice |
 * | [AIRCRAFT_ACTION] | discrete actions only — two lines per action | ~55 B/line; a whole M2 session is tens of lines |
 *
 * **[AIRCRAFT_STICK] is deliberately not re-wrapped.** It is 1829 lines in six minutes already and
 * `LogEntry.StickCmd` records more about each send than a call record could; putting a second
 * per-tick entry beside it would double the largest thing in the file to say less.
 */
enum class Channel(val peer: Peer, val way: Way) {

    /** Every datagram that leaves the MAVLink socket, tapped at the socket. */
    GCS_MAVLINK_OUT(Peer.GCS, Way.OUT),

    /** Every MAVLink message that arrives, with its genuine wire bytes. */
    GCS_MAVLINK_IN(Peer.GCS, Way.IN),

    /**
     * The camera passthrough's lifecycle — **not its payload**. 5 Mbit/s of H.264 does not belong
     * in a 32 MB flight record, and the question a record must answer about video is "was it up,
     * and if not why", which is what the phase trail says.
     */
    GCS_VIDEO(Peer.GCS, Way.OUT),

    /**
     * One discrete thing we asked the aircraft to do, and DJI's answer, correlated —
     * `LogEntry.DjiCall`. Takeoff, land, confirm-landing, return, virtual-stick enable/disable,
     * gimbal rotate, simulator start/stop.
     */
    AIRCRAFT_ACTION(Peer.AIRCRAFT, Way.OUT),

    /** The 25 Hz virtual-stick setpoint stream — `LogEntry.StickCmd`, already complete. */
    AIRCRAFT_STICK(Peer.AIRCRAFT, Way.OUT),

    /** The fast half of `AircraftState`, sampled. */
    AIRCRAFT_STATE(Peer.AIRCRAFT, Way.IN),

    /** The slow half of `AircraftState`, plus the keys `Recorder` subscribes to itself, on change. */
    AIRCRAFT_FIELD(Peer.AIRCRAFT, Way.IN),

    /** DJI's own warnings — device health, wind, and every later source — on change. */
    AIRCRAFT_WARN(Peer.AIRCRAFT, Way.IN),

    /**
     * Where the camera is pointing. **The gap this seam was built to close** — see
     * `replay/ReplayCoverage.GIMBAL`, and `Recorder.gimbal` for the deadband and the key.
     */
    AIRCRAFT_GIMBAL(Peer.AIRCRAFT, Way.IN),

    /**
     * **One encoded video frame arriving from the aircraft** — the index line, not the bytes.
     *
     * The bytes go to a sidecar beside the JSONL; this channel produces the line that says which
     * part and which offset. See [LogEntry.Frame] and `docs/apriltag-landing-recording.md` §2 for
     * why the payload is deliberately not in the record while its *arrival* is.
     *
     * Distinct from [GCS_VIDEO], which is the passthrough's **lifecycle** and always was: "was the
     * stream up, and if not why" is a different question from "when did this frame arrive".
     */
    AIRCRAFT_VIDEO_FRAME(Peer.AIRCRAFT, Way.IN),

    /**
     * **One AprilTag the on-board detector saw** — `LogEntry.Tag`.
     *
     * Inbound from the aircraft, because that is where the photons came from, even though the
     * detection happened on the phone. The alternative reading — that a sighting is the phone's own
     * derived state and crosses no boundary — was rejected for the reason [AIRCRAFT_GIMBAL] exists:
     * a channel that is *derived* is exactly the kind that gets forgotten, and this one is the only
     * evidence a tag was ever seen.
     */
    AIRCRAFT_TAG(Peer.AIRCRAFT, Way.IN),
    ;

    /**
     * The `k` of the flight-record line this channel produces.
     *
     * Exhaustive with no `else` on purpose: a new [Channel] is a compile error here until its
     * author has named the entry it lands in, which is the one question that gets forgotten.
     */
    val entryKind: String
        get() = when (this) {
            GCS_MAVLINK_OUT -> LogEntry.KIND_MAV_OUT
            GCS_MAVLINK_IN -> LogEntry.KIND_MAV_IN
            GCS_VIDEO -> LogEntry.KIND_EVENT
            AIRCRAFT_ACTION -> LogEntry.KIND_DJI_CALL
            AIRCRAFT_STICK -> LogEntry.KIND_STICK_CMD
            AIRCRAFT_STATE -> LogEntry.KIND_DJI_STATE
            AIRCRAFT_FIELD -> LogEntry.KIND_DJI_FIELD
            AIRCRAFT_WARN -> LogEntry.KIND_DJI_WARN
            AIRCRAFT_GIMBAL -> LogEntry.KIND_GIMBAL
            AIRCRAFT_VIDEO_FRAME -> LogEntry.KIND_FRAME
            AIRCRAFT_TAG -> LogEntry.KIND_TAG
        }

    companion object {
        /** The boundaries a *transport class* crosses, i.e. the ones [Tap] has a verb for. */
        val TAPPED: List<Channel> =
            listOf(GCS_MAVLINK_OUT, GCS_MAVLINK_IN, AIRCRAFT_ACTION, AIRCRAFT_TAG)
    }
}

/**
 * **The seam.** A transport is *constructed with* one of these; it does not call a recorder
 * beside itself, it hands traffic through on the way past.
 *
 * The distinction is the whole design. `MavlinkLink` used to expose `var onSent` and rely on
 * `Bridge` remembering to install a tap into it, and `Bridge.onInbound` used to open with a
 * `Recorder.mavIn(message)` that nothing obliged it to keep. Both worked. Both were conventions,
 * and the gimbal is the standing proof of what conventions are worth here. Now the tap is a
 * non-null constructor parameter, so a link that records nothing cannot be built, and the recorder
 * call lives inside the class that owns the wire.
 *
 * Modelled on [com.dimensional.mini4pro.command.Announcer], which yesterday made "a second
 * interface must not be able to be blind" structural for outbound sentences by the same move: the
 * fan-out is the thing you are given, not the thing you remember to call.
 *
 * ## Three non-negotiables every implementation inherits
 *
 * 1. **Never throw at the caller.** These methods run on DJI's callback thread, the `mavlink-rx`
 *    thread and inside the send path. An evidence problem must never become a flight problem, so
 *    containment is the implementation's job and not the transport's — a transport that had to
 *    wrap every tap call in a `try` would eventually forget one.
 * 2. **Never block.** `Recorder` reaches a bounded `offer` onto a writer thread and nothing else.
 *    A full queue drops the entry and records the drop; it does not stall a 25 Hz setpoint loop.
 * 3. **Never change the wire.** Same messages, same timing, same bytes. Every method here returns
 *    `Unit` (or a handle that only records), so there is no path by which a tap can alter what a
 *    transport does.
 */
interface Tap {

    /**
     * One datagram that actually left the MAVLink socket, tapped at the socket rather than at each
     * send call — so sequence numbers in the log are real and outbound loss is analysable.
     */
    fun gcsOut(datagram: ByteArray)

    /** One MAVLink message that arrived, with its genuine wire bytes. */
    fun gcsIn(message: MavlinkMessage<*>)

    /**
     * **We are about to ask the aircraft to do one discrete thing.** Returns the handle the
     * answer is reported through, so the ask and its answer are one correlated record rather than
     * two lines a reader has to pair by eye.
     *
     * Call it *before* the SDK call, not after: a `performAction` that throws, or a process that
     * dies between the call and the callback, must still leave the ask on the record. The answer
     * is optional by necessity — on 2026-07-26 a `performAction` on a healthy connected aircraft
     * invoked **neither** callback, four times
     * (`docs/measurements/2026-07-26-m2-first-command.md`) — which is exactly why the ask is a
     * line of its own and why an unanswered one is swept into a `none` after
     * [DjiCalls.Config.unansweredAfterMs] rather than left as an absence.
     *
     * @param op one of [DjiOp]. A short stable token, because a reader greps for it.
     * @param argsJson a **complete** pre-rendered JSON object describing what was asked, or null —
     *   `JsonObject.render { … }`, the same shape `LogEntry.Event.dataJson` takes. `putRaw`
     *   splices the string in verbatim, so bare members would produce an unparseable line.
     *   Rendered by the caller because only the caller knows the units.
     * @param urgent whether the ask and its answer are `fsync`ed immediately. True for the things
     *   that leave the ground and the things that take control; false for repeated aiming, which
     *   would otherwise put a flash sync on a 5 Hz stream. **A refusal is always urgent whatever
     *   this says** — see [DjiCalls].
     */
    fun aircraftOut(op: String, argsJson: String? = null, urgent: Boolean = true): Call

    /**
     * **One tag the on-board detector saw.** Called on the detector's worker thread, once per
     * detected frame, at most ten times a second.
     *
     * Deliberately a verb here rather than a sampler on the recorder, unlike the gimbal reading, and
     * the difference is what the two things are. A gimbal angle is *state*: it has a value at every
     * instant and sampling it at the recorder's own rate loses nothing. A sighting is an *event*
     * that exists for one frame and then does not — sampling would mean discarding most of them,
     * and the ones discarded would be the ones that decide a detection-rate curve.
     *
     * `vision/RecordedTagSink` is the decorator that makes calling this unavoidable, in exactly the
     * shape `gimbal/RecordedGimbalPort` and `light/RecordedLightPort` use: the recogniser is
     * *constructed with* the sink, so a recogniser that records nothing cannot be built.
     *
     * @param sighting what was seen, in pixels and in the camera's frame.
     * @param fix where it is in `drone/world`, or null when it could not be worked out — no
     *   position, no heading, no commanded camera angle, or a camera not near nadir. Null is a real
     *   state and must be recorded as one rather than as a zero.
     * @param latched true on the single frame at which this became the flight's latched tag.
     */
    fun tagSeen(
        sighting: com.dimensional.mini4pro.vision.TagSighting.Sighting,
        fix: com.dimensional.mini4pro.vision.TagFix?,
        latched: Boolean,
    )

    /**
     * One outstanding ask. Exactly one of the three methods should be called, once; a second call
     * is ignored rather than double-recorded, because DJI's callbacks are not ours to trust.
     */
    interface Call {
        /** DJI accepted the request. Never that the aircraft did the thing. */
        fun accepted()

        /** DJI refused, with its `IDJIError` name verbatim — the string an operator will search. */
        fun refused(error: String)

        /**
         * A **synchronous** call has returned: [error] is the throwable's message, or null when it
         * returned cleanly. For the void SDK setters (`setVirtualStickAdvancedModeEnabled`,
         * `sendVirtualStickAdvancedParam`) a clean return is the only acceptance signal there is,
         * and nothing may read it as the aircraft honouring anything.
         */
        fun settled(error: String?)
    }
}

/**
 * The `op` tokens of [Tap.aircraftOut]. Strings on the wire so a new one is additive, constants
 * here so the record and the reader cannot drift apart.
 *
 * One per **SDK surface we can command**, named for what an operator would call it rather than for
 * DJI's key, because the key is already recoverable from the port that owns it.
 */
object DjiOp {
    const val TAKEOFF = "takeoff"
    const val GO_HOME = "go_home"
    const val LAND = "land"
    const val CONFIRM_LANDING = "confirm_landing"

    /**
     * `performAction(FC.KeyStopAutoLanding)` — Stage C rule 1's withdrawal of a committed
     * autoland. On the record because whether DJI honours a stop mid-landing is one of the
     * facts the Stage C flights measure, and the ask/answer pair is how it gets measured.
     */
    const val STOP_LANDING = "stop_landing"
    const val CANCEL_LISTENS = "cancel_listens"

    const val SIM_START = "sim_start"
    const val SIM_STOP = "sim_stop"

    /** `gimbal/`: `performAction(Gimbal.KeyRotateByAngle)`. */
    const val GIMBAL_ROTATE = "gimbal_rotate"

    /**
     * `light/`: `setValue(FlightAssistant.KeyBottomAuxiliaryLightMode)`.
     *
     * Recorded because the lamp exists to answer a question — does it help a camera see a tag
     * after dark? — and that question is answered by comparing detections against when the lamp
     * was actually on, on the same clock as the frames.
     */
    const val LIGHT_MODE = "light_mode"

    /** `guided/`: `enableVirtualStick` / `disableVirtualStick`. */
    const val VS_ENABLE = "vs_enable"
    const val VS_DISABLE = "vs_disable"

    /** `guided/`: `setVirtualStickAdvancedModeEnabled` — void, synchronous. */
    const val VS_ADVANCED_MODE = "vs_advanced_mode"
}

/** The `phase` of a [LogEntry.DjiCall]. */
object DjiPhase {
    /** We made the call. Written before the SDK is touched. */
    const val ASK = "ask"

    /** DJI's success callback. */
    const val OK = "ok"

    /** DJI's failure callback, with its error name. */
    const val ERR = "err"

    /** A synchronous call returned — cleanly, or with the throwable in `err`. */
    const val SYNC = "sync"

    /**
     * **No callback ever arrived.** The measured DJI behaviour, made into positive evidence
     * instead of an absence a reader has to notice.
     */
    const val NONE = "none"
}
