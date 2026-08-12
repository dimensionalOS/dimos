package com.dimensional.mini4pro.replay

import com.dimensional.mini4pro.telemetry.StateSource

/**
 * **What a published replay says about itself**, in the three places it has to say it.
 *
 * A replay that is only drawn is contained by the screen: the watermark, the strip and the
 * banner are all in front of the person who opened it. A replay that is *published* is not — it
 * reaches a ground station in another field and a bus in another building, and the people at
 * those ends did not open anything. So every sentence this object produces exists to answer one
 * question for somebody who is not holding the phone: **is this an aircraft?**
 *
 * Pure, and separate from the Android that shows it, for the reason every other decision in this
 * project is: a sentence is a contract, and a contract asserted by a test is a different thing
 * from one asserted by whoever last edited a layout.
 *
 * ## The three places, and why the wording differs
 *
 * - [banner] is for the operator, who already knows and needs reminding. It names the sinks,
 *   because *"a replay is on screen"* and *"a replay is being broadcast"* are different states
 *   and the strip that says the first one has been on screen for a minute already.
 * - [watermark] is across the whole picture, and is the one an operator sees without looking.
 * - [announcement] is for the bus, and it is the only one a **subscriber** will ever see. It
 *   goes on `status`, which is the channel DiMOS already reads for operator sentences, so a
 *   consumer that is doing nothing more than logging status text still records that the flight
 *   it just ingested was somebody's afternoon from a file.
 *
 * ## The recorder, and the hall of mirrors
 *
 * **A publishing replay never reaches the flight recorder, and the recorder is not stopped
 * either.** Both halves are deliberate and they are answers to different questions.
 *
 * *Not fed*: `Recorder.sample` reads the live aircraft and does not consult
 * [com.dimensional.mini4pro.telemetry.StateSource] at all — structurally, not by a flag. A record
 * written from a replayed state would be a file that is byte-for-byte the shape of a flight and
 * is not one, with nothing in its schema to say so. Next week it is in the library beside the
 * real ones, and the week after somebody replays *it*. The flight record is this project's
 * primary evidence, and evidence that can be manufactured by replaying evidence has stopped
 * being evidence. That is the whole argument and it is not a close call.
 *
 * *Not stopped*: because a recorder that stops is the failure this project has already been bitten
 * by — the gimbal aimed a real camera for weeks with nothing in any record — and because the
 * live session around the replay is still a session worth having. What the record gets instead is
 * [START_EVENT] and [STOP_EVENT]: the window is **marked**, so a post-mortem reading a quiet
 * stretch of `dji_state` lines knows the phone was busy describing a different flight to two
 * networks, rather than inferring that nothing happened.
 *
 * The result is the honest pair. The record contains no replayed state and a plain statement that
 * a replay was published, which is exactly the provenance a hall of mirrors lacks.
 */
object ReplayPublication {

    /** `record/EventCode` value written when the first sink is switched on. */
    const val START_EVENT = "replay_publish_start"

    /** …and when the last one is switched off, or the recording is closed. */
    const val STOP_EVENT = "replay_publish_stop"

    /** The watermark when a recording is drawn and nothing is published. */
    const val WATERMARK_QUIET = "REPLAY · NOT THE AIRCRAFT"

    /** The watermark when at least one sink is being fed. */
    const val WATERMARK_PUBLISHING = "REPLAY · PUBLISHING · NOT THE AIRCRAFT"

    /**
     * The sinks being fed, in catalogue order, or an empty list.
     *
     * Order is fixed rather than insertion-ordered so that two phones publishing the same pair
     * produce the same sentence, and so a test can assert a string rather than a set.
     */
    fun sinks(toMavlink: Boolean, toZenoh: Boolean): List<StateSource.Sink> = buildList {
        if (toMavlink) add(StateSource.Sink.MAVLINK)
        if (toZenoh) add(StateSource.Sink.ZENOH)
    }

    /** `MAVLink`, `Zenoh`, or `MAVLink + Zenoh`. Empty when nothing is published. */
    fun sinkNames(toMavlink: Boolean, toZenoh: Boolean): String =
        sinks(toMavlink, toZenoh).joinToString(" + ") {
            when (it) {
                StateSource.Sink.MAVLINK -> "MAVLink"
                StateSource.Sink.ZENOH -> "Zenoh"
            }
        }

    /** The whole-picture watermark. */
    fun watermark(publishing: Boolean): String =
        if (publishing) WATERMARK_PUBLISHING else WATERMARK_QUIET

    /**
     * The banner across the top of the screen.
     *
     * Publishing gets its own sentence rather than a suffix on the quiet one, because the two
     * states differ in what an operator should do about them and a reader scanning a strip reads
     * the first three words.
     */
    fun banner(name: String, toMavlink: Boolean, toZenoh: Boolean): String =
        if (toMavlink || toZenoh) {
            "REPLAY PUBLISHING → ${sinkNames(toMavlink, toZenoh)} — NOT THE AIRCRAFT · $name"
        } else {
            "REPLAY — NOT THE AIRCRAFT · $name"
        }

    /**
     * The sentence put on the Zenoh `status` channel when Zenoh-out is switched on, and the one
     * thing on the bus that tells a subscriber what it is looking at.
     *
     * **Said only for Zenoh**, and that asymmetry is not an oversight: `status` is a Zenoh
     * channel, so announcing a MAVLink-only replay there would put a sentence about a stream a
     * subscriber cannot see onto the one it can, beside telemetry that is still the live
     * aircraft's. The MAVLink side has its own channel for this — `STATUSTEXT`, through the same
     * `Announcer` — and gets the same sentence by the same fan-out.
     *
     * Deliberately **not** a new channel and not a field on every message. A `Detection3D` and an
     * `Odometry` have nowhere honest to carry a provenance flag, and inventing one would be a
     * schema change that every consumer would have to be taught to read. A sentence on the
     * channel built for sentences is discoverable by anything already subscribed.
     */
    fun announcement(name: String, toMavlink: Boolean, toZenoh: Boolean): String =
        "REPLAY: this stream is a recording — $name — published to " +
            "${sinkNames(toMavlink, toZenoh)}. It is not an aircraft."

    /** The sentence when publishing ends, so a subscriber is not left assuming it still is. */
    fun endedAnnouncement(name: String): String =
        "REPLAY ended: $name is no longer being published."
}
