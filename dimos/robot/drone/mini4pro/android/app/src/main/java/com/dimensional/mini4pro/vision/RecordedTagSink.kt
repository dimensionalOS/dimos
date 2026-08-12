package com.dimensional.mini4pro.vision

import com.dimensional.mini4pro.record.Tap

/**
 * **Everything the detector sees goes on the record, on the way past.**
 *
 * The exact shape of `gimbal/RecordedGimbalPort` and `light/RecordedLightPort`, for the reason the
 * first of those argues at length: recording is a property of *reaching a consumer*, so it lives in
 * the object standing between the producer and the consumer, not beside either. [TagRecogniser] is
 * constructed with one of these, so a recogniser whose sightings reach nothing cannot be built.
 *
 * ## The standing proof that this is not paranoia
 *
 * The gimbal aimed a real camera for weeks and **not one gimbal reading existed in any flight
 * record** (`replay/ReplayCoverage.GIMBAL`). Nobody decided that; it is what happens when recording
 * is something you remember to call. The tag detector was at higher risk of the same thing than the
 * gimbal ever was, because when it merged its natural consumer — the `vision_msgs.Detection3D`
 * channel on the Zenoh bus — did not exist. A design that recorded through that channel would have
 * recorded nothing at all that day and nobody would have noticed until the first landing.
 *
 * **The channel exists now (2026-07-28) and the argument is unchanged**, in fact sharpened: it is
 * off by default and it is a network away. A record that depended on it would be empty on every
 * flight an operator did not think to enable it for, which is most of them.
 *
 * ## Order, and why the record goes first
 *
 * The tap is called **before** [downstream]. If a consumer throws, the record still has the
 * sighting; if the process dies between the two, the record still has the sighting. That is the
 * same ordering `RecordedActionPort` establishes for an ask and for the same reason — the evidence
 * must not be contingent on what was done with it.
 *
 * ## Faithfulness
 *
 * Every argument is forwarded untouched, and [downstream]'s throw is *not* swallowed here: the
 * recogniser's own worker contains it and logs it, and swallowing it twice would mean a broken
 * consumer produced no message anywhere. What this class does swallow is the **tap's** throw, per
 * `Tap`'s contract that an evidence problem must never become a flight problem.
 */
class RecordedTagSink(
    private val tap: Tap,
    /**
     * What the sighting is for — `zenoh/ZenohBus.publishDetection` in the app, and the default
     * no-op wherever a recogniser is built without a bus, which is every unit test and every
     * session with the channel switched off.
     */
    private val downstream: (TagSighting.Sighting, TagFix?, Boolean) -> Unit = { _, _, _ -> },
) : (TagSighting.Sighting, TagFix?, Boolean) -> Unit {

    override fun invoke(sighting: TagSighting.Sighting, fix: TagFix?, latched: Boolean) {
        // Contained, and first. `Recorder.tagSeen` contains its own throws too; this is the belt to
        // that brace, and it exists because `Tap` is an interface and the next implementation of it
        // will not necessarily have read the contract.
        try {
            tap.tagSeen(sighting, fix, latched)
        } catch (e: Throwable) {
            // Deliberately not rethrown and deliberately not logged from here: the recogniser owns
            // the log and would otherwise report the same failure twice a frame at 10 Hz.
        }
        downstream(sighting, fix, latched)
    }
}
