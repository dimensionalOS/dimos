package com.dimensional.mini4pro.telemetry

import kotlin.math.abs

/**
 * Decides when `HOME_POSITION` and `GPS_GLOBAL_ORIGIN` are due.
 *
 * They are **event** messages: QGC needs them once, and again if home actually
 * moves. Sending them at telemetry rate is noise on a link that also carries the
 * things that do change.
 *
 * Plain Kotlin and stateful, living here rather than in `Bridge`, because the
 * "has it changed?" rule is the part worth testing and `Bridge` is Android-bound.
 *
 * ## Why the naive change key failed — measured 2026-07-26 11:17
 *
 * The previous rule keyed on `"$homeLatitude,$homeLongitude,$takeoffAltitudeAmsl"`
 * and compared strings. `takeoffAltitudeAmsl` is barometric and never repeats:
 * 62.104833984375006 → 62.115… → 62.123… every single tick. So the key changed on
 * every tick, the on-change guard never held, and the pair went out at 1 Hz for
 * the whole session — 220 of each in 220 seconds.
 *
 * ## The rule now
 *
 * - **latitude/longitude: exact comparison.** They come from an event-driven key,
 *   they do not jitter, and any change in them is a genuinely different home.
 * - **altitude: hysteresis against the last *published* value**, threshold
 *   [ALT_CHANGE_M]. Hysteresis rather than quantising into fixed buckets: a value
 *   drifting across a bucket boundary flips back and forth and re-publishes
 *   exactly as often as the raw value does, which is the bug being fixed. Rebasing
 *   on each publish means a slow drift costs one message per [ALT_CHANGE_M] of
 *   drift rather than one per tick.
 *
 * A home that becomes *unknown* does not clear the memory. There is no way to
 * retract a `HOME_POSITION` — the GCS keeps the last one it was told either way —
 * so forgetting would only allow a home that flapped null/known/null to re-publish
 * a home the GCS already has, which is the 1 Hz failure in a slower costume.
 * [reset] exists for a new link, whose GCS genuinely knows nothing.
 */
class HomeEventGate {

    private data class Published(val lat: Double, val lon: Double, val altAmslM: Double)

    private var published: Published? = null

    /**
     * The event messages to send now, or empty when there is nothing new to say —
     * either because home is still unknown (stay silent, do not fake it) or
     * because the GCS already has this home.
     *
     * Records what it hands back, so a caller that sends the result keeps the gate
     * honest; a caller that drops it would re-announce, which is the safe direction.
     */
    fun messagesIfChanged(s: AircraftState): List<Any> {
        val messages = TelemetryEncoder.eventMessages(s)
        // Empty means the encoder found home unknown. Nothing to send, and nothing
        // to forget — see the class doc.
        if (messages.isEmpty()) return emptyList()
        // The same resolution the messages were built from, so the memory can never
        // describe a home other than the one that went out. Non-empty guarantees
        // both reads succeed; they are written to fail closed all the same.
        val home = TelemetryEncoder.homeCoordinate(s) ?: return emptyList()
        val alt = s.takeoffAltitudeAmsl?.takeIf { it.isFinite() } ?: return emptyList()
        val last = published
        val changed = last == null ||
            last.lat != home.first ||
            last.lon != home.second ||
            abs(last.altAmslM - alt) >= ALT_CHANGE_M
        if (!changed) return emptyList()
        published = Published(home.first, home.second, alt)
        return messages
    }

    /** What was last published, for the Bridge's log line. Null before the first. */
    fun lastPublished(): String? = published?.let { "${it.lat},${it.lon},${it.altAmslM}" }

    /** Forget what was published. For a new link, whose GCS has been told nothing. */
    fun reset() {
        published = null
    }

    companion object {
        /**
         * How far the home AMSL datum must move before it is worth re-announcing,
         * in metres.
         *
         * `KeyTakeoffLocationAltitude` is a barometric number on a parked aircraft
         * and it wanders: 62.104 → 62.123 tick to tick, 62.712 → 62.976 over 80 s
         * (`tmp/session-logs/20260726-102245.001.jsonl`), 59.187 → 59.461 → 59.181
         * over 40 s (`…-094037`), and the ground probe measured ~0.7 m of drift
         * while completely stationary. One metre sits above that noise floor with
         * margin, and below anything that matters: it is inside the barometer's own
         * accuracy, so a home datum up to a metre out is no less true than a fresh
         * one.
         *
         * It is also not the mechanism that catches a *relocated* home — moving
         * home moves its latitude and longitude, which are compared exactly. This
         * threshold only decides how finely we track the drift of a home that has
         * not moved.
         */
        const val ALT_CHANGE_M = 1.0
    }
}
