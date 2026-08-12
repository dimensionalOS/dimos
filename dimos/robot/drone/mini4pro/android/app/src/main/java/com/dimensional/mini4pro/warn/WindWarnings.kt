package com.dimensional.mini4pro.warn

/**
 * **DJI's own wind warning, mapped into the shared vocabulary** — the first source added after
 * device health, and the one landing17 asked for.
 *
 * ## The flight
 *
 * `datasets/landing17/20260730-172355.001.jsonl`. Ivan, watching QGroundControl in a wind that DJI
 * itself measured at up to **14.2 m/s** (`windSpeedDmS` 142 at t=140.075, against this airframe's
 * rated ~10.7 m/s wind resistance): *"is DJI giving us wind warnings or no? We should be logging
 * this in ground control, of course."*
 *
 * It was. The recorder had `KeyWindWarning` on a listener and wrote every transition down —
 * measured, from that record:
 *
 * | t (s) | `windWarning` |
 * |---|---|
 * | 3.100 | `LEVEL_0` (the first reading) |
 * | 111.078 | `LEVEL_2` |
 * | 151.074 | `LEVEL_0` |
 * | 159.083 | `LEVEL_2` |
 * | 183.076 | `LEVEL_0` |
 * | 202.074 | `LEVEL_2` |
 * | 216.079 | `LEVEL_0` |
 * | 225.084 | `LEVEL_2` |
 * | 227.085 | `LEVEL_0` |
 *
 * Eight transitions in 227 seconds, four of them into `LEVEL_2`, and not one of them reached the
 * pilot on any surface. The aircraft was explaining itself perfectly well through a channel we had
 * subscribed to and then said nothing about — the device-health failure with the subscription
 * already in place.
 *
 * Two things in that table are worth carrying into the design: **DJI never used `LEVEL_1`** on this
 * flight (it went 0 → 2 and back), and **it flaps** — 225.084 to 227.085 is two seconds. The first
 * is why `LEVEL_1` is mapped from the enum rather than from evidence; the second is why
 * [WarningMonitor]'s rate bound matters here more than it does for a standing overheat.
 *
 * ## The enum, as the jar defines it
 *
 * `javap -p -c` on `dji.sdk.keyvalue.value.flightcontroller.WindWarning` in
 * `dji-sdk-v5-aircraft-provided-5.18.0.jar` (2026-07-30, the method `docs/msdk/actions.md` §2a
 * uses — read the `static {}` block rather than trusting the docs):
 *
 * | DJI constant | wire `value` | ordinal |
 * |---|---|---|
 * | `LEVEL_0` | 0 | 0 |
 * | `LEVEL_1` | 1 | 1 |
 * | `LEVEL_2` | 2 | 2 |
 * | `UNKNOWN` | **65535** | 3 |
 *
 * Four constants: **there is no LEVEL_3**, and `UNKNOWN` is DJI's usual 65535 sentinel rather than
 * 3 — the same shape as `WarningLevel.UNKNOWN` in the health manager. Nothing here reads
 * `value()`: the jar's own `find(int)` *mutates* the returned `UNKNOWN` singleton's value field
 * when nothing matches, so an integer read is not even stable. The name is what crosses the seam,
 * as [WarnLevel.ofName] requires of every source.
 *
 * ## The mapping, and why each row is what it is
 *
 * | DJI | ours | what the operator gets |
 * |---|---|---|
 * | `LEVEL_0` | [WarnLevel.NORMAL] | recorded and published; **no `STATUSTEXT`** |
 * | `LEVEL_1` | [WarnLevel.CAUTION] | yellow in QGC, spoken once |
 * | `LEVEL_2` | [WarnLevel.WARNING] | red modal, spoken |
 * | anything else | [WarnLevel.UNKNOWN] | yellow, spoken; never silence |
 *
 * **`LEVEL_0` is reported, not withheld.** It is how this source says "nothing wrong", and mapping
 * it to NORMAL — rather than delivering an empty picture — is what turns the wind dropping back
 * into a `CHANGED`-to-NORMAL that [WarningMonitor] announces as a clear. Ivan asked for the clear
 * explicitly, and it is the half of the message an operator acts on: the warning going away is when
 * they stop fighting the wind.
 *
 * **`LEVEL_2` earns the modal.** DJI raises it where its own app tells the pilot to land, and on
 * landing17 it was raised in a wind above the airframe's rated resistance. The one thing worse than
 * a red modal on a windy day is a yellow line nobody reads — which is, precisely, what this project
 * measured on 2026-07-26 and again on 2026-07-30.
 *
 * **`LEVEL_1` is a caution rather than a warning**, one rung below, because DJI's own scale puts a
 * rung between "fine" and "land now" and flattening it would throw away the only early notice the
 * aircraft gives. Unmeasured on our flights — landing17 never showed it — and stated as such.
 *
 * ## The measurement
 *
 * `KeyWindSpeed`, DJI's own estimate in **decimetres per second**, is carried in the same sentence
 * as `"14.2 m/s"`. It is not a second opinion about the wind — it is the same aircraft's other
 * reading, already on the record and already on the `wind` Zenoh channel — and it is here because
 * landing14 proved a level alone is not enough: `windWarning` stayed `LEVEL_0` through the 9.1 m/s
 * incident that overwhelmed lateral control. Null when DJI has not delivered a speed yet, and null
 * means unknown: the sentence simply goes without it rather than saying `0 m/s`.
 */
object WindWarnings {

    /**
     * The code this source's single warning always carries: the DJI key it comes from, verbatim, so
     * a record reader greps the same string in the `dji_warn` line and in the `dji_field` line the
     * recorder writes for the raw value.
     */
    const val CODE = "windWarning"

    /** DJI's own name for "no wind warning". Public because the tests pin the mapping by name. */
    const val LEVEL_NONE = "LEVEL_0"

    /**
     * One `KeyWindWarning` delivery as a picture: always exactly one [Warning], never an empty
     * list.
     *
     * @param stateName DJI's `WindWarning` enum name, verbatim (`toString()` on the delivered
     *   value). Null when DJI delivered a null — treated as [WarnLevel.UNKNOWN] rather than as
     *   "fine", because a key that reports nothing is not a key that reports good news.
     * @param speedDmS the newest `KeyWindSpeed` reading in dm/s, or null when none has arrived.
     */
    fun picture(stateName: String?, speedDmS: Int?): List<Warning> = listOf(
        Warning(
            source = WarnSource.WIND,
            code = CODE,
            state = stateName ?: "UNKNOWN",
            level = levelOf(stateName),
            // Our words, and the one place in this package where they are ours rather than DJI's:
            // the SDK gives this key no title and no description at all, only a level. "Strong
            // wind" is what the level means, said in the fewest bytes an operator can act on.
            title = titleOf(stateName),
            measurement = speedOrNull(speedDmS),
        )
    )

    /**
     * DJI's enum name → the shared ladder. See the class doc's table for the argument behind each
     * row; anything unrecognised — a future `LEVEL_3`, a renamed constant, DJI's own `UNKNOWN` —
     * lands on [WarnLevel.UNKNOWN], which is forwarded rather than swallowed.
     */
    fun levelOf(stateName: String?): WarnLevel = when (stateName) {
        LEVEL_NONE -> WarnLevel.NORMAL
        "LEVEL_1" -> WarnLevel.CAUTION
        "LEVEL_2" -> WarnLevel.WARNING
        else -> WarnLevel.UNKNOWN
    }

    /**
     * The words. Three of them, because the sentence has to hold DJI's framing, the level word on a
     * change, and the measured speed, all inside fifty bytes.
     *
     * `LEVEL_0`'s words are "Wind" rather than "No wind warning": the sentence it appears in is
     * `"DJI cleared: Wind"`, and a clear that reads "DJI cleared: No wind warning" is a double
     * negative on a screen someone is reading while flying.
     */
    fun titleOf(stateName: String?): String = when (stateName) {
        LEVEL_NONE -> "Wind"
        "LEVEL_1" -> "Wind rising"
        "LEVEL_2" -> "Strong wind"
        else -> "Wind warning $stateName"
    }

    /**
     * dm/s → `"14.2 m/s"`, or null when there is no reading.
     *
     * One decimal because that is exactly the resolution DJI quantises to at source: 142 dm/s is
     * 14.2 m/s and there is no second digit to print. The conversion is the same arithmetic
     * `ZenohTelemetryEncoder.windOrNull` does for the `wind` channel — deliberately restated as
     * one line rather than imported, because this package imports nothing, and pinned against that
     * channel's own number in the tests so the two can never drift.
     */
    fun speedOrNull(speedDmS: Int?): String? {
        val dms = speedDmS ?: return null
        val whole = dms / 10
        val tenth = (dms % 10).let { if (it < 0) -it else it }
        return "$whole.$tenth m/s"
    }

}
