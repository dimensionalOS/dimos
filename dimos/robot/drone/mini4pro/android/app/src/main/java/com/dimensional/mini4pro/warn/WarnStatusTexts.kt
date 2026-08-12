package com.dimensional.mini4pro.warn

import com.dimensional.mini4pro.command.StatusTexts

/**
 * Every operator-facing sentence a DJI warning can produce, counted against `STATUSTEXT`'s hard
 * 50-byte field and clamped by the same [StatusTexts.clamp] the rest of the project uses.
 *
 * ## The shortening rule, applied here
 *
 * `command/StatusTexts` states it: **DJI's word survives; ours does not.** These sentences carry
 * DJI's own title for a warning (or DJI's code when the title is unmapped), so when the framing plus
 * the title exceeds 50 bytes, the framing is dropped and DJI's words go out alone. That is the
 * opposite of what feels natural — our prefix is the part that explains where the sentence came from
 * — but the operator can only act on DJI's half, and a truncated title sends them searching for a
 * string that does not exist.
 *
 * ## Why the level word is not in the sentence
 *
 * QGroundControl prints its own severity label in front of every `STATUSTEXT`
 * (`StatusTextHandler.cc:184-217`: `Critical`, `Error`, `Warning`, `Notice`, `Info`), so a level
 * word here would be printed twice — and would cost 6-15 bytes of DJI's title to do it. The
 * exception is [changed], where the level word *is* the news: without it an escalation from CAUTION
 * to SERIOUS_WARNING reads as a duplicate of the appearance, and the only difference an operator
 * could notice is a colour change in a scrolling list.
 *
 * ## Why the measurement is
 *
 * **landing14 is the argument, and it is measured.** DJI's own wind warning stayed `LEVEL_0` through
 * a 9.1 m/s incident in which lateral stick was ignored and the aircraft drifted 1 m/s against full
 * deflection; a day of compass theorising followed, and the finding written onto the record is *"the
 * warning is not a substitute for the number"*. So where a source has a measurement —
 * [Warning.measurement], DJI's own reading, formatted with its unit — it rides **in the same
 * sentence**, because the level and the number answer different halves of the pilot's question and a
 * pilot reading a scrolling panel gets one line.
 *
 * It is the **first** thing dropped when the bytes do not fit, and that ordering is deliberate: the
 * number is on the `wind` Zenoh channel and in the flight record either way, whereas the words are
 * the only thing this channel exists to carry. Three tiers, in order of preference:
 *
 *  1. `"DJI: Strong wind 14.2 m/s"` — framing, words, number;
 *  2. `"DJI: Strong wind"` — framing and words;
 *  3. `"Strong wind"` clamped — DJI's words alone.
 */
object WarnStatusTexts {

    /**
     * A warning DJI was not previously reporting. 5 bytes of framing.
     *
     * No verb, and no claim about what the aircraft will do next. `"DJI: Aircraft overheating"` says
     * exactly what is true — DJI is reporting this — and leaves the consequence to the operator, who
     * can see the aircraft. This is [com.dimensional.mini4pro.command.StatusTexts]' discipline:
     * never name an outcome we cannot observe.
     */
    fun appeared(warning: Warning): String =
        preferring("DJI: ${warning.name}", warning.measurement, warning.name)

    /**
     * The same warning at a new level — an escalation or a de-escalation.
     *
     * The level word is DJI's own enum name, verbatim, for the reason the whole project quotes DJI
     * verbatim: `SERIOUS_WARNING` is searchable and "very bad" is not. `"DJI SERIOUS_WARNING: "` is
     * 21 bytes, which leaves 29 for the title; past that the framing goes and the title survives
     * alone, so an escalation never turns into a truncated word salad.
     */
    fun changed(warning: Warning): String =
        preferring(
            "DJI ${warning.level.name}: ${warning.name}",
            warning.measurement,
            warning.name,
        )

    /**
     * DJI has stopped reporting a warning.
     *
     * Announced rather than left as silence, and that is a decision worth stating: a warning that
     * disappears without a word leaves the operator's most recent information saying the aircraft is
     * overheating long after it has cooled. Silence is not the absence of a claim here — the
     * previous `STATUSTEXT` is still on their screen, still making one. 13 bytes of framing.
     *
     * **The measurement rides a clear too**, where there is one: "the wind warning has gone and the
     * wind is 6.1 m/s" is a materially different message from "the wind warning has gone", and the
     * second is the one that gets an operator to relax on a day they should not.
     */
    fun cleared(warning: Warning): String =
        preferring("DJI cleared: ${warning.name}", warning.measurement, warning.name)

    /**
     * The three tiers, in order: the long form with the measurement, the long form without it, then
     * DJI's own words alone — `command/StatusTexts`' rule, with the number as the first casualty.
     */
    private fun preferring(full: String, measurement: String?, essential: String): String {
        if (measurement != null) {
            val withNumber = "$full $measurement"
            if (withNumber.utf8Size() <= StatusTexts.MAX_BYTES) return withNumber
        }
        return if (full.utf8Size() <= StatusTexts.MAX_BYTES) full else StatusTexts.clamp(essential)
    }

    private fun String.utf8Size(): Int = toByteArray(Charsets.UTF_8).size
}
