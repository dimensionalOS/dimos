package com.dimensional.mini4pro.warn

/**
 * **The decision core for every DJI warning**: turns a *picture* into *changes*, decides how loudly
 * each one is said on each surface, and bounds how fast any of it may reach a ground station.
 *
 * Pure Kotlin. No DJI types, no MAVLink objects, no clock of its own, no I/O — the same shape as
 * `guided/GuidedStickEngine`'s law and `telemetry/HomeEventGate`, and for the same reason: this is
 * the part that has to be right, so it has to be testable without an aircraft.
 *
 * ## A delivery is a picture, not a stream — and it belongs to one source
 *
 * `DJIDeviceHealthInfoChangeListener.onDeviceHealthInfoUpdate(List<DJIDeviceHealthInfo>)` hands over
 * **the complete current set of warnings** every time it fires, not a delta. DJI re-delivers that
 * list whenever anything in it moves, and — on the evidence of every other MSDK callback this
 * project has measured — is entirely free to re-deliver an identical one as often as it likes.
 *
 * So the naive implementation announces the overheat warning again on every delivery. This project
 * has already paid for that mistake once: a refusal path that logged at the loop rate put 25 lines a
 * second into a monitor and buried everything else in the file. **Emitting per delivery is forbidden
 * here.** [onDelivery] diffs against the previous picture *of that source* and returns only genuine
 * transitions — a key appearing, a key changing level, a key disappearing — and returns an empty
 * list for a repeat, which is the overwhelmingly common case.
 *
 * **Per source, and that is what makes one monitor able to hold them all.** Each source delivers its
 * own complete picture on its own schedule, and a wind delivery must not read as "every health
 * warning disappeared". The diff is therefore scoped to the keys belonging to the delivering source
 * ([Warning.key] carries it), and every other source's picture is left exactly as it was.
 *
 * A source whose warning is a single scalar — wind is one enum, not a list — delivers a
 * one-element picture, and the levelling that makes that work is [WarnLevel.NORMAL]: DJI's
 * `LEVEL_0` is *reported*, at NORMAL, rather than delivered as an empty picture. That way the
 * transition out of a warning is a `CHANGED`-to-NORMAL, which the retraction rule below announces
 * as a clear, and the aircraft saying "the wind is fine now" reaches the operator exactly as the
 * cooled overheat does.
 *
 * ## Two independent brakes
 *
 * The diff is the first brake and it is the one that matters: an aircraft that is simply hot
 * produces exactly **one** event, no matter how many times DJI says so. The [RateBound] is the
 * second, for the case the diff cannot help with — a level that genuinely flaps, appearing and
 * clearing over and over. landing17 is the measured example: `windWarning` crossed between `LEVEL_0`
 * and `LEVEL_2` **eight times in 227 s**, twice within 2.0 s of each other (t=225.084 and t=227.085).
 * The bound bounds the *ground station* only: a suppressed event still reaches the flight record,
 * the Zenoh bus and logcat, marked [WarnEvent.rateLimited], so a post-mortem never loses a warning
 * that an operator's screen missed.
 *
 * ## Ordering
 *
 * Within one delivery, events come back **most serious first**, then appearances/changes before
 * clears. That ordering is not cosmetic: it is what decides which events win the scarce tokens when
 * the rate bound is biting, so the sentence that reaches QGC under flapping is the worst thing DJI
 * is currently saying rather than whichever entry DJI happened to list first.
 *
 * Not thread-safe by itself; [WarningBus] serialises every call, because DJI delivers on the
 * Android main thread.
 */
class WarningMonitor(
    private val rateBound: RateBound = RateBound(),
) {

    /**
     * The last picture each source delivered, keyed by [Warning.key].
     *
     * One map rather than one per source, because [Warning.key] already carries the source and a
     * single map is what lets [snapshot] answer "everything the aircraft is currently saying" in
     * one read — which is what the phone screen and the Zenoh publisher both want.
     *
     * A `LinkedHashMap` so that "delivery order" is a real, reproducible thing for the tie-break in
     * [onDelivery]'s sort, rather than whatever a hash bucket happens to yield.
     */
    private val current = LinkedHashMap<String, Warning>()

    /** Everything every source is currently reporting. Ordered as DJI delivered it. */
    fun snapshot(): List<Warning> = current.values.toList()

    /**
     * Diff [warnings] against [source]'s previous picture and return the changes, dressed and
     * rate-decided.
     *
     * @param source whose picture this is. **Only this source's keys are diffed**; anything another
     *   source is reporting survives untouched, because this delivery says nothing about it.
     * @param warnings the complete current set for that source. Warnings whose [Warning.source]
     *   disagrees with [source] are dropped rather than trusted — a source may only speak for
     *   itself, and a mislabelled delivery would otherwise clear another source's picture.
     * @param nowMs a monotonic clock (`SystemClock.elapsedRealtime`). Only the [RateBound] reads it;
     *   the diff itself is timeless, which is what lets the change tests hand-crank a clock that
     *   never moves and still be honest.
     * @return an empty list when nothing changed — the common case, and the whole point.
     */
    fun onDelivery(source: WarnSource, warnings: List<Warning>, nowMs: Long): List<WarnEvent> {
        // DJI's own list may repeat a key (it is a list, not a map). Last wins, and the fold is
        // what stops a duplicated entry from producing an appear+clear pair on the next delivery.
        val next = LinkedHashMap<String, Warning>()
        for (warning in warnings) {
            if (warning.source != source) continue
            next[warning.key] = warning
        }

        val changes = ArrayList<Triple<WarnChange, Warning, WarnLevel?>>()

        for ((key, warning) in next) {
            val previous = current[key]
            when {
                previous == null -> changes += Triple(WarnChange.APPEARED, warning, null)
                previous.level != warning.level ->
                    changes += Triple(WarnChange.CHANGED, warning, previous.level)
                // Same key, same level: DJI is repeating itself. Nothing to say. Title, description
                // and measurement are deliberately NOT compared — DJI rewrites the words as its
                // localisation tables load, and the wind speed moves continuously. A re-worded
                // identical warning is not news, and neither is 13.9 m/s becoming 14.0 m/s at the
                // same warning level. (The number itself is published every delivery on the `wind`
                // channel and written to the record; this channel is for the *level*.)
                else -> Unit
            }
        }
        for ((key, warning) in current) {
            if (warning.source != source) continue
            if (!next.containsKey(key)) changes += Triple(WarnChange.CLEARED, warning, null)
        }

        for (key in current.keys.filter { current[it]?.source == source }) current.remove(key)
        current.putAll(next)

        if (changes.isEmpty()) return emptyList()

        // Most serious first, clears last, delivery order as the tie-break. See the class doc: this
        // is what the rate bound spends its tokens on.
        val ordered = changes.sortedWith(
            compareBy(
                { if (it.first == WarnChange.CLEARED) 1 else 0 },
                { -severityRank(it.second.level) },
            )
        )

        return ordered.map { (change, warning, previousLevel) ->
            event(change, warning, previousLevel, nowMs)
        }
    }

    /**
     * Forget everything. The next delivery re-announces the whole picture as appearances.
     *
     * Called when a subscription is dropped: what we remember is a claim about *now*, and after a
     * teardown we no longer have grounds for it. Deliberately re-announcing rather than staying
     * quiet — the operator of a freshly reconnected session is owed the current warnings, and a
     * monitor that remembered across a gap would silently withhold exactly the overheat message
     * this package exists to surface.
     */
    fun reset() {
        current.clear()
        rateBound.reset()
    }

    private fun event(
        change: WarnChange,
        warning: Warning,
        previousLevel: WarnLevel?,
        nowMs: Long,
    ): WarnEvent {
        val level = warning.level
        val severity = when (change) {
            // A warning going away is good news, always, whatever level it was at. It is reported
            // at INFO so it neither speaks nor raises the red modal QGC gives ERROR and above —
            // "the overheat cleared" must never look like a new emergency.
            WarnChange.CLEARED ->
                if (level.forwardable()) WarnLevel.MAV_SEVERITY_INFO else null
            WarnChange.APPEARED -> level.mavSeverity()
            // A de-escalation to a level we do not forward is a retraction, and it is announced for
            // the same reason a clear is: the operator's last message still says the aircraft is
            // overheating, and silence does not withdraw it. DJI keeps a cooled code in the list at
            // NORMAL rather than dropping it — and the wind source deliberately does the same with
            // LEVEL_0 — so without this the only difference between "recovered" and "still on fire"
            // is which of the two DJI happened to choose.
            WarnChange.CHANGED ->
                level.mavSeverity()
                    ?: if (previousLevel != null && previousLevel.forwardable()) {
                        WarnLevel.MAV_SEVERITY_INFO
                    } else {
                        null
                    }
        }
        val text = when (change) {
            WarnChange.APPEARED -> WarnStatusTexts.appeared(warning)
            // The retraction reads as a clear, because to the operator it is one — the warning they
            // were told about is over. The level word would say `NORMAL`, which is a true but
            // useless thing to put in fifty bytes.
            WarnChange.CHANGED ->
                if (level.mavSeverity() == null) WarnStatusTexts.cleared(warning)
                else WarnStatusTexts.changed(warning)
            WarnChange.CLEARED -> WarnStatusTexts.cleared(warning)
        }
        val recordSeverity = when (change) {
            WarnChange.CLEARED -> "info"
            WarnChange.APPEARED, WarnChange.CHANGED -> level.recordSeverity()
        }
        // The bus's own ladder, from the same decision: a clear is OK on `diagnostic_msgs` for the
        // reason it is INFO on MAVLink — a consumer must not see the retraction of a fault as a
        // fault. Everything else is the level's own rung.
        val diagnosticLevel = when (change) {
            WarnChange.CLEARED -> WarnLevel.DIAGNOSTIC_OK
            WarnChange.APPEARED, WarnChange.CHANGED -> level.diagnosticLevel()
        }
        val allowed = severity != null && rateBound.take(nowMs)
        return WarnEvent(
            change = change,
            warning = warning,
            previousLevel = previousLevel,
            mavSeverity = severity,
            recordSeverity = recordSeverity,
            diagnosticLevel = diagnosticLevel,
            text = text,
            announce = allowed,
            rateLimited = severity != null && !allowed,
        )
    }

    /**
     * Ordering key for the "most serious first" sort.
     *
     * DJI's own ranks are scaled by ten purely to leave room for [WarnLevel.UNKNOWN], which is not
     * on DJI's scale at all and is slotted **between CAUTION and WARNING** — the same place
     * [WarnLevel.mavSeverity] puts it, so the order events are announced in and the loudness they
     * are announced at cannot disagree.
     */
    private fun severityRank(level: WarnLevel): Int =
        level.rank?.times(RANK_SCALE) ?: UNKNOWN_SORT_RANK

    private companion object {
        /** DJI's ranks are multiplied by this, to leave gaps for levels not on DJI's scale. */
        const val RANK_SCALE = 10

        /** [WarnLevel.UNKNOWN]'s slot: between CAUTION (20) and WARNING (30). */
        const val UNKNOWN_SORT_RANK = 25
    }
}
