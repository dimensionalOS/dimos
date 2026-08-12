package com.dimensional.mini4pro.warn

/**
 * **The one vocabulary every warning the aircraft gives us is expressed in** — DJI Fly's device
 * health messages ("Aircraft overheating", "Compass interference"), DJI's wind warning, and every
 * source added after them.
 *
 * ## Why this package exists
 *
 * On 2026-07-26 the aircraft repeatedly force-landed and returned home on the bench, and an hour
 * went into theorising about it before someone opened DJI Fly and found an **overheat warning that
 * had been showing the whole time**. Every layer of this bridge was working: telemetry was
 * accurate, commands were answered, the flight recorder was recording. The aircraft was telling its
 * operator exactly what was wrong, through a channel we had simply never subscribed to. That is the
 * worst shape a display failure can take — everything looks healthy and the explanation is one API
 * call away.
 *
 * That produced `health/`, which wired **one** source (`DeviceHealthManager`) to **two** surfaces
 * (QGC and the flight record). Four days later landing17 produced the same failure in miniature and
 * from the other end: DJI raised `KeyWindWarning` `LEVEL_2` four times in a 14.2 m/s wind — above
 * this airframe's rated ~10.7 m/s resistance — the recorder wrote every transition down, and
 * **nobody told the pilot**, who was watching QGC and asking out loud whether DJI was giving us
 * wind warnings at all. A warning that reaches a file is not a warning.
 *
 * Ivan's instruction closing that: *"make sure to pass all of these DJI warnings into both Zenoh
 * and QGroundControl, of course. Just so I don't need to ask you again for the next warning."*
 *
 * ## The shape that answers it
 *
 * ```
 *   DJI source                     the owner                        the surfaces
 *   ─────────────────────────────  ───────────────────────────────  ─────────────────────
 *   DeviceHealthManager  ┐         WarningMonitor (diff, order,     QGC     STATUSTEXT
 *   KeyWindWarning       ├──────►  rate) ──► WarningBus (fan-out)   phone   the screen
 *   …the next one        ┘                                          record  dji_warn
 *                                                                   Zenoh   warnings
 * ```
 *
 * A source's whole job is to say **what DJI is currently reporting**, as [Warning] objects. It does
 * not decide severity, does not phrase a sentence, does not decide whether anything is news, and
 * **does not talk to a surface**. Everything after that is [WarningMonitor]'s and [WarningBus]'s,
 * once, so that the four surfaces cannot disagree about what the operator was told — and so that
 * the next source is a mapping function rather than another wiring campaign.
 *
 * The rule is written down in `CLAUDE.md`, because wiring one ad hoc is precisely the
 * two-places-for-one-property failure this project keeps deleting.
 *
 * ## The seam
 *
 * `dji.v5.manager.diagnostic.DJIDeviceHealthInfo` is unwrapped exactly once, in
 * `health/MsdkDeviceHealthPort`, and `dji.sdk.keyvalue.value.flightcontroller.WindWarning` exactly
 * once, in `record/Recorder`'s tap (by name, never by ordinal). Nothing in this package imports a
 * DJI type, a MAVLink type or an Android type — the same DJI-free seam `telemetry/AircraftState`
 * draws for the key path and `gimbal/GimbalPort` for the gimbal. `docs/architecture.md` has the
 * general argument; `docs/device-health.md` has this package's.
 */

/**
 * **Where a warning came from.** DJI's own subsystem, not a category of ours.
 *
 * Part of [Warning.key], so two sources can never collide on a code, and carried verbatim onto
 * every surface — the record's `src`, the Zenoh status name, the logcat line. A reader asking "was
 * this the health manager or the flight controller?" must never have to infer it from the code's
 * shape.
 */
enum class WarnSource(
    /** The word that goes on the wire and into the record. Lowercase, stable, greppable. */
    val label: String,
) {
    /**
     * `dji.v5.manager.diagnostic.DeviceHealthManager` — the DJI Fly message list. Overheat,
     * compass interference, ESC faults, the simulator notice. `health/MsdkDeviceHealthPort`.
     */
    DEVICE_HEALTH("health"),

    /**
     * `FlightControllerKey.KeyWindWarning` — the flight controller's own wind assessment, three
     * levels and a sentinel. [WindWarnings] maps it, and pairs it with `KeyWindSpeed` so the
     * sentence carries the number as well as the word.
     */
    WIND("wind"),
    ;

    companion object {
        /** Every source, for the doc tables and the tests that assert this list is the whole list. */
        val ALL: List<WarnSource> = entries
    }
}

/**
 * **The one severity ladder**, and the single owner of what each rung costs on each surface.
 *
 * It is DJI's `WarningLevel` — the richest of the vocabularies the sources speak, and the one
 * already measured — and every other source is *mapped onto* it rather than carrying a ladder of
 * its own. That is the whole reason a wind `LEVEL_2` and an overheat `WARNING` can be announced by
 * the same code at the same loudness: there is one table, below, and no second opinion anywhere.
 *
 * Verified by `javap` against `dji-sdk-v5-aircraft-provided-5.18.0.jar` on 2026-07-26 —
 * `dji.v5.manager.diagnostic.WarningLevel` declares exactly these six constants, with the integer
 * `level` field shown in the table (recovered from the enum's `static {}` block). `UNKNOWN` is
 * `65535`, not `5`, which is DJI's usual "no mapping" sentinel.
 *
 * | DJI constant | `level` | meaning |
 * |---|---|---|
 * | `NORMAL` | 0 | nothing wrong; DJI Fly shows nothing |
 * | `NOTICE` | 1 | informational |
 * | `CAUTION` | 2 | worth knowing |
 * | `WARNING` | 3 | the class the overheat that started this belongs to |
 * | `SERIOUS_WARNING` | 4 | the aircraft is about to act on its own |
 * | `UNKNOWN` | 65535 | DJI could not map its own code — **not** the same as NORMAL |
 *
 * [UNKNOWN] is deliberately its own value rather than being folded into [NORMAL] or dropped. "DJI
 * said something we could not interpret" and "DJI said everything is fine" are opposite claims, and
 * this project's rule (PLAN.md, honesty boundaries) is that the absence of evidence never reads as
 * good news. See [mavSeverity] for what it costs on the wire.
 *
 * Declared in ascending order of seriousness **except** [UNKNOWN], which is last because it is not
 * on the scale at all. Nothing may use `ordinal` for severity comparison; use [atLeast].
 */
enum class WarnLevel {
    NORMAL,
    NOTICE,
    CAUTION,
    WARNING,
    SERIOUS_WARNING,
    UNKNOWN,
    ;

    /**
     * Rank on DJI's own scale, or `null` for [UNKNOWN], which has no rank.
     *
     * Kept as an explicit `when` rather than `ordinal` so that adding a constant is a compile error
     * here instead of silently re-ranking everything below it.
     */
    val rank: Int?
        get() = when (this) {
            NORMAL -> 0
            NOTICE -> 1
            CAUTION -> 2
            WARNING -> 3
            SERIOUS_WARNING -> 4
            UNKNOWN -> null
        }

    /** True when this level is at least as serious as [other]. [UNKNOWN] compares to nothing. */
    fun atLeast(other: WarnLevel): Boolean {
        val mine = rank ?: return false
        val theirs = other.rank ?: return false
        return mine >= theirs
    }

    /**
     * **The severity table. This mapping is the whole user-visible contract of the package**, and
     * it was chosen against what QGroundControl actually does with each value, read out of
     * `ref/qgroundcontrol` on 2026-07-26 rather than assumed:
     *
     *  - `severity <= MAV_SEVERITY_NOTICE (5)` is **spoken aloud** by QGC's TTS (`Vehicle.cc:3466`,
     *    `readAloud = true`).
     *  - `MAV_SEVERITY_ERROR (3)` and below is an **error**: `StatusText::severityIsError`
     *    (`StatusTextHandler.cc:18-26`) drives `showCriticalVehicleMessage`, QGC's red modal, and
     *    the toolbar's error count.
     *
     * | level | MAV_SEVERITY | what the operator gets in QGC |
     * |---|---|---|
     * | `NORMAL` (0) | *not forwarded* | flight record + Zenoh + logcat only |
     * | `NOTICE` (1) | `INFO` (6) | listed in the message panel; not spoken, not red |
     * | `CAUTION` (2) | `WARNING` (4) | yellow in the panel, **and spoken**; not red |
     * | `WARNING` (3) | `ERROR` (3) | **red modal + spoken** |
     * | `SERIOUS_WARNING` (4) | `CRITICAL` (2) | **red modal + spoken** |
     * | `UNKNOWN` (65535) | `WARNING` (4) | yellow in the panel, **and spoken** |
     *
     * **The speech threshold is `NOTICE` (5)**: `Vehicle.cc:3466` reads
     * `severity <= MAV_SEVERITY_NOTICE -> readAloud = true`, and `MAV_SEVERITY_WARNING` is **4**,
     * so a WARNING *is* read aloud. Only `INFO` (6) among the severities this package emits is
     * silent. This was documented backwards when the package was written and corrected on
     * 2026-07-27 against the vendored QGC source; the mapping itself was left unchanged, because
     * what `ERROR` buys over `WARNING` is the modal and the toolbar error count, not the speech.
     *
     * Three judgements are worth defending.
     *
     * **`NORMAL` is not forwarded to QGC at all.** DJI uses it for entries that are present and
     * fine, and forwarding them would put routine chatter into the one channel this bridge has for
     * saying something is wrong. It is still recorded, published and logged, because "DJI listed
     * this at NORMAL" is evidence and evidence is what the flight record is for. This is the only
     * level that is dropped, and it is dropped on the wire only. **It is also how a source says
     * "nothing wrong here"** — the wind source maps DJI's `LEVEL_0` onto it — which is what makes a
     * clear announceable rather than silent (see [WarnChange] and [WarningMonitor]).
     *
     * **`WARNING` maps to `ERROR`, not to `WARNING`.** One step *up*, deliberately. A
     * MAV_SEVERITY_WARNING is a yellow line in a scrolling panel — spoken once, then gone — and the
     * entire reason this package exists is that an operator did not notice a DJI warning for an
     * hour while the aircraft force-landed under them. What `ERROR` adds is the red modal and the
     * toolbar error count: a claim that *stays on screen* until it is dismissed.
     *
     * **`UNKNOWN` is forwarded, at `WARNING`.** DJI could not map its own code. That is not
     * "everything is fine" and must not be silently dropped. It is not given the modal either: we
     * do not know that it is serious, and claiming so would be the same invention in the other
     * direction.
     *
     * @return the `MAV_SEVERITY` to send at, or `null` for "never forwarded".
     */
    fun mavSeverity(): Int? = when (this) {
        NORMAL -> null
        NOTICE -> MAV_SEVERITY_INFO
        CAUTION -> MAV_SEVERITY_WARNING
        WARNING -> MAV_SEVERITY_ERROR
        SERIOUS_WARNING -> MAV_SEVERITY_CRITICAL
        UNKNOWN -> MAV_SEVERITY_WARNING
    }

    /** True when [mavSeverity] would forward this level to a ground station. */
    fun forwardable(): Boolean = mavSeverity() != null

    /**
     * The flight record's own severity. Aligned with [mavSeverity] so a reader of the jsonl and an
     * operator watching QGC are looking at the same judgement, with the one difference that **the
     * record keeps `NORMAL`** — at `info`, which is what it is.
     *
     * Returns a `LogEntry.SEV_*` string, spelled out rather than imported so this package depends
     * on nothing; `record/LogEntryTest` pins the two against each other.
     */
    fun recordSeverity(): String = when (mavSeverity()) {
        MAV_SEVERITY_CRITICAL, MAV_SEVERITY_ERROR -> "error"
        MAV_SEVERITY_WARNING -> "warn"
        else -> "info"
    }

    /**
     * **The `diagnostic_msgs.DiagnosticStatus` level for the Zenoh bus** — 0 OK, 1 WARN, 2 ERROR,
     * 3 STALE, exactly as the LCM definition declares them.
     *
     * Derived from the same ladder as [mavSeverity] so the bus and the ground station cannot
     * disagree about how bad something is. Two rows are judgements rather than arithmetic:
     *
     *  - **`NOTICE` is `OK`.** ROS's ladder has no informational rung, and calling a notice a WARN
     *    would make a DiMOS consumer's "any WARN" check fire on the simulator banner. The DJI word
     *    survives in the status's `values`, where a consumer that cares can read it.
     *  - **`UNKNOWN` is `STALE`.** ROS's own meaning for STALE is "this diagnostic has no usable
     *    reading", which is exactly what DJI failing to map its own code is. It is not `OK`,
     *    because absence of evidence is not good news, and not `ERROR`, because we do not know.
     */
    fun diagnosticLevel(): Byte = when (this) {
        NORMAL -> DIAGNOSTIC_OK
        NOTICE -> DIAGNOSTIC_OK
        CAUTION -> DIAGNOSTIC_WARN
        WARNING -> DIAGNOSTIC_ERROR
        SERIOUS_WARNING -> DIAGNOSTIC_ERROR
        UNKNOWN -> DIAGNOSTIC_STALE
    }

    companion object {
        /**
         * DJI's enum name → ours, with anything unrecognised becoming [UNKNOWN].
         *
         * Sources pass a **name**, never the enum itself and never an ordinal, so a future SDK that
         * adds a level lands on [UNKNOWN] — visible and forwarded — instead of throwing inside a
         * DJI callback on the main thread. (`WindWarning.find(int)` in the 5.18.0 jar *mutates* the
         * `UNKNOWN` singleton's value field when nothing matches, which is a second reason nothing
         * here touches DJI's integers.)
         */
        fun ofName(name: String?): WarnLevel = entries.firstOrNull { it.name == name } ?: UNKNOWN

        // MAV_SEVERITY values, from the MAVLink common dialect. Spelled as integers rather than as
        // the dronefleet enum so this package stays free of the MAVLink library; `Bridge` converts
        // at the wire.
        const val MAV_SEVERITY_CRITICAL = 2
        const val MAV_SEVERITY_ERROR = 3
        const val MAV_SEVERITY_WARNING = 4
        const val MAV_SEVERITY_INFO = 6

        // diagnostic_msgs.DiagnosticStatus's own constants, from
        // `dimos_lcm/lcm_files/diagnostic_msgs_DiagnosticStatus.lcm`.
        const val DIAGNOSTIC_OK: Byte = 0
        const val DIAGNOSTIC_WARN: Byte = 1
        const val DIAGNOSTIC_ERROR: Byte = 2
        const val DIAGNOSTIC_STALE: Byte = 3
    }
}

/**
 * **One thing the aircraft is currently saying is wrong**, in the shared vocabulary.
 *
 * Sources build these and nothing else. Every field is either DJI's own word or a measurement DJI
 * supplied; there is no field here for an opinion of ours, which is deliberate — the opinions
 * ([WarnEvent.mavSeverity], the sentence, whether anything is announced at all) are made once,
 * downstream, by code that can be tested without an aircraft.
 *
 * For device health the fields are literally what `IDJIDeviceHealthInfo` exposes (verified by
 * `javap`, 2026-07-26): `informationCode()`, `componentId()`, `sensorIndex()`, `title()`,
 * `description()`, `warningLevel()`. [title] and [description] are nullable because DJI builds them
 * by table lookup (`DJIDeviceHealthInfo.updateDesDescription`) and an unmapped code yields a blank
 * string; the [name] fallback is what keeps an unmapped code legible rather than announcing an
 * empty sentence.
 */
data class Warning(
    /** Which DJI subsystem said it. Part of [key]; never inferred, never defaulted. */
    val source: WarnSource,

    /**
     * The source's own identifier for this warning, verbatim: a device-health code (`"0x1600A0"`),
     * or the DJI key name for a source with a single warning (`"windWarning"`).
     */
    val code: String,

    /**
     * **DJI's own state word, verbatim** — `"WARNING"` from the health manager's `WarningLevel`,
     * `"LEVEL_2"` from `WindWarning`. Not our [level], which is the translation.
     *
     * Both are kept because both are load-bearing: the record and the bus carry DJI's word, which
     * is the searchable one and the one a DJI forum post will use; the operator's sentence carries
     * ours, which is the one they can act on. Collapsing them would mean either announcing
     * `LEVEL_2` to a pilot or writing `CAUTION` into a record where DJI never said it.
     */
    val state: String,

    val level: WarnLevel,

    /** DJI's own title, where DJI supplies one. */
    val title: String? = null,

    /** DJI's own description, where DJI supplies one. */
    val description: String? = null,

    /** Which component raised it — DJI's `componentId()`. Null for sources with no components. */
    val componentId: Int? = null,

    /** Which sensor of that component — DJI's `sensorIndex()`. */
    val sensorIndex: Int? = null,

    /**
     * **The number that belongs with this warning, already formatted with its unit** — `"14.2 m/s"`
     * for a wind warning — or null when the source has no measurement to offer.
     *
     * There is exactly one of these because there is exactly one *fact* worth putting in fifty
     * bytes next to the words. It exists because of landing14: DJI's own wind warning stayed
     * `LEVEL_0` through a 9.1 m/s incident that overwhelmed lateral control, and the finding on the
     * record is *"the warning is not a substitute for the number"*. A level without its measurement
     * is exactly the report that failed then.
     *
     * **Never invented and never a second source.** The wind figure is `KeyWindSpeed`'s own dm/s
     * reading converted once ([WindWarnings]); a source with nothing measured passes null, and null
     * means nobody measured it, never zero.
     */
    val measurement: String? = null,
) {
    /**
     * The identity a warning keeps across deliveries.
     *
     * **The source, component and sensor are all part of it, and each is load-bearing.** DJI
     * reports per-component faults with the same [code] — four ESCs, two IMUs, two compasses — and
     * keying on the code alone would collapse "all four motors overheating" into one warning and
     * make three of them invisible; it would also produce a fake `changed` every time DJI happened
     * to order the list differently. The source is there so that two subsystems that both number
     * something `1` cannot silently become one warning.
     */
    val key: String get() = "${source.name}/$code/$componentId/$sensorIndex"

    /**
     * What an operator should read: DJI's own title, then DJI's own description, then DJI's code.
     *
     * DJI's word survives, ours does not — `command/StatusTexts`' shortening rule. We never invent
     * a friendly sentence; every branch here is something DJI said.
     *
     * **The description fallback was added 2026-07-27, from the first real health messages this
     * project ever received, and it is the difference between the feature working and not.**
     * Measured on hardware: DJI populated `description` on every one of them and `title` on none.
     *
     * | code | title | description |
     * |---|---|---|
     * | `0x16100013` | *(blank)* | "Running Flight Simulator. Restart aircraft to take off" |
     * | `0x1B030010` | *(blank)* | "Obstacle sensing not available at night. Adjust RTH altitude…" |
     * | `0x16100089` | *(blank)* | "Remote controller not bound/still confirming binding status" |
     *
     * Without this the operator gets `DJI: 0x16100013` — a hex code for a message whose whole
     * purpose is to be read at a glance while an aircraft is doing something unexpected.
     *
     * **The first sentence, where there is one.** These descriptions are two clauses: what is
     * wrong, then what to do about it. The first is what fits in fifty bytes and is the half that
     * identifies the problem — "Running Flight Simulator." is 25 bytes and complete; the advice
     * that follows is what the flight record is for. Where there is no sentence break, the clamp
     * takes it from the front as it always has.
     *
     * The raw code remains the last resort and is still the right one: it is searchable, and it is
     * DJI's, which a sentence of ours would not be.
     */
    val name: String get() =
        title?.takeIf { it.isNotBlank() }
            ?: description?.takeIf { it.isNotBlank() }?.let { firstSentence(it) }
            ?: code

    private fun firstSentence(text: String): String {
        val stop = text.indexOf(". ")
        // Only when the head is substantial enough to stand alone — a stray early period must not
        // reduce a warning to two words.
        return if (stop >= MIN_SENTENCE_CHARS) text.substring(0, stop + 1) else text
    }

    companion object {
        /**
         * How much of a description has to precede a full stop before it is taken as the sentence.
         *
         * Guards against an abbreviation or a version number cutting a warning off at its first
         * dot. Measured against the three real descriptions this project has seen, whose first
         * sentences are 24, 39 and (no break) 58 characters.
         */
        const val MIN_SENTENCE_CHARS = 12
    }
}

/** What happened to a warning between two deliveries. */
enum class WarnChange {
    /** A key that was not in the previous picture. */
    APPEARED,

    /** A key that was already there, at a different [WarnLevel]. */
    CHANGED,

    /** A key that was in the previous picture and is not in this one. */
    CLEARED,
}

/**
 * One change, decided and dressed: what happened, to what, how it is to be announced, and whether
 * the wire actually gets it.
 *
 * Produced only by [WarningMonitor.onDelivery] — there is no other constructor call in the app — so
 * every field below is the core's decision rather than a shell's, and **every surface renders from
 * this one object**. That is the property [WarningBus] exists to hold: QGC's severity, the record's
 * severity, the phone's line and the bus's `DiagnosticStatus` are four views of one decision, and
 * none of them may recompute it.
 */
data class WarnEvent(
    val change: WarnChange,
    /** For [WarnChange.CLEARED], the warning as it was last seen. */
    val warning: Warning,
    /** Set only for [WarnChange.CHANGED]. */
    val previousLevel: WarnLevel? = null,
    /**
     * `MAV_SEVERITY` for the `STATUSTEXT`, or `null` when this event is never forwarded to a ground
     * station at all (see [WarnLevel.mavSeverity]). A `null` here is not a failure — it is a
     * decision, and the event is still recorded, published and logged.
     */
    val mavSeverity: Int?,
    /** `LogEntry.SEV_*` for the flight record. Always set; the record never drops anything. */
    val recordSeverity: String,
    /**
     * The `diagnostic_msgs.DiagnosticStatus.level` for the Zenoh bus. Always set, for the reason
     * [recordSeverity] always is: the bus, like the record, carries everything.
     */
    val diagnosticLevel: Byte,
    /** The exact sentence, already clamped to `STATUSTEXT`'s 50 bytes. */
    val text: String,
    /**
     * True when this event both *may* be forwarded ([mavSeverity] non-null) and won a token from
     * the rate bound. False with a non-null [mavSeverity] means the bound suppressed it: the flight
     * record, the bus and logcat still carry it, the ground station does not.
     */
    val announce: Boolean,
    /**
     * True when [announce] is false *because of the rate bound* rather than because this level is
     * never forwarded. Written into the flight record so a post-mortem can tell "QGC was never
     * told" from "QGC is never told about this level".
     */
    val rateLimited: Boolean = false,
)
