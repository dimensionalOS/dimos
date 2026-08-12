package com.dimensional.mini4pro.command

/**
 * Composes the operator-facing sentences that carry a DJI refusal to QGroundControl, inside the
 * one channel that exists for them.
 *
 * `STATUSTEXT.text` is **50 bytes, hard** — a fixed-width `char[50]` in the MAVLink XML, with no
 * length prefix and no continuation on a message we send once. Anything longer is not an error,
 * it is silently cut on the wire, which is worse: the operator reads a truncated error name and
 * searches for the wrong string. `HandshakeResponder.MODE_REFUSAL_TEXT` is pinned against this
 * limit by an existing test (`HandshakeResponderTest.kt:760`); everything built here is
 * generated at runtime from a DJI string of unknown length, so it needs the limit enforced
 * rather than counted by hand.
 *
 * ## The shortening rule
 *
 * **DJI's word survives; ours does not.** `docs/decisions/2026-07-25-m2-command-safety.md` §Q3 is
 * that the operator should read DJI's own name for the refusal rather than a paraphrase of ours,
 * so when the full sentence does not fit, the framing is dropped and the bare error name goes
 * out alone. Only if DJI's name *by itself* exceeds 50 bytes is it cut, and then on a whole-
 * character boundary so the text is still valid UTF-8.
 *
 * The measured names are comfortable: `FC_AUTH_STATE`, `GPS_DISCONNECT` and
 * `NAV_SYS_EXCEPTION` were all captured during the 19:16 session on 2026-07-25 and all fit the
 * long form with room to spare. The clamp is for the ones nobody has seen yet — `FCFlightMode`
 * alone has 79 constants (`Px4Mode.kt:50`), so DJI is entirely capable of naming something long.
 */
object StatusTexts {

    /** The `STATUSTEXT.text` field width, in bytes. Not characters. */
    const val MAX_BYTES = 50

    /**
     * What the operator reads when the bridge auto-confirms its own landing past DJI's ~0.5 m
     * stall — the one automatic command this project sends, and therefore the one that must
     * never happen silently. 25 bytes, counted, and pinned by a test like
     * `MODE_REFUSAL_TEXT`. "0.5m" is DJI's own documented figure for
     * `KeyIsLandingConfirmationNeeded` (its `KeyConfirmLanding` page says 0.7 m — the
     * disagreement is DJI's, recorded in `docs/msdk/actions.md`; the announcement quotes the
     * key that actually triggers the confirm).
     */
    const val LANDING_CONFIRMED = "Landing confirmed at 0.5m"

    /**
     * "Return sent to DJI" — we handed the command over, and that is the whole claim.
     *
     * Added 2026-07-26 by measurement (`docs/measurements/2026-07-26-m2-first-command.md`). An
     * operator pressed Return with the interlock on, the dispatcher called
     * `performAction(KeyStartGoHome)` three times, and **DJI never invoked the callback** — no
     * `onSuccess`, no `onFailure`, `goHomeState` stuck at `IDLE`. Not one `STATUSTEXT` went out in
     * the whole session.
     *
     * A fourth press with 14 satellites locked made the aircraft's side legible: **the RC beeped.**
     * The flight controller had received the request and rejected it, raising its alert to the
     * pilot holding the remote rather than to the SDK. So the failure this sentence fixes is not
     * "MSDK lost the command" — it is that the DJI pilot was told and the QGC operator was not,
     * leaving two people watching one aircraft with different beliefs about it.
     *
     * The design said [ActionOutcome.Requested] should be silent because the flight mode changing
     * in the heartbeat is the honest confirmation; that reasoning assumed DJI would eventually say
     * something *to us*. It does not have to. Silence then leaves the operator unable to tell
     * "sent" from "this bridge is broken", which is the one distinction this layer exists to
     * preserve.
     *
     * **The verb is load-bearing.** "sent to DJI" is a report about *us* — the mirror of
     * [refusal]'s "refused **by** DJI", so the two sentences together always say who did what.
     * There is deliberately no "Returning home", no "RTH engaged", no "Return started": nothing on
     * this side of the seam knows whether the aircraft agreed, and [ActionOutcome]'s KDoc forbids
     * naming an outcome after a state we cannot observe. The confirmation is still the flight mode
     * arriving from the aircraft in a later heartbeat; this only says the request left the bridge.
     *
     * 18 bytes for Return, 16 for Land — counted, and pinned by a test like [LANDING_CONFIRMED].
     * Composed rather than declared so a third action cannot be added without a sentence.
     */
    fun dispatched(action: FlightAction): String = "${action.label} sent to DJI"

    /**
     * "Takeoff: DJI goes to 1.2m, not 3.0m" — the height the operator asked for, beside the only
     * height this aircraft has.
     *
     * **The single most important sentence this object composes.** QGC's Takeoff button asks for
     * an altitude and gets an acknowledgement back; without this line, an `ACCEPTED` would let an
     * operator believe a 3 m takeoff was agreed to when DJI will deliver ~1.2 m and stop. The
     * discrepancy is in the safe direction — lower than commanded, with the RC pilot's authority
     * untouched — but "safe" is not "as asked", and PLAN.md's honesty boundaries do not have an
     * exemption for differences that happen to be favourable.
     *
     * Both numbers, always, and never a claim that either has happened. The verb is deliberately
     * absent: this says what DJI's takeoff *is*, not that the aircraft is doing it — the same
     * discipline as [dispatched].
     *
     * The two are always different in practice, so there is no "they match" case to suppress:
     * QGC's takeoff slider is floored at `Vehicle::minimumTakeoffAltitudeMeters()`, which PX4 does
     * not override, so it is `FirmwarePlugin.h:204`'s **3.048 m** and an operator cannot type
     * 1.2 m even if they wanted to. That is also why refusing a mismatched altitude outright was
     * rejected: it would refuse every takeoff QGC can send.
     *
     * 35 bytes at one decimal place for a single-digit request, 37 at three digits — inside the
     * 50-byte field across QGC's whole slider range of 3.048 m to 121.92 m
     * (`FlyView.SettingsGroup.json`), and clamped anyway.
     */
    fun takeoffHeight(requestedRelativeM: Double, djiHeightM: Double): String = clamp(
        "Takeoff: DJI goes to ${oneDecimal(djiHeightM)}m," +
            " not ${oneDecimal(requestedRelativeM)}m"
    )

    /**
     * "Takeoff: DJI 1.2m, then we climb to 3.0m" — the two-phase takeoff, said before the aircraft
     * moves.
     *
     * **[takeoffHeight]'s replacement wherever a climb is actually armed, and the pair must never
     * both go out.** Once the second phase exists, "DJI goes to 1.2m, not 3.0m" stops being the
     * honest sentence and becomes a lie in the other direction: the aircraft *will* reach 3.0 m,
     * a few seconds later and under a different controller. [CommandDispatcher.onTakeoff] chooses
     * between the two on [ClimbArm], which is the only thing that knows whether anything was
     * armed.
     *
     * Both numbers again, and again no verb claiming either has happened — the same discipline as
     * [dispatched] and [takeoffHeight]. The "we" is deliberate and is the one place a sentence in
     * this project says it: the climb is flown by **this bridge**, not by DJI's takeoff, and an
     * operator watching the aircraft stop at 1.2 m and then start moving again is owed the fact
     * that the second movement is ours.
     *
     * 40 bytes at one decimal for a single-digit request, 42 at three digits — inside the 50-byte
     * field across QGC's whole slider range, and clamped anyway.
     */
    fun takeoffThenClimb(climbToRelativeM: Double, djiHeightM: Double): String = clamp(
        "Takeoff: DJI ${oneDecimal(djiHeightM)}m," +
            " then we climb to ${oneDecimal(climbToRelativeM)}m"
    )

    /**
     * "Takeoff climb capped at 100m" — the requested height was above the M3 ceiling
     * (`GuidedEnvelope.CEILING_M`) and the climb was bound to it.
     *
     * Capped and **announced**, never silently clamped: the JC-2/JC-5 split the goto and the orbit
     * already apply to their own altitudes. It is sent *in addition to* [takeoffThenClimb], which
     * carries the number that will actually be flown to, so the operator reads both what they get
     * and that it is not what they asked for.
     *
     * The number lives in the caller rather than here because this object must not import
     * `guided/`; the ceiling is `GuidedEnvelope.CEILING_M`, which moved from 30 m to 100 m on
     * 2026-07-27 and will move again.
     */
    fun takeoffClimbCapped(ceilingM: Double): String = clamp(
        "Takeoff climb capped at ${ceilingM.toInt()}m"
    )

    /**
     * "Takeoff failed: 500.0m not in 0-122m" — the requested height, once converted back out of
     * QGC's AMSL, is not a height this bridge will treat as a takeoff.
     *
     * The number is in the sentence because the operator's next question is *which* number was
     * wrong, and because a height far outside the band is the one visible symptom of the failure
     * this check exists for: a `param7` composed against a datum that is **not the one we
     * published** (`docs/measurements/2026-07-26-amsl-datum.md`). DJI's "AMSL" is pressure
     * altitude on the 1013.25 hPa reference and was 14 m high one day and 28 m low the next, so
     * an AMSL from a saved plan, a terrain lookup or a survey is in a different datum by tens of
     * metres and must be refused rather than converted.
     */
    fun takeoffAltOutOfRange(relativeM: Double, maxM: Double): String =
        if (relativeM <= 0.0) {
            // **A negative height has one cause and the operator can act on it.** It means the
            // GCS's `param7` sits *below* the AMSL we published, so the two are working from
            // different datums — and this happened in real air on 2026-07-28 at 09:46:26, where
            // the bare number "-49.5m outside 0..121.92m" told Ivan nothing he could use. The
            // fix is to make the GCS re-read home, and the sentence should say which way the
            // disagreement runs so that is the obvious thing to try.
            //
            // "Takeoff failed: GCS datum 49.5m below ours" is 41 bytes at that magnitude and
            // clamp() holds the 50-byte STATUSTEXT budget for the rest.
            clamp(
                "${FlightAction.TAKEOFF.label} failed: GCS datum" +
                    " ${oneDecimal(-relativeM)}m below ours"
            )
        } else {
            clamp(
                "${FlightAction.TAKEOFF.label} failed: ${oneDecimal(relativeM)}m" +
                    " not in 0-${maxM.toInt()}m"
            )
        }

    /**
     * What an operator reads when anything asks this bridge to **arm** — including QGC asking on
     * its own initiative, one ack after an accepted takeoff. 42 bytes, counted.
     *
     * `MAV_CMD_COMPONENT_ARM_DISARM param1=1` is refused, always, in every interlock state, and
     * that refusal is correct rather than a gap: **MSDK 5.18.0 has no arm vocabulary at all**
     * (`docs/msdk/actions-rth-and-arming.md` §6). The nearest thing, `KeyTurnOnTheMotor`, is
     * undocumented, has no symmetric stop, and would be this bridge spinning propellers nobody
     * asked it to — so accepting the command could only ever be a lie or a hazard.
     *
     * The sentence exists because of the pairing. `PX4FirmwarePlugin::_mavCommandResult` sends
     * that arm within milliseconds of our `ACCEPTED` on `MAV_CMD_NAV_TAKEOFF`, so an operator who
     * has just had a takeoff accepted gets a modal error about arming a second later, and the
     * obvious reading — "the takeoff failed" — is wrong and could make them press things. This
     * says the true thing instead: DJI arms as part of its own takeoff, so there is nothing for a
     * separate arm to do.
     *
     * Sent for a plain Arm press too, not only after a takeoff, and unconditional on
     * [CommandInterlock] — same reasoning as [CommandDispatcher.EMERGENCY_STOP_TEXT]. It is
     * information rather than actuation, and it is equally true in every state, so gating it on a
     * timing window would only create states in which a true and useful sentence is withheld.
     */
    const val NO_SEPARATE_ARM = "DJI has no arm - takeoff starts the motors"

    /**
     * One decimal place, always, and with a `.` regardless of locale — `String.format` without an
     * explicit [java.util.Locale] renders `3,0` in half of Europe, which is where this runs.
     */
    private fun oneDecimal(value: Double): String =
        String.format(java.util.Locale.ROOT, "%.1f", value)

    /**
     * "Return refused by DJI: FC_AUTH_STATE" — the aircraft was asked and said no.
     *
     * "by DJI" is in there because the operator's next question is always *who refused*: our own
     * interlock and the flight controller are both plausible, and they lead to different
     * actions. The interlock's own refusal says "bridge is telemetry-only" and is a different
     * sentence entirely, produced by `HandshakeResponder`, not here.
     */
    fun refusal(action: FlightAction, djiError: String): String =
        preferring("${action.label} refused by DJI: $djiError", djiError)

    /**
     * "Land failed: NO_AIRCRAFT" — we could not put the question to the flight controller.
     *
     * A different verb from [refusal] on purpose. "Refused" attributes a decision to the
     * aircraft; if no aircraft was reachable there was no decision, and saying so would invent
     * one.
     */
    fun unavailable(action: FlightAction, reason: String): String =
        preferring("${action.label} failed: $reason", reason)

    /**
     * "Return failed: IllegalStateException" — the implementation of [FlightActions] threw.
     *
     * A thrown exception is the shape an unwritten or half-written DJI layer takes, so this is
     * the string the operator sees when M2's other half is stubbed. It reports a failure, never
     * a success: the whole point of catching is that the command did not happen.
     */
    fun threw(action: FlightAction, throwable: Throwable): String {
        val detail = throwable.message?.takeIf { it.isNotBlank() }
            ?: throwable::class.java.simpleName
        return preferring("${action.label} failed: $detail", detail)
    }

    /**
     * An asynchronous DJI error, arriving outside any command — `motorStartFailureError` and
     * friends, which the recorder saw fire on their own during the 19:16 session.
     */
    fun djiError(djiError: String): String = preferring("DJI: $djiError", djiError)

    /**
     * The long form if it fits, otherwise the essential part alone, otherwise the essential part
     * cut to fit. Never returns anything over [MAX_BYTES].
     */
    private fun preferring(full: String, essential: String): String =
        if (full.utf8Size() <= MAX_BYTES) full else clamp(essential)

    /**
     * Cuts a string to [MAX_BYTES] bytes without splitting a UTF-8 character in half.
     *
     * DJI's enum names are ASCII, so in practice this never has to think. It is written for
     * bytes anyway because the field is specified in bytes and because a half-character at the
     * end of a `char[50]` is a decoding failure on the ground station rather than a short
     * string.
     */
    fun clamp(text: String): String {
        if (text.utf8Size() <= MAX_BYTES) return text
        var end = text.length
        while (end > 0 && text.substring(0, end).utf8Size() > MAX_BYTES) {
            end--
            // Never leave a lone high surrogate: it encodes to a replacement character and
            // would push the size back up.
            if (end > 0 && text[end - 1].isHighSurrogate()) end--
        }
        return text.substring(0, end)
    }

    private fun String.utf8Size(): Int = toByteArray(Charsets.UTF_8).size
}
