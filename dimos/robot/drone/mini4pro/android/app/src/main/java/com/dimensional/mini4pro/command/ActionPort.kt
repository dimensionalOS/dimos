package com.dimensional.mini4pro.command

/**
 * The thin seam between [MsdkFlightActions] — every decision the DJI half of M2 makes — and the
 * five `KeyManager` calls those decisions ride on.
 *
 * **This interface contains no DJI types and must never contain any**, for the same reason
 * [FlightActions] doesn't: the MSDK is not on the unit-test classpath (`docs/architecture.md`),
 * so anything on the DJI side of a seam cannot be tested, and the decisions that keep a landing
 * honest — which landings we may auto-confirm, when the confirm has been sent, what an operator
 * is told — are exactly the code that must be. `KeyManagerActionPort` is the production
 * implementation and is *deliberately too dumb to test*: one method per key, no branches beyond
 * unwrapping a callback. If a method here ever wants an `if`, the `if` belongs in
 * [MsdkFlightActions] where a fake port can reach it.
 *
 * The six keys behind these methods are all `[class, verified twice]` against the 5.18.0 jar —
 * `docs/msdk/actions.md`'s table — and were re-checked by `javap` against
 * `dji-sdk-v5-aircraft-provided-5.18.0.jar` when this seam was written, `KeyStartTakeoff` again
 * on 2026-07-26 when takeoff was added:
 *
 * | method | key | shape |
 * |---|---|---|
 * | [startTakeoff] | `FC.KeyStartTakeoff` | `performAction`, `EmptyMsg→EmptyMsg` |
 * | [startGoHome] | `FC.KeyStartGoHome` | same |
 * | [startAutoLanding] | `FC.KeyStartAutoLanding` | same |
 * | [confirmLanding] | `FC.KeyConfirmLanding` | same |
 * | [listenIsLandingConfirmationNeeded] | `FC.KeyIsLandingConfirmationNeeded` | `Boolean`, get/listen |
 * | [listenIsInLandingMode] | `FC.KeyIsInLandingMode` | `Boolean`, get/listen |
 * | [canStartTakeoff], [canStartGoHome], [canStartAutoLanding] | the same three action keys | `DJIKey.canPerformAction()` |
 *
 * The last row is a **static declaration, not a pre-flight check** — see [canStartGoHome] for the
 * bytecode. MSDK 5.18.0 has no synchronous way to ask whether an action will be honoured now.
 *
 * **`EmptyMsg→EmptyMsg` on the first row is the whole answer to "what altitude?"** — see
 * [canStartTakeoff] for the bytecode and [FlightActions.takeoff] for what the MAVLink half does
 * about it.
 */
interface ActionPort {

    companion object {
        /**
         * **How to read `dji-sdk-v5-aircraft-provided-5.18.0.jar`, and the trap in it.** Every
         * bytecode citation in this package assumes this; a reader who repeats a `javap` without
         * it will reach the opposite conclusion and believe they have caught us in an error.
         *
         * The `-provided` jar is a **compile-only ABI stub**. Every *method body* has a
         * return-default injected in front of it, with the real implementation left behind as
         * unreachable dead code:
         *
         * ```
         * // DJIKey.canPerformAction()
         * 0: iconst_0
         * 1: ireturn                     <-- injected stub. NOT what runs on a phone.
         * 2: aconst_null                 <-- the real body starts here
         * 3: aload_0
         * 4: getfield      mKeyInfo
         * 7: if_acmpeq     24
         * 14: invokevirtual DJIKeyInfo.isCanPerformAction:()Z
         * ...
         *
         * // DJIKeyInfo.isCanPerformAction()
         * 0: iconst_0
         * 1: ireturn                     <-- injected stub
         * 2: aload_0
         * 3: getfield      canPerformAction
         * 6: ireturn                     <-- the real body: return the field
         * ```
         *
         * So **"javap shows `return X`" is evidence about the linker, not about the aircraft.**
         * The useful part is the dead code from offset 2 onward, and it is the best MSDK
         * documentation that exists — better than DJI's own reference, which this project has
         * caught out repeatedly.
         *
         * **Static initialisers are exempt and are therefore the solid ground.** `<clinit>` is
         * not stubbed — `DJIFlightControllerKey.<clinit>` begins at offset 0 with `new
         * java/util/ArrayList` and runs for ~42 000 bytes of genuine builder calls. Every claim
         * this package makes about a key's *declaration* — `canGet`, `canSet`, `canListen`,
         * `canPerformAction`, the identifier string, and the value converters that decide whether
         * an action takes a parameter — is read from there, where the data is real.
         *
         * The rule for anyone adding a citation: **say which form you are quoting.** "the stub"
         * or "the real body at offset 2" or "the static initialiser". A bare offset table is
         * ambiguous exactly where it matters.
         */
        const val READING_THE_JAR =
            "the -provided jar is a compile-only ABI stub: method bodies have an injected " +
                "return-default at offset 0 and the real implementation as dead code from " +
                "offset 2. Static initialisers are not stubbed."
    }

    /**
     * Why the MSDK cannot be asked anything right now, or null if it can.
     *
     * Checked before every action. The two production reasons are `SDK_NOT_REGISTERED` and
     * `NO_PRODUCT`; both are statements about *us*, which is why they surface as
     * [ActionOutcome.Unavailable] rather than [ActionOutcome.Refused] — no flight controller was
     * consulted.
     */
    fun unavailableReason(): String?

    /**
     * Whether `FC.KeyStartGoHome` declares itself performable — `DJIKey.canPerformAction()`.
     *
     * **Read what this is before trusting it.** It is *not* a pre-flight check, and DJI does not
     * offer one. `canPerformAction` is a constructor-chain flag baked into the key at class-load
     * time, verified straight out of `dji-sdk-v5-aircraft-provided-5.18.0.jar`:
     *
     * ```
     * // static initialiser (not stubbed — see READING_THE_JAR)
     * // DJIFlightControllerKey.<clinit>, the KeyStartGoHome entry
     * 693: iconst_1
     * 694: invokevirtual  DJIActionKeyInfo.canPerformAction:(Z)LDJIActionKeyInfo;
     * 701: putstatic      KeyStartGoHome
     *
     * // the REAL bodies, read from offset 2 past each injected `return false` stub:
     * // DJIKey.canPerformAction() — mKeyInfo != null && mKeyInfo.isCanPerformAction()
     * // DJIKeyInfo.isCanPerformAction() — return this.canPerformAction   (the field above)
     * ```
     *
     * Both of those methods disassemble to `iconst_0; ireturn` at offsets 0–1. **That is the ABI
     * stub, not the behaviour** ([READING_THE_JAR]); taking it at face value would say this
     * method can never return true, which is the opposite of the truth.
     *
     * So for this key it is the constant `true`, on the ground, in the air, connected or not. It
     * cannot report "the aircraft will not do this right now" — the only structured refusal
     * channel an action has is `CompletionCallback.onFailure`, asynchronously, and
     * `docs/msdk/actions.md` says so explicitly: *"there is no separate precondition check API to
     * call first."* `KeyManager.isKeySupported` is not one either; its **real** body — offsets
     * 2–3, past the injected stub — is `iconst_1; ireturn`, i.e. `return true` for every key ever
     * passed to it.
     *
     * It is asked anyway, and it earns its place narrowly: it is DJI's own declaration about the
     * key we are one line away from performing, so if an SDK upgrade ever demotes `KeyStartGoHome`
     * or someone wires a non-action key here, the action fails closed with a name instead of
     * disappearing into a callback that never fires. [MsdkFlightActions] decides what a `false`
     * means; this only reports the flag.
     */
    fun canStartGoHome(): Boolean

    /** As [canStartGoHome], for `FC.KeyStartAutoLanding`. Also the constant `true` in 5.18.0. */
    fun canStartAutoLanding(): Boolean

    /**
     * As [canStartGoHome], for `FC.KeyStopAutoLanding` — `[class, verified twice]` in
     * `docs/msdk/actions.md` (`canPerformAction` only, `EmptyMsg→EmptyMsg`; djidoc: *"the
     * aircraft will stop landing and hover at the current altitude"*). Wired 2026-07-28 for
     * Stage C's rule-1 withdrawal of a committed autoland; QGC deliberately still has no
     * Cancel button ([FlightActions]' KDoc — that decision stands untouched).
     */
    fun canStopAutoLanding(): Boolean

    /**
     * As [canStartGoHome], for `FC.KeyStartTakeoff`. Also the constant `true` in 5.18.0, and
     * verified from the jar rather than from the docs — `canPerformAction` is a compile-time
     * declaration, not the pre-flight check its name suggests.
     *
     * **Read [READING_THE_JAR] before you repeat this `javap`.** What follows is a *static
     * initialiser*, which the ABI stub leaves intact; method bodies in the same jar are not, and
     * quoting one of those the same way would produce a confident, wrong conclusion.
     *
     * `javap -c -p dji.sdk.keyvalue.key.DJIFlightControllerKey`, the `KeyStartTakeoff` entry of
     * `<clinit>`, offsets 210–257, transcribed in full because two separate claims rest on it —
     * that the action is performable, and that it **takes no argument**:
     *
     * ```
     * 210: new           #90   // class dji/sdk/keyvalue/key/DJIActionKeyInfo
     * 226: ldc           #92   // String StartTakeoff
     * 228: getstatic     #94   // EmptyValueConverter.converter   <-- param converter
     * 231: getstatic     #94   // EmptyValueConverter.converter   <-- result converter
     * 234: invokespecial #100  // DJIActionKeyInfo."<init>":(IILjava/lang/String;
     *                          //   Ldji/sdk/keyvalue/converter/IDJIValueConverter;
     *                          //   Ldji/sdk/keyvalue/converter/IDJIValueConverter;)V
     * 237: iconst_0
     * 238: invokevirtual #103  // canGet(false)
     * 241: iconst_0
     * 242: invokevirtual #106  // canSet(false)
     * 245: iconst_0
     * 246: invokevirtual #108  // canListen(false)
     * 249: iconst_1
     * 250: invokevirtual #110  // canPerformAction(true)
     * 253: iconst_0
     * 254: invokevirtual #112  // setIsEvent(false)
     * 257: putstatic     #114  // Field KeyStartTakeoff
     * ```
     *
     * The declared type is `DJIActionKeyInfo<EmptyMsg, EmptyMsg>`. Both converters are the same
     * `EmptyValueConverter` singleton, so **there is no field in which an altitude could be
     * sent** — DJI's fixed ~1.2 m hop is the only takeoff this key can ask for.
     *
     * A sweep of every key in `dji/sdk/keyvalue/key/` whose name contains `takeoff` confirms
     * there is no other door: `KeyStopTakeoff` and `KeyPrecisionStartTakeoff` are the same
     * `EmptyMsg→EmptyMsg` shape (and DJI's own widget performs `KeyStartTakeoff` for *both* of
     * its takeoff buttons — the "precision" one differs only in the height it displays,
     * `TakeOffWidgetModel.kt:143-154`); `KeyTakeoffLocationAltitude` is a `Double` we already read
     * as the AMSL datum; `KeyTakeoffFailureError` is telemetry. The single settable height in the
     * whole surface is `FlightAssistantKey.KeyFlyingOnShipTakeoffHeight`
     * (`Double`, get/set/listen, `canPerformAction(false)`) — a ship-deck launch setting on a
     * different component, not a parameter to this action.
     */
    fun canStartTakeoff(): Boolean

    /**
     * `performAction(FC.KeyStartGoHome)`. [onFailure] carries the `IDJIError` name verbatim and
     * arrives whenever DJI answers, on DJI's thread — possibly long after the call returns,
     * possibly never.
     *
     * **"Possibly never" is measured, not defensive.** On 2026-07-26 an operator's Return reached
     * this call four times on a connected, healthy, grounded aircraft and DJI invoked neither
     * callback, ever (`docs/measurements/2026-07-26-m2-first-command.md`). On the fourth, with 14
     * satellites, the RC beeped: the flight controller had received the request and refused it,
     * and alerted the pilot instead of the SDK. Nothing above this seam may wait on [onFailure],
     * treat its absence as success, or treat its absence as *"no decision was made"*.
     */
    fun startGoHome(onFailure: (String) -> Unit)

    /**
     * `performAction(FC.KeyStartTakeoff)` — **the one call in this project that leaves the
     * ground.** No parameter, because the key has none ([canStartTakeoff]).
     *
     * Deliberately the same shape as [startGoHome] and not [startAutoLanding]: failure only, no
     * `onSuccess`. `startAutoLanding` needs its success because that is what authorises the
     * automatic confirm at 0.5 m — a standing permission to send a *further* command. Nothing
     * downstream of a takeoff acquires any such permission, so a success callback here would
     * feed only an announcement, and the honest announcement ("Takeoff sent to DJI") is already
     * made by [CommandDispatcher] from the fact that the call was made. Adding a second, later
     * "DJI accepted" would also invite the reader to treat its absence as a refusal, and the
     * absence of a callback is measured to mean nothing at all
     * (`docs/measurements/2026-07-26-m2-first-command.md`).
     *
     * [onFailure] carries DJI's `IDJIError` name verbatim, on DJI's thread, possibly never.
     * `-7` is the one this airframe is known to produce
     * (DJI [#783](https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/783)).
     */
    fun startTakeoff(onFailure: (String) -> Unit)

    /**
     * `performAction(FC.KeyStartAutoLanding)`. [onSuccess] means DJI **accepted the start** — the
     * one fact [MsdkFlightActions] builds its "this landing is ours" claim on — not that the
     * aircraft has landed or ever will.
     */
    fun startAutoLanding(onSuccess: () -> Unit, onFailure: (String) -> Unit)

    /**
     * `performAction(FC.KeyStopAutoLanding)` — withdraw a landing mid-descent; DJI documents
     * the aircraft stopping and hovering at its current altitude. **Not always honoured**:
     * DJI's own widget refuses to offer the cancel during the six forced-landing reasons
     * (`docs/msdk/actions.md` §2), so a caller must treat the outcome as a measurement.
     * [onFailure] carries the `IDJIError` name verbatim, on DJI's thread, possibly never.
     */
    fun stopAutoLanding(onFailure: (String) -> Unit)

    /**
     * `performAction(FC.KeyConfirmLanding)` — the release for DJI's ~0.5 m stall. The single most
     * consequential call on this interface: it tells a hovering aircraft to descend the last half
     * metre. [onSuccess] gates the operator announcement, so what QGC reads is what DJI accepted,
     * not what we attempted.
     */
    fun confirmLanding(onSuccess: () -> Unit, onFailure: (String) -> Unit)

    /**
     * Subscribes to `FC.KeyIsLandingConfirmationNeeded`. [onDelivery] fires on **every**
     * delivery, repeats included, with `null` meaning DJI's component-gone signal — the same
     * listener contract `StateCache` documents. The subscriber must therefore be idempotent
     * against repeated `true`s; de-duplication is a decision and lives above this seam.
     */
    fun listenIsLandingConfirmationNeeded(onDelivery: (Boolean?) -> Unit)

    /** Subscribes to `FC.KeyIsInLandingMode`. Same delivery contract as the other listen. */
    fun listenIsInLandingMode(onDelivery: (Boolean?) -> Unit)

    /**
     * Cancels both subscriptions. Idempotent, and must be safe to call when nothing was ever
     * subscribed — `Bridge.stop()` calls it unconditionally so no listener outlives the link.
     */
    fun cancelListens()
}
