package com.dimensional.mini4pro.guided

/**
 * **Stage C's commit seam: DJI's own landing, started and stopped** — the two calls the guided
 * engine may make into the command layer once a full autoland reaches its commit, wired by
 * `Bridge` to `MsdkFlightActions.land()` / `cancelLanding()` and faked in every unit test (the
 * [MissionTakeoff] pattern, for the same reason: `KeyStartAutoLanding` lives behind the
 * command layer's port and the engine must stay drivable without it).
 *
 * Why this exists at all — **measured, landing04 (2026-07-28)**: the FC floors a virtual-stick
 * descent at ~1.4 m via downward obstacle sensing and holds it indefinitely, never raising its
 * confirmation question; the identical stick-down from the physical RC lands. The only landing
 * DJI permits from software is its own, so the engine's commit is `land()` — the same call, the
 * same claim machinery and the same auto-confirm scope QGC's Land button uses — and rule 1's
 * withdrawal during it is `stop()`, `KeyStopAutoLanding` (`docs/msdk/actions.md` §2, verified,
 * with its own honest caveat: DJI's reference UI treats forced landings as possibly
 * uncancellable, so whether the stop is honoured is a measurement, not a promise).
 *
 * Both return **the refusal reason, or null when the ask reached DJI** (`ActionOutcome.
 * Requested`) — the engine acts on the synchronous truth and the record carries the
 * asynchronous one (`dji_call` pairs from `RecordedActionPort`). Neither call may block.
 */
interface DjiLanding {

    /**
     * Ask DJI to start its auto-landing, here, now. Null = asked; otherwise the reason the ask
     * never left (SDK unavailable, capability false, no command layer wired). Called **exactly
     * once per engagement**, on the commit edge — the single-shot latch is structural
     * (`TagDescentPhase.DJI_LANDING` has no exit and re-entry needs a fresh arm), and the
     * engine's tests pin it.
     */
    fun start(): String?

    /**
     * Ask DJI to stop the landing [start] began — rule 1's one action: manual sticks during a
     * committed landing cancel our engagement *and* withdraw the landing itself, because a
     * pilot grabbing the sticks at two metres wants the aircraft, not a race with an
     * autonomous descent we started. Null = asked. Also sent by the operator's own
     * withdrawals (disarm, pause) during a committed landing, for the same reason.
     */
    fun stop(): String?
}
