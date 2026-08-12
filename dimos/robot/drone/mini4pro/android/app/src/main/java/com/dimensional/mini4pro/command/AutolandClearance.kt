package com.dimensional.mini4pro.command

/**
 * **What the guided engine can vouch for at the instant DJI asks its landing-confirmation
 * question** — the facts [MsdkFlightActions] weighs before answering `KeyConfirmLanding` on
 * behalf of a Stage C full autoland, flattened to plain values at the `Bridge` seam so the
 * whole decision runs under `MsdkFlightActionsTest` with no engine, no detector and no
 * aircraft (the `TagDescentSense` pattern, pointing the other way across the same boundary).
 *
 * Produced by `GuidedStickEngine.autolandClearance()`: **null unless a full-autoland
 * engagement is live and committed to LANDING right now**, so a null is the strong statement —
 * nothing this bridge is flying wants a confirmation, and a `KeyIsLandingConfirmationNeeded`
 * that arrives anyway is someone else's landing (the RC pilot's, DJI's own failsafe) whose
 * question is theirs to answer. Read fresh at the moment of the question, never cached: the
 * engagement can die (rule 1) between two deliveries of the same key.
 *
 * @param engagementAtMs the engine's clock at the arm that produced this landing — identity,
 *   carried so a confirm can never be attributed to a different engagement than the one whose
 *   facts justified it.
 * @param fixAgeMs how old the newest believed tag fix is, at the moment of the read. The
 *   confirm gate requires it under [MsdkFlightActions.CONFIRM_FRESH_MS]: a confirmation is a
 *   claim that the aircraft is over the pad, and a fix old enough that the law itself would
 *   have stopped believing it cannot carry that claim.
 * @param fixWasInCone whether the last **fresh** fix sat inside the alignment cone at a known
 *   height — evaluated by the engine on every steering tick, so it is the newest statement the
 *   sensor made about "actually over the tag" rather than a re-derivation from stale numbers.
 */
data class AutolandClearance(
    val engagementAtMs: Long,
    val fixAgeMs: Long,
    val fixWasInCone: Boolean,
)
