package com.dimensional.mini4pro.guided

/**
 * Which interface a command or a stick frame came in on — the identity the liveness watchdogs are
 * evaluated *within*.
 *
 * ## Why liveness needs a name for the sender
 *
 * `GuidedStickEngine` draws a distinction that keeps an unattended manoeuvre from flying: the
 * sticks going quiet while the rest of the link is alive is a **release** (standard wind-down),
 * and total inbound silence is **link loss** (the armed `LinkLossPolicy`). Both readings rest on
 * one field — when traffic was last seen — and that field has exactly one meaning while there is
 * exactly one thing that can send traffic.
 *
 * With two interfaces the sentence "traffic was seen recently" has three possible readings, and
 * `docs/zenoh-dimos-transport.md` §4.3 shows only one survives contact: **the commanding
 * controller must be alive**. "Any interface alive keeps the engagement" is the dangerous one — a
 * controller commands a manoeuvre and dies, and a live-but-idle second ground station keeps the
 * aircraft flying it, making the system *less* safe the more interfaces are attached. Traffic on
 * the interface that is not commanding is not evidence that the one that is commanding is alive.
 *
 * So the engine's watchdogs ask their question through one owner —
 * `GuidedStickEngine.controllerSeenAtLocked` — for the origin that established the current
 * engagement, and **what "alive" means is a property of the origin**, stated on each value below.
 * For a transport it is recency of inbound traffic; for [PHONE] it is identity (the argument is
 * on the value). The per-origin rule is pinned by `GuidedStickEngineTest`'s "the commanding
 * controller (`ControlOrigin`)" section and its 2026-07-29 mutation table: the three mutants that
 * were structurally unkillable while this enum had one value (hardcoded key, any-origin-alive,
 * origin never recorded) all went red the day [PHONE] made them observable.
 *
 * ## Two values, deliberately
 *
 * `ZENOH` is designed (`docs/zenoh-dimos-transport.md` §2) and not built, and it is not added
 * here in advance. An enum value that nothing constructs is a claim that a second interface
 * exists, and this project's rule is that the code says only what is true. [PHONE] is here
 * because the phone-only flight is real and was flown — landing08 is the measurement. Note that
 * [PHONE] is *not* the second transport §4.3 anticipates: it carries no inbound traffic at all,
 * so the transport-vs-transport property (A's manoeuvre must die on A's silence while B
 * chatters) remains untestable until `ZENOH` lands, and whoever adds it owes those tests —
 * `one transport today - a second one must bring the cross-transport tests with it` in
 * `GuidedStickEngineTest` is the tripwire.
 *
 * Note what this type is **not**. It is not `record.CommandSource`, which names the *message*
 * (`"MANUAL_CONTROL"`, `"DO_REPOSITION"`, and later `"zenoh:dimos/gps_goal"`) and its sequence
 * for the flight record. The two travel together: the record wants to know which message,
 * the watchdog wants to know which controller.
 *
 * And it emphatically does not parameterise authority. `GuidedEnvelope`, the abort ladder,
 * `LinkLossPolicy` and the engagement-confirmation machine take no origin and must not: an origin
 * changes *whose liveness is being asked about*, never how much the aircraft may do
 * (`docs/zenoh-dimos-transport.md` §3.4, prohibition 3). That prohibition is what "nothing
 * weakens for [PHONE]" rests on: the RC-feed gate, rule 1 on both stick channels, and every
 * DJI-side abort rung apply to a phone engagement exactly as to a MAVLink one.
 */
enum class ControlOrigin {
    /**
     * QGroundControl over UDP — every command and every stick frame that arrives on the wire.
     *
     * Liveness is **recency of inbound traffic**: the engine stamps `gcsSeenAtMs[MAVLINK]` on
     * every inbound payload, and silence past `GuidedEnvelope.LINK_LOST_MS` runs the armed
     * `LinkLossPolicy`. This is the Q4 heartbeat watchdog, byte-for-byte as it has always been.
     */
    MAVLINK,

    /**
     * The operator's own screen, in hand, running this very process — the phone Take off button
     * and the tag-descent arm control (`MainActivity` → `Bridge` → the engine's phone doors).
     *
     * ## Liveness is identity, not traffic
     *
     * The phone app **is** the process executing `GuidedStickEngine`. There is no link between
     * the commander and the engine whose loss could strand an engagement: if this process dies,
     * the 10 Hz virtual-stick stream stops with it and DJI's own virtual-stick timeout takes the
     * aircraft — the failsafe backstop that already covers a crashed bridge, phone origin or
     * not. A heartbeat here would be the process attesting its own liveness to itself: the
     * watcher and the watched are the same program, so the answer is "alive" whenever the
     * question can be asked at all. `controllerSeenAtLocked` therefore answers `now` for a
     * PHONE-origin engagement, and no traffic is ever stamped for this origin.
     *
     * ## The measurement that forced this value: landing08
     *
     * `datasets/landing08/20260729-112216.001.jsonl`, 2026-07-29 — the first phone-only flight
     * (no QGC, zero `mav_in` lines on the record). Every phone engagement was labelled MAVLINK
     * because no other label existed: the Take off button's climb engaged at t=32.33 and the Q4
     * watchdog released it at t=33.93 (`guided_released link-lost` — no GCS heartbeat had ever
     * been seen), so the aircraft hopped to 1.2 m and never climbed; every autoland arm was then
     * refused `tag_descent_denied LINK_DOWN` (t=51.4, 60.2, 63.4, 65.2, 71.5, 76.9) and Ivan
     * landed manually. Every prior flight had QGC connected, masking the mislabel.
     *
     * ## What a phone flight still depends on, and what still cancels it
     *
     * Deliberately unchanged, each pinned by the 2026-07-29 mutation table:
     *
     *  - **the physical RC feed** (`NO_RC_FEED`) — abort gesture 1 is the RC sticks, and a phone
     *    flight is *more* dependent on that gesture, not less, since there is no GCS to type at;
     *  - **rule 1 on both stick channels** — an RC grab or a deliberate GCS deflection (a QGC
     *    that connects mid-flight and deflects) cancels a PHONE-origin engagement completely;
     *  - **every DJI-side abort rung** — authority loss, mode seizure, interlock, failsafe.
     */
    PHONE,
}
