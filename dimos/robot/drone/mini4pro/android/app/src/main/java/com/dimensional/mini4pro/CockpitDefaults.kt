package com.dimensional.mini4pro

/**
 * **What an untouched install comes up as — the single owner of every cockpit default.**
 *
 * Before this object the answer was a literal at each read site: `prefs.getBoolean(PREF_VIDEO,
 * false)` appears in the settings dialog, in `zenohPlan()`, in `videoPlan()`, in the extras
 * absorbers. Four copies of one property, which is the "two-places-for-one-property" failure this
 * project has already been bitten by twice — and the dangerous half is that the copies are only
 * *read*, so a disagreement between them shows up as a switch that says one thing and a bridge that
 * does another, with nothing failing.
 *
 * ## Where the values come from
 *
 * Not from taste. They are the configuration the flight phone actually held on 2026-07-30, read out
 * of its `shared_prefs/mini4pro.xml` after thirteen recorded tag descents and three fully
 * autonomous missions:
 *
 * ```
 * gcs_host 10.55.1.50   gcs_port 14550   video_enabled true   video_host ""
 * zenoh_enabled true    zenoh_video true  zenoh_detections true   fastRecord true
 * ```
 *
 * Ivan, on getting the new airframe phone: *"by default when the app starts it should start the
 * bridge and enable interlock … it should enable zenoh video sending, 25hz telemetry by default"*.
 * So the defaults stop being "everything off" and become "the way this aircraft is flown". A phone
 * swap should not be an archaeology exercise, and every one of these was going to be switched on by
 * hand within the first minute anyway.
 *
 * ## What that costs, stated plainly
 *
 * Each of these switches had an off-by-default argument written on it, and those arguments were
 * about *cost*, not safety — they are answered by measurement, not overruled:
 *
 * - **Uplink.** GCS video is ~5.9 Mbit/s and the Zenoh copy is the same bytes again, so a default
 *   install now spends roughly twice the uplink of one before it. Measured, chosen, and the reason
 *   the two switches stay separate: turning the bus off does not silently take the GCS feed with it.
 * - **Thermal.** `docs/zenoh-dimos-transport.md` §6.4 still lists Zenoh's thermal cost as
 *   *unmeasured* on an airframe whose characteristic failure is a battery overheat. Defaulting it
 *   on does not measure it. What it does do is guarantee the number gets collected, because from
 *   here every flight carries the load — and `warn/WarningBus` now surfaces DJI's overheat warning
 *   to all four surfaces, which is the instrument that was missing on 2026-07-26.
 * - **Coarse pose.** `zenoh_detections` publishes a pose built on a fitted focal length and an
 *   assumed principal point. Every message still carries `metric=false` in its `id`, so the caveat
 *   travels with the number and cannot be separated from it by a default.
 *
 * ## What is deliberately NOT here
 *
 * The arm-time decisions. Full autoland's default lives in `activity_main.xml` (pinned by
 * `CockpitDefaultsTest`) because it is a switch position, and the descent's own gates —
 * `TagArming`, the commit conjuncts, `ReplayAdmission` — are not defaults at all and are reachable
 * from nothing here. This object decides what a session *starts as*, never what the aircraft is
 * *allowed to do*.
 */
object CockpitDefaults {

    /**
     * The relay on hyper1, not the laptop. This is the value that makes `LaunchPolicy` start the
     * bridge on a bare launch: it consults the *saved* host, and treats empty as "nowhere to send
     * telemetry, so do not start". A fresh install therefore came up silent until somebody opened
     * the dialog — which on a new airframe phone is exactly the wrong first experience, because the
     * phone's only USB port belongs to the RC and plugging that in is what launches us.
     *
     * The 2026-07-27 incident this interacts with is **not** reopened: `--ez autostart false` still
     * outranks a saved host, which is the whole content of that fix. What changes is that a fresh
     * install now behaves like a configured one, rather than like a refusal nobody asked for.
     */
    const val GCS_HOST = "10.55.1.50"

    /** Camera video to the GCS. Blank host means "same address as telemetry" — see `VideoRequest`. */
    const val GCS_VIDEO = true

    /** The second transport. `ZenohSettings.ROUTER_HOST` supplies the router; blank means default. */
    const val ZENOH = true

    /** …and the camera feed on it. The uplink-doubling switch, on deliberately (see class KDoc). */
    const val ZENOH_VIDEO = true

    /** …and AprilTag detections, which still announce `metric=false` in every `id`. */
    const val ZENOH_DETECTIONS = true

    /**
     * Record `dji_state` at 25 Hz rather than 5 Hz.
     *
     * The rate at which a ~200 ms command-to-motion delay is measurable at all, which is the
     * question every landing analysis this month has actually been asking. Measured cost:
     * **29.6 kB/s**, against the video's 739 — a rounding error on a session that is already
     * streaming two camera feeds.
     */
    const val FAST_RECORD = true

    /**
     * **On-phone video recording, started with the session.**
     *
     * Not the same thing as [GCS_VIDEO] or [ZENOH_VIDEO], which send frames to somebody else; this
     * one writes `.vNNN.h264` sidecars beside the JSONL, and it is what makes a flight
     * reconstructable afterwards instead of merely summarised. Every landing analysis this month
     * leaned on those files.
     *
     * What it spends is **disk, about 625 kB/s** — roughly 37 MB a minute, against a 2 GiB per
     * session budget the sidecar enforces itself and then says so on the switch. That budget is why
     * defaulting this on is affordable: the failure mode is bounded and announced, not a phone that
     * silently fills up.
     *
     * `Recorder.videoEnabled` remains the single owner of "is it recording right now", and the
     * switch is assigned *from* it on the drawing tick. This constant only seeds a session; flipping
     * the switch mid-flight still takes effect on the next frame, and nothing here is persisted —
     * the next launch starts from this value again, deliberately, so a session that was turned off
     * for one reason cannot silently swallow the next flight.
     */
    const val RECORD_VIDEO = true

    /**
     * **The command interlock, armed at startup.**
     *
     * This is the only default here that is about authority rather than bandwidth, so it is the one
     * to read carefully. It means QGroundControl's Return and Land buttons reach DJI from the
     * moment the app is up, with nobody having tapped anything.
     *
     * Two properties survive the change, and they are the ones that were load-bearing:
     *
     * 1. **No inbound MAVLink message can turn the interlock on.** `CommandInterlock.enable()` is
     *    still called only from `MainActivity` — from `onCreate`, from the switch, and from
     *    `followSimulatorInterlock` — never from a network or DJI callback. Startup is a colder
     *    path than a button, not a warmer one: it runs before any socket exists.
     * 2. **A replay can still never arm a real aircraft.** `ReplayAdmission.mayArm` is checked at
     *    startup exactly as it is on the switch, and a loaded replay refuses by name.
     *
     * What is genuinely given up is the confirmation dialog — the one that said "there is no
     * Emergency Stop: DJI has no in-flight motor cut". That sentence is still true and is still on
     * the record; what stopped being true is that a dialog once per app start was informing
     * anybody. Ivan flies this aircraft daily and has answered it every session since the interlock
     * existed: *"We don't need validation confirmation"*. A prompt whose answer is never anything
     * but "Allow" is not a safety device, it is a keystroke — and it was being answered *before*
     * takeoff, about buttons pressed much later.
     *
     * The stick gesture on the physical RC remains the only motor cut, and remains outside all of
     * this software.
     */
    const val COMMAND_INTERLOCK = true
}
