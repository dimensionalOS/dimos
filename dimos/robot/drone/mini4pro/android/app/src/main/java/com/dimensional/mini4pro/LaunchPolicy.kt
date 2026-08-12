package com.dimensional.mini4pro

/**
 * **Whether launching the app should also start the bridge.**
 *
 * Lifted out of [MainActivity.onCreate] so it can be tested: the Activity needs a framework to
 * run in, this decision does not, and it is the decision that opened a flight session nobody
 * asked for.
 *
 * ## The incident, 2026-07-27 19:37
 *
 * An agent ran `am start -n com.dimensional.mini4pro/.MainActivity` to unfreeze the app during a
 * profiling run. The app came up, read a saved host from preferences, **started a real bridge
 * session**, and wrote `20260727-193709.001.jsonl` before the next step killed it 20 s later. No
 * aircraft was connected, so nothing flew — but the same command during a flight would have
 * attached a second telemetry link to a live aircraft, and the log was briefly read as the
 * operator's own session.
 *
 * The agent had checked for [MainActivity.EXTRA_AUTOSTART] and missed that the branch above it
 * does not consult it.
 *
 * ## Why the answer is not "stop starting the bridge on launch"
 *
 * Because that branch is load-bearing and the comment on it is right: **the phone's only USB port
 * belongs to the RC**, and adb over WiFi has proved unreliable on this device. Plugging the RC in
 * launches us through `UsbAttachActivity`, and if telemetry needed an adb command to begin, the
 * whole thing would be unusable in the field. A bare launch *must* come up talking.
 *
 * So the rule is not "never start" but **"a caller that wants only the Activity must be able to
 * say so"** — and before this change, it could not. `--ez autostart false` now means it, and the
 * field path is untouched because plugging in a cable passes no extras at all.
 *
 * ## The three-valued extra
 *
 * [autostart] is `null` when the caller said nothing, which is *not* the same as `false`:
 *
 * | intent host | saved host | autostart | start? | who does this |
 * |---|---|---|---|---|
 * | absent | set | absent | **yes** | RC plugged in, or the launcher icon |
 * | absent | set | `false` | no | a tool that wants the Activity and nothing else |
 * | absent | empty | anything | no | nowhere to send telemetry |
 * | present | — | `true` | **yes** | `tools/session`, deliberately |
 * | present | — | absent | no | a host was named but no start was asked for |
 */
object LaunchPolicy {

    /**
     * @param intentHost the GCS host named on the intent, already trimmed, or null.
     * @param savedHost the GCS host in preferences, already trimmed; empty when unset.
     * @param autostart the `autostart` extra: true, false, or **null when absent**.
     */
    fun shouldStartBridge(intentHost: String?, savedHost: String, autostart: Boolean?): Boolean {
        // An explicit refusal outranks everything, including a saved host. This is the whole
        // point of the change: one flag a tool can pass to get an Activity and nothing else.
        if (autostart == false) return false
        if (intentHost != null) {
            // A named host is not on its own a request to start — a validator may be setting up
            // a port before it is ready to receive. Only an explicit `true` starts.
            return autostart == true
        }
        return savedHost.isNotEmpty()
    }
}
