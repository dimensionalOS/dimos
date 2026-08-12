package com.dimensional.mini4pro.command

import java.util.concurrent.atomic.AtomicBoolean

/**
 * The switch between "this bridge translates telemetry" and "this bridge can move an aircraft".
 *
 * `docs/decisions/2026-07-25-m2-command-safety.md` §Q2. Three properties, and each one is a
 * property of *this file's shape* rather than of a caller remembering something:
 *
 *  1. **Off at every process start.** There is no constructor parameter for the initial state,
 *     so there is no way to construct an instance that begins enabled. A test that wants the
 *     enabled state must call [enable], which is the same thing the operator has to do.
 *  2. **Not persisted.** This class touches no `SharedPreferences`, no file, no `Context` — it
 *     holds one [AtomicBoolean] and nothing else, so "not persisted" is not a policy that could
 *     be quietly reversed by adding a save call somewhere, it is the absence of any storage to
 *     save into. The failure the decision doc expects is not a wrong command, it is a *forgotten
 *     enable*: a phone that boots into "commands live" because of a setting from last week.
 *  3. **No inbound MAVLink message can enable it.** [enable] is never wired to
 *     `HandshakeResponder`; the only caller is the UI. We do not authenticate the link at all,
 *     so a ground station on a hostile or noisy network must not be able to arm the command
 *     path. `CommandDispatcherTest` replays the whole inbound surface — every PX4 button
 *     command, every `SET_MODE` QGC can send, and a `PARAM_SET` for every parameter we publish —
 *     and asserts this stays false.
 *
 * While disabled, every actuating request is answered exactly as it was before M2 existed:
 * `MAV_RESULT_UNSUPPORTED` for commands, and `HandshakeResponder`'s own
 * [com.dimensional.mini4pro.handshake.HandshakeResponder.MODE_REFUSAL_TEXT] warning for
 * `SET_MODE`. That is deliberate — [CommandDispatcher] declines the request and lets the
 * existing, already-tested refusal path run, rather than growing a second refusal branch that
 * would drift away from the first one.
 *
 * Thread safety: read on the `mavlink-rx` thread, written from the Android main thread.
 */
class CommandInterlock(
    /** Optional trace hook, so state changes land in the log that the flight recorder mirrors. */
    private val log: (String) -> Unit = {},
    /**
     * Wall clock, injected so [enabledSinceMs] is testable without sleeping. Used for nothing
     * but that timestamp.
     */
    private val nowMs: () -> Long = { System.currentTimeMillis() },
) {

    private val state = AtomicBoolean(false)

    @Volatile
    private var since: Long? = null

    /** True only if an operator has turned commands on during *this* run of the process. */
    val enabled: Boolean get() = state.get()

    /**
     * When the interlock was last enabled, or null if it is off. For the UI and the flight
     * recorder — an interlock that has been on for an hour is worth showing differently from one
     * the operator just flipped. Nothing in the command path reads it.
     */
    val enabledSinceMs: Long? get() = if (state.get()) since else null

    /**
     * Turn commands on. Called only from the UI, never from a MAVLink handler.
     *
     * Returns true if this call changed the state, so a caller can log the transition without
     * racing another one.
     */
    fun enable(): Boolean {
        val changed = state.compareAndSet(false, true)
        if (changed) {
            since = nowMs()
            log("command interlock ENABLED — QGC's return/land/emergency stop can now reach DJI")
        }
        return changed
    }

    /**
     * Turn commands off. Always available, never fails, and safe to call from anywhere — this is
     * the direction that can only make things safer, which is why it has no guard on it.
     */
    fun disable(): Boolean {
        val changed = state.compareAndSet(true, false)
        if (changed) {
            since = null
            log("command interlock DISABLED — bridge is telemetry-only again")
        }
        return changed
    }

    /** Convenience for a UI toggle. */
    fun set(on: Boolean): Boolean = if (on) enable() else disable()
}
