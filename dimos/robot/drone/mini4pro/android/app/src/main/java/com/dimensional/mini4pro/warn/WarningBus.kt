package com.dimensional.mini4pro.warn

/**
 * **The single owner of "a DJI warning reaches the people and the machines that need it."**
 *
 * Every source hands its picture here and nowhere else, and every surface is fed from here and
 * nowhere else. That is the whole design, and it is a rule rather than a convenience: a source that
 * builds its own `STATUSTEXT`, or writes its own record line, is the two-places-for-one-property
 * failure this project has caught twice by mutation campaign, and it is how the four surfaces come
 * to disagree about what the operator was told. `CLAUDE.md` states it; this class is where it is
 * enforced.
 *
 * ## The four surfaces, and why each one exists
 *
 * | surface | why | what it costs to lose |
 * |---|---|---|
 * | **QGroundControl** (`STATUSTEXT`) | where the pilot's eyes are on a plan | landing17: four `LEVEL_2` wind warnings in a 14.2 m/s wind, none announced |
 * | **the phone screen** | Ivan flies looking at the phone as often as at QGC | the same, on the surface he is actually holding |
 * | **the flight record** (`dji_warn`) | the post-mortem, and the only lossless one | 2026-07-26: an hour of theorising about force-lands with the answer in DJI Fly |
 * | **the Zenoh bus** (`warnings`) | DiMOS, and anything autonomous downstream | a robot that cannot know the aircraft is in trouble |
 *
 * The record and the bus get **everything**, including events QGC is never told about and events
 * the rate bound suppressed. The two operator surfaces get what [WarnEvent.announce] allows. That
 * asymmetry is deliberate and is the reason [WarnEvent] carries `announce` and `rateLimited`
 * separately: "QGC is never told about this level" and "QGC would have been told but the bound
 * bit" are different conversations, and a post-mortem must be able to have both.
 *
 * ## Edge-triggered, and where the edge is decided
 *
 * In [WarningMonitor], once, for every source. A source may deliver its picture as often as it
 * likes — DJI's health manager re-delivers the whole list whenever anything in it moves, and a key
 * listener fires on every value — and the monitor's diff turns that into events only on genuine
 * transitions. **Nothing downstream of the monitor may announce per sample**, and nothing upstream
 * of it may decide what is news.
 *
 * ## Containment
 *
 * Every sink is independently contained and the order is fixed: **the flight record first**,
 * because it is the sink whose whole purpose is surviving the session and it must not be lost to a
 * failure in any of the others. A throw here would land on DJI's callback thread — the main thread
 * — where this project has already killed the process once by letting an exception escape a DJI
 * callback (`Bridge.sendOffMain`). Nothing about reporting a warning may become a bigger problem
 * than the warning.
 *
 * ## Threading
 *
 * DJI delivers on the Android main thread; sources are also driven from `Bridge`'s telemetry
 * thread. Every path into [WarningMonitor] is serialised on [gate], because the monitor holds
 * mutable state and is documented as not thread-safe. `qgc` must therefore be `Bridge.sendOffMain`
 * and not a raw socket write — a UDP send on the main thread kills the process.
 */
class WarningBus(
    /**
     * Emit one `STATUSTEXT`. Takes the already-clamped text and a `MAV_SEVERITY` integer; building
     * the MAVLink object is the caller's job, so this package stays free of the MAVLink library
     * exactly as it stays free of DJI.
     */
    private val qgc: (text: String, mavSeverity: Int) -> Unit,
    /** Put the sentence in front of the pilot on the phone. */
    private val screen: (WarnEvent) -> Unit,
    /** One `dji_warn` entry in the flight record. Gets every event, announced or not. */
    private val record: (WarnEvent) -> Unit,
    /**
     * One `warnings` message on the Zenoh bus. Gets every event, announced or not, for the record's
     * reason: a subscriber is a machine, and a machine is not protected by being told less.
     */
    private val bus: (WarnEvent) -> Unit,
    /** One logcat line. [WarnEvent.recordSeverity] decides `Log.i` vs `Log.w` at the call site. */
    private val log: (WarnEvent) -> Unit,
    /** Diagnostics about the plumbing itself, not about any warning. */
    private val note: (String) -> Unit = {},
    private val nowMs: () -> Long,
    private val monitor: WarningMonitor = WarningMonitor(),
) {

    private val gate = Any()

    /** Everything every source is currently reporting — for the phone screen and for tests. */
    fun snapshot(): List<Warning> = synchronized(gate) { monitor.snapshot() }

    /** The last sentence any surface was given, or null before the first one. For the screen. */
    @Volatile
    var lastText: String? = null
        private set

    /**
     * **One source's complete current picture.** The only entry point.
     *
     * @param source who is speaking. Only this source's warnings are diffed; every other source's
     *   picture survives untouched, because this delivery says nothing about it.
     * @param warnings everything [source] is currently reporting — empty is a legitimate picture
     *   and means "this source has nothing to report", which clears whatever it reported before.
     */
    fun deliver(source: WarnSource, warnings: List<Warning>) {
        val events = try {
            synchronized(gate) { monitor.onDelivery(source, warnings, nowMs()) }
        } catch (t: Throwable) {
            note("warning diff failed for ${source.label}: $t")
            return
        }
        // The overwhelmingly common case: DJI repeated itself and there is nothing to say.
        if (events.isEmpty()) return
        for (event in events) fanOut(event)
    }

    /**
     * Forget everything, so the next delivery re-announces the current picture as appearances.
     *
     * Called when a subscription is dropped. See [WarningMonitor.reset] for why re-announcing is
     * the honest choice rather than staying quiet.
     */
    fun reset() {
        synchronized(gate) { monitor.reset() }
        lastText = null
    }

    private fun fanOut(event: WarnEvent) {
        try {
            record(event)
        } catch (t: Throwable) {
            note("flight recorder warning entry failed: $t")
        }
        try {
            bus(event)
        } catch (t: Throwable) {
            note("zenoh warning publish failed: $t")
        }
        try {
            log(event)
        } catch (t: Throwable) {
            // Nothing sensible to do about a logging failure but carry on.
        }
        if (!event.announce) return
        val severity = event.mavSeverity ?: return
        lastText = event.text
        try {
            qgc(event.text, severity)
        } catch (t: Throwable) {
            note("warning STATUSTEXT failed: $t")
        }
        try {
            screen(event)
        } catch (t: Throwable) {
            note("warning screen line failed: $t")
        }
    }
}
