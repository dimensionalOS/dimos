package com.dimensional.mini4pro.guided

/**
 * The mission executor's run state. Five values, and the set is **closed on purpose**.
 *
 * `docs/m4-mission-execution.md` §4, with M4-5's amendment folded in: [FINISHED] means *"arrived at
 * the last item and holding"*, **not** *"landed"*. Ivan's answer was *"just hover for now, I'll land
 * manually"*, so a mission ends with the aircraft in the air, holding station, waiting for a human
 * on the sticks — and holding station means holding **authority**, which the idle window, the link
 * watchdog and the abort ladder are what eventually end. That is stated here rather than only in the
 * decision doc, because a reader of this enum will otherwise assume the aircraft is on the ground.
 *
 * **There is no `ABORTED` state, and that is a decision.** `MISSION_STATE` has no such value, and
 * inventing an internal one whose only wire representation is `PAUSED` would create exactly the
 * divergence between what we know and what we say that this project spends its effort avoiding.
 * What would have been "aborted" is [PAUSED] **plus a [ResumeBlock]** — a separate, nullable field
 * that is not a state and does not appear on the wire. [PAUSED] is true in every one of those cases:
 * the run stopped, the cursor is preserved, and resuming takes an act.
 */
enum class MissionLifecycle {
    /** The store is empty. */
    NO_MISSION,

    /** A stored plan that passed static admission; cursor at 0; never started, or reset. */
    LOADED,

    /** Our executor is engaged and flying the item at the cursor. The only state that holds authority. */
    RUNNING,

    /** A run stopped part-way; cursor and plan id preserved; resuming needs an explicit Start. */
    PAUSED,

    /**
     * The last item completed — **arrived and holding, in the air.** Not landed; see the class KDoc
     * and `docs/decisions/2026-07-27-m4-eighteen-answers.md` M4-5.
     */
    FINISHED,
}

/**
 * Everything that can move the lifecycle. One value per *cause*, never per mechanism, so the table
 * in [MissionTransitions] reads as policy rather than as plumbing.
 */
enum class MissionEvent {
    /** A plan was accepted into the store. */
    PLAN_COMMITTED,

    /** `MISSION_CLEAR_ALL`, accepted. */
    PLAN_CLEARED,

    /** An operator Start passed the full launch check and a route was handed to the engine. */
    START_ACCEPTED,

    /** An operator Start failed a launch check. **Nothing flew**, and the state must not move. */
    START_REFUSED,

    /** The cursor completed the final item. */
    LAST_ITEM_COMPLETE,

    /** Any row of the abort table whose consequence is a pause — blocked or not. */
    RUN_PAUSED,

    /**
     * The store's plan id changed under a **paused** cursor, so the cursor indexes a plan that no
     * longer exists. Under a *running* mission this is a no-op by M4-12: the run continues on the
     * snapshot it began with and the new plan takes effect at the next Start.
     */
    PLAN_GENERATION_CHANGED,

    /** `Bridge.stop()`. Nothing is persisted; the next process start is [MissionLifecycle.NO_MISSION]. */
    BRIDGE_STOPPED,
}

/**
 * The `(state × event) → state` table, and the only place a lifecycle transition is decided.
 *
 * The `when`s below are **exhaustive over both enums with no `else`**, so adding a state or an event
 * fails compilation until every pair has an answer — the same discipline `Px4ModeTest` uses to
 * account for all 79 `FCFlightMode` constants, moved from a test into the compiler. `MissionLifecycleTest`
 * additionally enumerates every pair at runtime against a hand-written expectation, so a *wrong*
 * answer is caught as well as a missing one.
 *
 * A null return means **illegal**: the caller ignores the event and logs it. Illegal is not the same
 * as "no change" — `LOADED + START_REFUSED` is a legal no-change, and `NO_MISSION + START_ACCEPTED`
 * is a bug in the caller.
 */
object MissionTransitions {

    /**
     * **A report about a run, arriving when there is no run.** Not a caller bug — the ordinary
     * shape of a flight.
     *
     * [next] returns `null` for two different things and the caller cannot otherwise tell them
     * apart: *"that cannot happen from here"* (a `START_ACCEPTED` while already `RUNNING` — someone
     * skipped the launch check) and *"there is nothing left for that to be about"*. This
     * distinguishes the second.
     *
     * The case that made it worth having is the commonest ending there is. A mission reaches its
     * last item and the lifecycle goes `FINISHED`; the aircraft holds station, still engaged; the
     * operator takes the sticks. The engine reports `RUN_PAUSED`, correctly — its run *did* end —
     * and the state machine has nothing to do with it, also correctly. Until 2026-07-27 that was
     * logged as **"illegal mission transition ignored: FINISHED + RUN_PAUSED"**, on a flight where
     * nothing whatever had gone wrong. It happened on the first real mission this project ever
     * flew, at t=82.3 (`docs/measurements/2026-07-27-first-real-mission.md`).
     *
     * A log that cries wolf on the normal case is worse than no log, because the next reader
     * learns to skip the line — and the line is the only warning that a genuine sequencing bug
     * would ever give.
     *
     * Deliberately derived from the shape of the event rather than listed pair by pair: the two
     * events below are the only ones that *report on a run*, so any state that is not `RUNNING`
     * has nothing for them to be about, and a new lifecycle state cannot forget to be included.
     */
    fun isMoot(state: MissionLifecycle, event: MissionEvent): Boolean =
        (event == MissionEvent.RUN_PAUSED || event == MissionEvent.LAST_ITEM_COMPLETE) &&
            state != MissionLifecycle.RUNNING


    fun next(state: MissionLifecycle, event: MissionEvent): MissionLifecycle? = when (state) {
        MissionLifecycle.NO_MISSION -> when (event) {
            MissionEvent.PLAN_COMMITTED -> MissionLifecycle.LOADED
            MissionEvent.PLAN_CLEARED -> MissionLifecycle.NO_MISSION
            MissionEvent.BRIDGE_STOPPED -> MissionLifecycle.NO_MISSION
            // A plan id can change with nothing committed (a clear bumps it); nothing to drop.
            MissionEvent.PLAN_GENERATION_CHANGED -> MissionLifecycle.NO_MISSION
            // Nothing to start, nothing running, nothing to finish.
            MissionEvent.START_ACCEPTED -> null
            MissionEvent.START_REFUSED -> MissionLifecycle.NO_MISSION
            MissionEvent.LAST_ITEM_COMPLETE -> null
            MissionEvent.RUN_PAUSED -> null
        }

        MissionLifecycle.LOADED -> when (event) {
            MissionEvent.PLAN_COMMITTED -> MissionLifecycle.LOADED
            MissionEvent.PLAN_CLEARED -> MissionLifecycle.NO_MISSION
            MissionEvent.START_ACCEPTED -> MissionLifecycle.RUNNING
            // Nothing flew, so nothing moved. The refusal is a sentence, not a state.
            MissionEvent.START_REFUSED -> MissionLifecycle.LOADED
            MissionEvent.PLAN_GENERATION_CHANGED -> MissionLifecycle.LOADED
            MissionEvent.BRIDGE_STOPPED -> MissionLifecycle.NO_MISSION
            MissionEvent.LAST_ITEM_COMPLETE -> null
            MissionEvent.RUN_PAUSED -> null
        }

        MissionLifecycle.RUNNING -> when (event) {
            // M4-12: the upload commits to the store and **this run carries on with the snapshot it
            // began with**. Swapping the plan under a moving aircraft would leave the cursor
            // indexing a list nobody flew a metre of.
            MissionEvent.PLAN_COMMITTED -> MissionLifecycle.RUNNING
            MissionEvent.PLAN_GENERATION_CHANGED -> MissionLifecycle.RUNNING
            // A clear is refused by the transport while running; if one ever lands, the honest
            // answer is that we hold no plan any more, and the engine is stopped with it.
            MissionEvent.PLAN_CLEARED -> MissionLifecycle.NO_MISSION
            MissionEvent.LAST_ITEM_COMPLETE -> MissionLifecycle.FINISHED
            MissionEvent.RUN_PAUSED -> MissionLifecycle.PAUSED
            MissionEvent.BRIDGE_STOPPED -> MissionLifecycle.NO_MISSION
            // Already running: a Start while running is not a resume and must not re-run the
            // launch check on a moving aircraft.
            MissionEvent.START_ACCEPTED -> null
            MissionEvent.START_REFUSED -> MissionLifecycle.RUNNING
        }

        MissionLifecycle.PAUSED -> when (event) {
            // **The only cause of PAUSED → RUNNING**, and it is Ivan's binding M4-11 answer:
            // *"yeah never automatic"*. No timer, no watchdog, no helpful retry.
            MissionEvent.START_ACCEPTED -> MissionLifecycle.RUNNING
            MissionEvent.START_REFUSED -> MissionLifecycle.PAUSED
            // The paused cursor indexed a plan that no longer exists.
            MissionEvent.PLAN_GENERATION_CHANGED -> MissionLifecycle.LOADED
            MissionEvent.PLAN_COMMITTED -> MissionLifecycle.LOADED
            MissionEvent.PLAN_CLEARED -> MissionLifecycle.NO_MISSION
            MissionEvent.BRIDGE_STOPPED -> MissionLifecycle.NO_MISSION
            // Nothing is flying, so nothing can pause or complete.
            MissionEvent.RUN_PAUSED -> null
            MissionEvent.LAST_ITEM_COMPLETE -> null
        }

        MissionLifecycle.FINISHED -> when (event) {
            // From cursor 0, with the launch check re-run in full.
            MissionEvent.START_ACCEPTED -> MissionLifecycle.RUNNING
            MissionEvent.START_REFUSED -> MissionLifecycle.FINISHED
            MissionEvent.PLAN_COMMITTED -> MissionLifecycle.LOADED
            MissionEvent.PLAN_GENERATION_CHANGED -> MissionLifecycle.LOADED
            MissionEvent.PLAN_CLEARED -> MissionLifecycle.NO_MISSION
            MissionEvent.BRIDGE_STOPPED -> MissionLifecycle.NO_MISSION
            // The aircraft is holding, not flying a plan. A pause of the *hold* is not a pause of
            // the mission, and there is no item left to complete.
            MissionEvent.RUN_PAUSED -> null
            MissionEvent.LAST_ITEM_COMPLETE -> null
        }
    }
}

/**
 * How long a [ResumeBlock] stands. Three answers, because the nineteen rows genuinely need three.
 */
enum class BlockPersistence {
    /**
     * **Spent by telling the operator.** The next Start refuses, naming the reason, and the block
     * clears; a second, deliberate Start may then proceed. §6.1's *"spent as soon as its condition
     * clears **or the operator is told about it**"*.
     *
     * This is right for the rows whose message is *look at the aircraft* — a leg timeout usually
     * means wind or an obstacle brake, and the useful safety behaviour is one refusal that makes
     * the operator stop and think, not a permanent lockout they will work around by restarting.
     */
    SPENT_ON_REFUSAL,

    /** Stands while an **observed** condition holds, and clears with it. The interlock; a seized mode. */
    WHILE_CONDITION,

    /** Stands for the session. A battery does not un-deplete. */
    SESSION,
}

/**
 * Why a paused mission may not be resumed *yet*.
 *
 * Not a state: [MissionLifecycle.PAUSED] is true in every one of these cases. This is the field that
 * makes the subsequent refusal **truthful rather than mysterious** — an operator pressing Start and
 * getting nothing is the failure this exists to prevent.
 *
 * @param reason the word the `STATUSTEXT` carries. Short, because the field is 50 bytes and the
 *   framing has to fit around it.
 */
enum class ResumeBlock(val reason: String, val persistence: BlockPersistence) {
    /** Row 5: the operator withdrew consent for commanded flight. */
    INTERLOCK_OFF("interlock off", BlockPersistence.WHILE_CONDITION),

    /** Row 6: DJI is flying the aircraft on its own authority, for its own reason. */
    MODE_SEIZED("DJI has the aircraft", BlockPersistence.WHILE_CONDITION),

    /**
     * Row 2: the RC's Return button. The aircraft is going home; the plan is not cancelled, but
     * resuming from home is a new flight and the rejoin bound is the honest arbiter.
     */
    GOING_HOME("going home", BlockPersistence.SPENT_ON_REFUSAL),

    /** Row 7: a geofence takeback or a DJI failsafe — the world changed, not the pilot's mind. */
    WORLD_CHANGED("DJI took control", BlockPersistence.SPENT_ON_REFUSAL),

    /** Row 15. */
    LEG_TIMEOUT("leg timeout", BlockPersistence.SPENT_ON_REFUSAL),

    /** Row 16. */
    MISSION_TIMEOUT("mission timeout", BlockPersistence.SPENT_ON_REFUSAL),

    /**
     * The tag landing declined — a failed [PrecisionLand.gate], a camera that never reached nadir, or
     * a descent arm the engine's own door refused. The aircraft is **holding at the height the
     * sequence got to**, and the specific reason is on the record and on the operator's screen; this
     * only decides what the next Start does about it.
     *
     * [BlockPersistence.SPENT_ON_REFUSAL], on the [LEG_TIMEOUT] argument: the message is *look at the
     * aircraft, or at the plan* — the pad is 40 m away, or the tag is not in view, or the camera is
     * pointing somewhere else — and one refusal that makes the operator stop and think is the useful
     * behaviour, not a lockout they work around by restarting. A second, deliberate Start then rejoins
     * the land item and re-runs every gate against the world as it then is.
     */
    LAND_TAG_REFUSED("tag landing refused", BlockPersistence.SPENT_ON_REFUSAL),

    /** Row 12: DJI's first low-battery warning. Get out of the way early rather than be taken out late. */
    LOW_BATTERY("low battery", BlockPersistence.SESSION),

    /** Row 13: the serious warning or a battery failsafe. DJI is flying now. */
    BATTERY_DEPLETED("battery", BlockPersistence.SESSION),
}

/** What an abort gesture does to the *plan*, as distinct from what it does to the engagement. */
sealed interface MissionConsequence {

    /** Rows 10 and 11: the gesture degrades an axis and the mission is untouched. */
    object None : MissionConsequence

    /** The run stops, the cursor is kept, and a later explicit Start may continue. */
    object PauseResumable : MissionConsequence

    /** The same, plus a block that will refuse the next Start until it is spent or clears. */
    data class PauseBlocked(val block: ResumeBlock) : MissionConsequence

    /** The cursor is dropped. Only for gestures after which the plan is no longer the plan we admitted. */
    object Abandon : MissionConsequence
}

/**
 * One row of `docs/m4-mission-execution.md` §6.2 — *every* abort gesture, and what it does to a
 * running mission. **This table is the heart of the design**, and each value has its own named test.
 *
 * The M3 abort ladder is not modified by any of this: every gesture still ends the engagement
 * exactly as it does today, on the same wire, with the same sentence. What M4 adds is a second
 * question for each — *and what happens to the plan?*
 */
enum class MissionPauseCause {
    /** 1. RC stick grab. The pilot's hands mean *"I have it"*, not *"delete the plan"*. */
    RC_STICK_GRAB,

    /** 2. The RC's Return button — `RC_ONE_KEY_GO_HOME`, measured and instant. */
    RC_RETURN_HOME,

    /** 3. A deliberate GCS stick deflection. The operator chose the GCS-stick channel; the plan waits. */
    GCS_STICK_DEFLECTION,

    /**
     * 3b. An explicit GCS **destination** — a `DO_REPOSITION` goto, or an orbit.
     *
     * The same consequence as [GCS_STICK_DEFLECTION] and a different sentence, which is the whole
     * reason it exists. Until 2026-07-27 a goto and an orbit both paused the plan reporting
     * `GCS_STICK_DEFLECTION`, and an operator reading their own flight record was told a stick had
     * been deflected when none had been touched. The consequence was right; the account of it was
     * false, and a record that misdescribes why an aircraft stopped flying a plan is worse than one
     * that says nothing, because it will be believed.
     *
     * Kept distinct from a stick deflection rather than folded in, because the two differ in the
     * one way that matters when reading a log after the fact: a stick deflection is continuous and
     * the operator may not have meant a *destination* by it, whereas this is a discrete, validated,
     * acknowledged command with coordinates in it.
     */
    GCS_NEW_DESTINATION,

    /** 4. QGC link loss. A returning link is evidence of a link, not of an operator. */
    QGC_LINK_LOSS,

    /** 5. The interlock switch. */
    INTERLOCK_OFF,

    /** 6. DJI seizes the flight mode — the measured overheat `GO_HOME` shape. */
    DJI_MODE_SEIZURE,

    /** 7a. An RC-side authority change: `RC_SWITCH`, `RC_PAUSE_STOP`, `RC_NOT_P_MODE`, `RC_LOST`. */
    RC_AUTHORITY_CHANGE,

    /** 7b. `NEAR_BOUNDARY` or a battery failsafe reason — the world changing rather than a pilot deciding. */
    WORLD_AUTHORITY_CHANGE,

    /** 8. Product gone / RC stick feed null — the measured FC-blackout shape, transient by nature. */
    PRODUCT_GONE,

    /** 9. The position feed stale or lost. Nothing is dead-reckoned, and the cursor never advances blind. */
    POSITION_LOST,

    /** 10. Velocity feed stale. Withholds *arrival* only; a pass-through is geometric. */
    VELOCITY_STALE,

    /** 11. Altitude feed stale. The vertical axis goes inert; lateral continues. */
    ALTITUDE_STALE,

    /**
     * 12. DJI's first low-battery warning.
     *
     * **Not wired, and that must not be quiet.** `AircraftState` carries no `KeyIsLowBatteryWarning`
     * (§6.5) and this layer is forbidden from inferring one from `batteryPercent` — `chargeState`'s
     * thresholds are documented placeholders and must not become a flight-control input. Until the
     * key is wired and *measured*, this row is unreachable and rows 12–13 **degrade to row 7**: we
     * still abort on DJI's battery failsafe authority reasons, which are handled. What is lost is
     * the early warning, and the value exists here so that the loss is visible rather than absent.
     */
    LOW_BATTERY_WARNING,

    /** 13. The serious warning, or `BATTERY_LOW_GO_HOME` / `BATTERY_SUPER_LOW_LANDING`. See row 12's note. */
    SERIOUS_LOW_BATTERY,

    /** 14. `sendVirtualStickAdvancedParam` threw. A bridge that cannot deliver setpoints holds no authority. */
    SEND_FAILED,

    /** 15. A leg did not complete inside [MissionGuidance.legTimeoutMs]. */
    LEG_TIMEOUT,

    /** 16. The whole mission ran past [MissionGuidance.MISSION_MAX_S]. */
    MISSION_TIMEOUT,

    /** 17. `Bridge.stop()`. Nothing is persisted; a plan resuming into a session nobody armed is the failure. */
    BRIDGE_STOPPED,

    /** 18. The plan was re-uploaded or cleared while paused: the cursor indexed a plan that is gone. */
    PLAN_CHANGED,

    /** 19. QGC's Pause button — a withdrawal, not a new intent. */
    GCS_PAUSE,

    /**
     * Beyond the nineteen: **the tag landing declined**, at any of its three doors — a failed
     * [PrecisionLand.gate] at the item's begin, a camera that never reached believed nadir, or an arm
     * the descent's own gate refused.
     *
     * Not in §6.2 because §6.2 was written when no mission item could descend. It is the one row here
     * that is not an *abort* of anything: nothing seized the aircraft, no feed died, and the
     * engagement is intact — the run simply reached its last item and could not do the one thing that
     * item asked for. The consequence is a blocked pause with the aircraft holding, which is M4-5's
     * ending arrived at by a different road, plus a reason.
     */
    LAND_TAG_REFUSED,

    /**
     * Beyond the nineteen: Q1's idle window expiring on a hold.
     *
     * Not in §6.2 because §6.2 was written before [MissionLifecycle.FINISHED] meant *holding in the
     * air*. A mission that ends holding is holding **authority**, and the idle window is one of the
     * three things that eventually ends it — so it needs an answer, and the answer is the mildest
     * one: the aircraft simply stops being ours and the plan is left resumable.
     */
    IDLE_HOLD,
}

/**
 * §6.2's table, as code: engine gesture → row → mission consequence.
 *
 * Split into two functions on purpose. [causeOf] is the *plumbing* — which row of the table a
 * [GuidedStickEngine.DisengageReason] and its detail string belong to — and [consequence] is the
 * *policy*. Keeping them apart means the nineteen policy tests read as policy, and a change to how
 * the engine spells a reason cannot silently change what a gesture does to a plan.
 */
object MissionAbortPolicy {

    /** DJI's authority reason for the RC's Return button. Measured, instant. */
    const val REASON_RC_GO_HOME = "RC_ONE_KEY_GO_HOME"

    /** The detail the engine attaches to a send that threw. */
    const val REASON_SEND_FAILED = "SEND_FAILED"

    /** The detail the engine attaches when any subscribed RC stick key delivers null. */
    const val REASON_RC_STICK_NULL = "RC_STICK_NULL"

    /** The prefix the engine puts on a seized flight mode, e.g. `MODE_GO_HOME`. */
    const val PREFIX_MODE = "MODE_"

    /** The detail the mission tick attaches to a [GuidedStickEngine.DisengageReason.TIMEOUT] it caused. */
    const val DETAIL_LEG_TIMEOUT = "leg"

    /** The detail the mission tick attaches when the whole-mission cap expires. */
    const val DETAIL_MISSION_TIMEOUT = "mission"

    /**
     * DJI authority reasons that mean **the world changed** rather than **the pilot decided** —
     * a geofence takeback and both battery failsafes. Everything else DJI reports that is not
     * [REASON_RC_GO_HOME] is treated as an RC-side decision, which is the *resumable* reading and
     * therefore the one that must be justified: an operator flicking a mode switch has not told us
     * anything about the world, and blocking their resume would punish the correct instinct.
     */
    val WORLD_REASONS = setOf(
        "NEAR_BOUNDARY",
        "BATTERY_LOW_GO_HOME",
        "BATTERY_SUPER_LOW_LANDING",
    )

    /**
     * Which row of §6.2 an engine disengagement belongs to.
     *
     * [detail] is the engine's own second word — DJI's authority reason verbatim, a `MODE_x`, or one
     * of the two timeout details this object defines. Null details are handled: a reason that arrives
     * without one falls to the mildest row it could be, because over-blocking a resume is a real cost
     * and this function is plumbing rather than policy.
     *
     * **Not every cause comes through here**, and [MissionPauseCause.LAND_TAG_REFUSED] is the one that
     * does not: nothing disengaged, so there is no [GuidedStickEngine.DisengageReason] to translate.
     * The engine reports that row to the sink directly, which is why [consequence] is the policy half
     * and this is only the translation of *gestures*.
     */
    fun causeOf(reason: GuidedStickEngine.DisengageReason, detail: String?): MissionPauseCause =
        when (reason) {
            GuidedStickEngine.DisengageReason.RC_STICKS -> MissionPauseCause.RC_STICK_GRAB
            GuidedStickEngine.DisengageReason.INTERLOCK -> MissionPauseCause.INTERLOCK_OFF
            GuidedStickEngine.DisengageReason.LINK_LOST -> MissionPauseCause.QGC_LINK_LOSS
            GuidedStickEngine.DisengageReason.NO_POSITION -> MissionPauseCause.POSITION_LOST

            // Unreachable while a mission is running, and **still unreachable after the precision
            // NAV_LAND** (2026-07-30): the two are mutually exclusive setpoint sources at every
            // instant, because the sequence *ends the run* — `mission` is nulled under the lock —
            // before the arm is attempted, and the descent's own busy gate refuses over a live one.
            // So a tag loss during a mission's landing arrives when there is no run to report about,
            // and the sink call never happens. The `when` is exhaustive on purpose, and the
            // sensor-loss row is the honest family if the ordering ever changes.
            GuidedStickEngine.DisengageReason.TAG_LOST -> MissionPauseCause.POSITION_LOST
            // Same unreachability argument as TAG_LOST, and the same 2026-07-30 note: a touchdown
            // belongs to a tag autoland, which by then owns the aircraft alone. The idle-hold row is
            // the honest family if that ever changes — the aircraft is down and nothing is being flown.
            GuidedStickEngine.DisengageReason.TOUCHDOWN -> MissionPauseCause.IDLE_HOLD
            GuidedStickEngine.DisengageReason.STOPPED -> MissionPauseCause.BRIDGE_STOPPED
            GuidedStickEngine.DisengageReason.IDLE -> MissionPauseCause.IDLE_HOLD

            // The GCS stick stream stopping while the link is alive. During a mission there is no
            // stick stream driving anything, so this can only arrive from a passthrough that took
            // the mission over — row 3's aftermath, and row 3's consequence.
            GuidedStickEngine.DisengageReason.RELEASED -> MissionPauseCause.GCS_STICK_DEFLECTION

            GuidedStickEngine.DisengageReason.TIMEOUT -> when (detail) {
                DETAIL_MISSION_TIMEOUT -> MissionPauseCause.MISSION_TIMEOUT
                else -> MissionPauseCause.LEG_TIMEOUT
            }

            GuidedStickEngine.DisengageReason.AUTHORITY -> when {
                detail == REASON_RC_GO_HOME -> MissionPauseCause.RC_RETURN_HOME
                detail == REASON_SEND_FAILED -> MissionPauseCause.SEND_FAILED
                detail == REASON_RC_STICK_NULL -> MissionPauseCause.PRODUCT_GONE
                detail != null && detail.startsWith(PREFIX_MODE) -> MissionPauseCause.DJI_MODE_SEIZURE
                detail in WORLD_REASONS -> MissionPauseCause.WORLD_AUTHORITY_CHANGE
                else -> MissionPauseCause.RC_AUTHORITY_CHANGE
            }
        }

    /**
     * §6.2's third column. Exhaustive over [MissionPauseCause] with no `else`, so a new row cannot
     * be added without an answer.
     *
     * The three answers and how they are chosen (§6.1):
     *
     *  - **Pause-and-resumable** when the gesture says *"not now"*.
     *  - **Pause-and-blocked** when the gesture says *"something about the world has changed"*.
     *  - **Abandon** only when the plan itself is no longer the plan we admitted.
     *
     * There is no case in which a gesture leaves the mission **running**. That is the whole point of
     * a ladder, and a test asserts it over every value rather than trusting the reading.
     */
    fun consequence(cause: MissionPauseCause): MissionConsequence = when (cause) {
        MissionPauseCause.RC_STICK_GRAB -> MissionConsequence.PauseResumable
        MissionPauseCause.RC_RETURN_HOME -> MissionConsequence.PauseBlocked(ResumeBlock.GOING_HOME)
        MissionPauseCause.GCS_STICK_DEFLECTION -> MissionConsequence.PauseResumable
        MissionPauseCause.GCS_NEW_DESTINATION -> MissionConsequence.PauseResumable
        MissionPauseCause.QGC_LINK_LOSS -> MissionConsequence.PauseResumable
        MissionPauseCause.INTERLOCK_OFF -> MissionConsequence.PauseBlocked(ResumeBlock.INTERLOCK_OFF)
        MissionPauseCause.DJI_MODE_SEIZURE -> MissionConsequence.PauseBlocked(ResumeBlock.MODE_SEIZED)
        MissionPauseCause.RC_AUTHORITY_CHANGE -> MissionConsequence.PauseResumable
        MissionPauseCause.WORLD_AUTHORITY_CHANGE -> MissionConsequence.PauseBlocked(ResumeBlock.WORLD_CHANGED)
        MissionPauseCause.PRODUCT_GONE -> MissionConsequence.PauseResumable
        MissionPauseCause.POSITION_LOST -> MissionConsequence.PauseResumable
        MissionPauseCause.VELOCITY_STALE -> MissionConsequence.None
        MissionPauseCause.ALTITUDE_STALE -> MissionConsequence.None
        MissionPauseCause.LOW_BATTERY_WARNING -> MissionConsequence.PauseBlocked(ResumeBlock.LOW_BATTERY)
        MissionPauseCause.SERIOUS_LOW_BATTERY -> MissionConsequence.PauseBlocked(ResumeBlock.BATTERY_DEPLETED)
        MissionPauseCause.SEND_FAILED -> MissionConsequence.PauseResumable
        MissionPauseCause.LEG_TIMEOUT -> MissionConsequence.PauseBlocked(ResumeBlock.LEG_TIMEOUT)
        MissionPauseCause.MISSION_TIMEOUT -> MissionConsequence.PauseBlocked(ResumeBlock.MISSION_TIMEOUT)
        MissionPauseCause.BRIDGE_STOPPED -> MissionConsequence.Abandon
        MissionPauseCause.PLAN_CHANGED -> MissionConsequence.Abandon
        MissionPauseCause.GCS_PAUSE -> MissionConsequence.PauseResumable
        MissionPauseCause.IDLE_HOLD -> MissionConsequence.PauseResumable
        MissionPauseCause.LAND_TAG_REFUSED ->
            MissionConsequence.PauseBlocked(ResumeBlock.LAND_TAG_REFUSED)
    }

    /** The consequence of an engine disengagement, in one call. [causeOf] then [consequence]. */
    fun consequenceOf(
        reason: GuidedStickEngine.DisengageReason,
        detail: String?,
    ): MissionConsequence = consequence(causeOf(reason, detail))
}
