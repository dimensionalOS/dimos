package com.dimensional.mini4pro.guided

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The lifecycle state machine and **every row of `docs/m4-mission-execution.md` §6.2** — the table
 * that says what each abort gesture does to a running plan.
 *
 * Two disciplines are in force here, both deliberate:
 *
 *  1. **The transition table is enumerated exhaustively at runtime** against a hand-written
 *     expectation, so a *wrong* answer is caught as well as a missing one. The `when`s in
 *     [MissionTransitions] are already exhaustive over both enums with no `else`, so a new state or
 *     event **fails the build** before this test ever runs; [every state and event pair has an
 *     answer, and the build fails without one] is the belt to that compiler's braces.
 *  2. **One named test per abort row.** §6.2 is nineteen rows and it is the heart of the design, so
 *     each row's mission consequence is asserted by a test named after the row rather than by a
 *     table-driven loop that reads as one assertion.
 *
 * Written to fail loudly for:
 *
 *  - **`PAUSED → RUNNING` by anything other than an explicit Start**, which is Ivan's binding M4-11
 *    answer and the property the whole design rests on.
 *  - **a gesture that leaves the mission running**, which is the failure a ladder exists to prevent.
 *  - a pause consequence swapped for another (resumable ↔ blocked ↔ abandon).
 *  - a resume block cleared early, or one that never clears at all.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one at a time, applied to the shipped source, the **whole** suite
 * run, confirmed red, reverted. Counts are failing tests across all 1802.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `PAUSED + START_REFUSED → RUNNING` (an automatic resume) | 7 |
 *  | a paused cursor survives a bridge stop | 2 |
 *  | a stick grab abandons the plan instead of pausing it | 5 |
 *  | the interlock pause made resumable rather than blocked | 2 |
 *  | a leg timeout made resumable rather than blocked | 2 |
 *  | every DJI authority reason read as an RC decision (row 7b folded into 7a) | 2 |
 *
 * The first row is the one that matters: seven tests notice an automatic resume, which is the
 * measurement behind M4-11 being called binding rather than intended.
 */
class MissionLifecycleTest {

    // ------------------------------------------------------------------ the transition table

    @Test
    fun `every state and event pair has an answer, and the build fails without one`() {
        // The compiler already enforces this: MissionTransitions has no `else` in either `when`.
        // This asserts the shape of the enumeration so that a reader of the test knows the count,
        // and so that a pair silently answered `null` by a future edit is visible below.
        val pairs = MissionLifecycle.values().flatMap { state ->
            MissionEvent.values().map { event -> state to event }
        }
        assertEquals(MissionLifecycle.values().size * MissionEvent.values().size, pairs.size)
        assertEquals(EXPECTED.size, pairs.size)
        for (pair in pairs) {
            assertTrue("no expectation written for $pair", EXPECTED.containsKey(pair))
            assertEquals(
                "transition $pair",
                EXPECTED[pair],
                MissionTransitions.next(pair.first, pair.second),
            )
        }
    }

    @Test
    fun `a plan committed with nothing loaded moves to LOADED`() {
        assertEquals(
            MissionLifecycle.LOADED,
            MissionTransitions.next(MissionLifecycle.NO_MISSION, MissionEvent.PLAN_COMMITTED),
        )
    }

    @Test
    fun `THE ONE CAUSE - PAUSED becomes RUNNING only on an accepted Start`() {
        // M4-11, binding: "yeah never automatic". Every other event leaves a paused mission paused,
        // dropped to LOADED, or cleared — never running.
        for (event in MissionEvent.values()) {
            val next = MissionTransitions.next(MissionLifecycle.PAUSED, event)
            if (event == MissionEvent.START_ACCEPTED) {
                assertEquals(MissionLifecycle.RUNNING, next)
            } else {
                assertNotEquals("event $event resumed a paused mission", MissionLifecycle.RUNNING, next)
            }
        }
    }

    @Test
    fun `a refused Start changes nothing, because nothing flew`() {
        assertEquals(
            MissionLifecycle.PAUSED,
            MissionTransitions.next(MissionLifecycle.PAUSED, MissionEvent.START_REFUSED),
        )
        assertEquals(
            MissionLifecycle.LOADED,
            MissionTransitions.next(MissionLifecycle.LOADED, MissionEvent.START_REFUSED),
        )
    }

    @Test
    fun `a paused cursor is dropped when the plan it indexed changes`() {
        assertEquals(
            MissionLifecycle.LOADED,
            MissionTransitions.next(MissionLifecycle.PAUSED, MissionEvent.PLAN_GENERATION_CHANGED),
        )
    }

    @Test
    fun `M4-12 - an upload arriving mid-flight does not disturb the run`() {
        // The store changes; the running mission carries on with the snapshot it began with, and
        // the new plan takes effect at the next Start.
        assertEquals(
            MissionLifecycle.RUNNING,
            MissionTransitions.next(MissionLifecycle.RUNNING, MissionEvent.PLAN_COMMITTED),
        )
        assertEquals(
            MissionLifecycle.RUNNING,
            MissionTransitions.next(MissionLifecycle.RUNNING, MissionEvent.PLAN_GENERATION_CHANGED),
        )
    }

    @Test
    fun `a Start while running is illegal, not a resume`() {
        assertNull(MissionTransitions.next(MissionLifecycle.RUNNING, MissionEvent.START_ACCEPTED))
        assertFalse(
            "a Start while running is a caller skipping the launch check — never moot",
            MissionTransitions.isMoot(MissionLifecycle.RUNNING, MissionEvent.START_ACCEPTED),
        )
    }

    /**
     * **The commonest ending there is, and it used to be logged as a fault.**
     *
     * A mission reaches its last item, the lifecycle goes `FINISHED`, the aircraft holds station
     * still engaged, and the operator takes the sticks. The engine reports `RUN_PAUSED` — correct,
     * its run did end — and the state machine has nothing to do with it, also correct. That was
     * logged as *"illegal mission transition ignored: FINISHED + RUN_PAUSED"* on the first real
     * mission this project flew, where nothing had gone wrong at all.
     *
     * Both halves are asserted: the transition stays `null` (nothing happens), and it is *named*
     * moot rather than illegal, because the whole point is what the operator reads afterwards.
     */
    @Test
    fun `a run report with no run is moot, not illegal`() {
        for (state in MissionLifecycle.entries.filter { it != MissionLifecycle.RUNNING }) {
            for (event in listOf(MissionEvent.RUN_PAUSED, MissionEvent.LAST_ITEM_COMPLETE)) {
                assertNull("$state + $event must not transition", MissionTransitions.next(state, event))
                assertTrue("$state + $event is moot", MissionTransitions.isMoot(state, event))
            }
        }
    }

    @Test
    fun `a run report while running is neither moot nor ignored`() {
        // The guard must not swallow the two transitions that carry a real run to its end.
        assertEquals(
            MissionLifecycle.PAUSED,
            MissionTransitions.next(MissionLifecycle.RUNNING, MissionEvent.RUN_PAUSED),
        )
        assertEquals(
            MissionLifecycle.FINISHED,
            MissionTransitions.next(MissionLifecycle.RUNNING, MissionEvent.LAST_ITEM_COMPLETE),
        )
        assertFalse(MissionTransitions.isMoot(MissionLifecycle.RUNNING, MissionEvent.RUN_PAUSED))
        assertFalse(
            MissionTransitions.isMoot(MissionLifecycle.RUNNING, MissionEvent.LAST_ITEM_COMPLETE)
        )
    }

    @Test
    fun `FINISHED means arrived and holding, and a Start from it re-runs the whole launch check`() {
        assertEquals(
            MissionLifecycle.FINISHED,
            MissionTransitions.next(MissionLifecycle.RUNNING, MissionEvent.LAST_ITEM_COMPLETE),
        )
        assertEquals(
            MissionLifecycle.RUNNING,
            MissionTransitions.next(MissionLifecycle.FINISHED, MissionEvent.START_ACCEPTED),
        )
    }

    @Test
    fun `nothing survives a bridge stop`() {
        for (state in MissionLifecycle.values()) {
            assertEquals(
                "state $state survived stop()",
                MissionLifecycle.NO_MISSION,
                MissionTransitions.next(state, MissionEvent.BRIDGE_STOPPED),
            )
        }
    }

    // --------------------------------------------------------- §6.2, one test per row

    @Test
    fun `row 1 - an RC stick grab pauses the mission, resumably`() {
        // The pilot's hands mean "I have it", not "delete the plan". Their next act is usually to
        // fix something and give it back; abandoning would punish the correct instinct.
        assertEquals(
            MissionPauseCause.RC_STICK_GRAB,
            MissionAbortPolicy.causeOf(GuidedStickEngine.DisengageReason.RC_STICKS, null),
        )
        assertEquals(
            MissionConsequence.PauseResumable,
            MissionAbortPolicy.consequenceOf(GuidedStickEngine.DisengageReason.RC_STICKS, null),
        )
    }

    @Test
    fun `row 2 - the RC Return button pauses the mission, blocked while it goes home`() {
        val consequence = MissionAbortPolicy.consequenceOf(
            GuidedStickEngine.DisengageReason.AUTHORITY, MissionAbortPolicy.REASON_RC_GO_HOME,
        )
        assertEquals(MissionConsequence.PauseBlocked(ResumeBlock.GOING_HOME), consequence)
    }

    @Test
    fun `row 3 - a GCS stick deflection pauses the mission, resumably`() {
        // The operator chose the GCS-stick channel, so they get GCS-stick control. Snatching the
        // aircraft to an RC across the field would be worse. The plan waits.
        assertEquals(
            MissionConsequence.PauseResumable,
            MissionAbortPolicy.consequence(MissionPauseCause.GCS_STICK_DEFLECTION),
        )
        assertEquals(
            MissionPauseCause.GCS_STICK_DEFLECTION,
            MissionAbortPolicy.causeOf(GuidedStickEngine.DisengageReason.RELEASED, null),
        )
    }

    @Test
    fun `row 4 - QGC link loss pauses the mission, resumably and never automatically`() {
        // A returning link is evidence of a link, not of an operator. The "never automatically"
        // half is the lifecycle's, asserted above; this is the consequence half.
        assertEquals(
            MissionConsequence.PauseResumable,
            MissionAbortPolicy.consequenceOf(GuidedStickEngine.DisengageReason.LINK_LOST, null),
        )
    }

    @Test
    fun `row 5 - the interlock going off pauses the mission, blocked while it is off`() {
        val consequence =
            MissionAbortPolicy.consequenceOf(GuidedStickEngine.DisengageReason.INTERLOCK, null)
        assertEquals(MissionConsequence.PauseBlocked(ResumeBlock.INTERLOCK_OFF), consequence)
        assertEquals(BlockPersistence.WHILE_CONDITION, ResumeBlock.INTERLOCK_OFF.persistence)
    }

    @Test
    fun `row 6 - DJI seizing the flight mode pauses the mission, blocked`() {
        // The measured overheat GO_HOME: DJI flew the aircraft for 40 s while VirtualStickState
        // still reported MSDK. The reason has not gone away just because the mode has.
        val consequence = MissionAbortPolicy.consequenceOf(
            GuidedStickEngine.DisengageReason.AUTHORITY, MissionAbortPolicy.PREFIX_MODE + "GO_HOME",
        )
        assertEquals(MissionConsequence.PauseBlocked(ResumeBlock.MODE_SEIZED), consequence)
    }

    @Test
    fun `row 7a - an RC-side authority change pauses the mission, resumably`() {
        // An RC mode switch is the pilot deciding, not the world changing.
        for (reason in listOf("RC_SWITCH", "RC_PAUSE_STOP", "RC_NOT_P_MODE", "RC_LOST")) {
            assertEquals(
                "reason $reason",
                MissionConsequence.PauseResumable,
                MissionAbortPolicy.consequenceOf(GuidedStickEngine.DisengageReason.AUTHORITY, reason),
            )
        }
    }

    @Test
    fun `row 7b - a geofence or battery failsafe pauses the mission, blocked`() {
        for (reason in MissionAbortPolicy.WORLD_REASONS) {
            assertEquals(
                "reason $reason",
                MissionConsequence.PauseBlocked(ResumeBlock.WORLD_CHANGED),
                MissionAbortPolicy.consequenceOf(GuidedStickEngine.DisengageReason.AUTHORITY, reason),
            )
        }
    }

    @Test
    fun `row 8 - a product gone or a null RC stick feed pauses the mission, resumably`() {
        // A blackout is transient and measured (nulls across the board for 45–120 s). Blocking
        // would make a normal post-flight event look like a fault.
        assertEquals(
            MissionConsequence.PauseResumable,
            MissionAbortPolicy.consequenceOf(
                GuidedStickEngine.DisengageReason.AUTHORITY, MissionAbortPolicy.REASON_RC_STICK_NULL,
            ),
        )
    }

    @Test
    fun `row 9 - a lost position feed pauses the mission, resumably`() {
        assertEquals(
            MissionConsequence.PauseResumable,
            MissionAbortPolicy.consequenceOf(GuidedStickEngine.DisengageReason.NO_POSITION, null),
        )
    }

    @Test
    fun `row 10 - a stale velocity feed has no consequence for the plan`() {
        // It withholds *arrival* only; a pass-through is geometric and does not read speed.
        assertEquals(
            MissionConsequence.None,
            MissionAbortPolicy.consequence(MissionPauseCause.VELOCITY_STALE),
        )
    }

    @Test
    fun `row 11 - a stale altitude feed has no consequence for the plan`() {
        assertEquals(
            MissionConsequence.None,
            MissionAbortPolicy.consequence(MissionPauseCause.ALTITUDE_STALE),
        )
    }

    @Test
    fun `row 12 - DJI's first low-battery warning pauses the mission, blocked for the session`() {
        assertEquals(
            MissionConsequence.PauseBlocked(ResumeBlock.LOW_BATTERY),
            MissionAbortPolicy.consequence(MissionPauseCause.LOW_BATTERY_WARNING),
        )
        assertEquals(BlockPersistence.SESSION, ResumeBlock.LOW_BATTERY.persistence)
    }

    @Test
    fun `row 13 - a serious low battery pauses the mission, blocked for the session`() {
        // A battery does not un-deplete.
        assertEquals(
            MissionConsequence.PauseBlocked(ResumeBlock.BATTERY_DEPLETED),
            MissionAbortPolicy.consequence(MissionPauseCause.SERIOUS_LOW_BATTERY),
        )
        assertEquals(BlockPersistence.SESSION, ResumeBlock.BATTERY_DEPLETED.persistence)
    }

    @Test
    fun `rows 12 and 13 are unreachable from the engine until DJI's warning keys are wired`() {
        // §6.5: AircraftState carries no KeyIsLowBatteryWarning, and this layer is forbidden from
        // inferring one from a percentage. Until the keys exist the two rows degrade to row 7 —
        // we still abort on DJI's battery *failsafe* authority reasons — and this test pins the
        // degradation so that wiring the keys is a visible change rather than a silent one.
        for (reason in GuidedStickEngine.DisengageReason.values()) {
            for (detail in listOf(null, "BATTERY_LOW_GO_HOME", "BATTERY_SUPER_LOW_LANDING")) {
                val cause = MissionAbortPolicy.causeOf(reason, detail)
                assertNotEquals(MissionPauseCause.LOW_BATTERY_WARNING, cause)
                assertNotEquals(MissionPauseCause.SERIOUS_LOW_BATTERY, cause)
            }
        }
        // And the failsafe reasons do still pause, blocked, through row 7b.
        assertEquals(
            MissionConsequence.PauseBlocked(ResumeBlock.WORLD_CHANGED),
            MissionAbortPolicy.consequenceOf(
                GuidedStickEngine.DisengageReason.AUTHORITY, "BATTERY_LOW_GO_HOME",
            ),
        )
    }

    @Test
    fun `row 14 - a send failure pauses the mission, resumably`() {
        assertEquals(
            MissionConsequence.PauseResumable,
            MissionAbortPolicy.consequenceOf(
                GuidedStickEngine.DisengageReason.AUTHORITY, MissionAbortPolicy.REASON_SEND_FAILED,
            ),
        )
    }

    @Test
    fun `row 15 - a leg timeout pauses the mission, blocked until the operator is told`() {
        val consequence = MissionAbortPolicy.consequenceOf(
            GuidedStickEngine.DisengageReason.TIMEOUT, MissionAbortPolicy.DETAIL_LEG_TIMEOUT,
        )
        assertEquals(MissionConsequence.PauseBlocked(ResumeBlock.LEG_TIMEOUT), consequence)
        assertEquals(BlockPersistence.SPENT_ON_REFUSAL, ResumeBlock.LEG_TIMEOUT.persistence)
    }

    @Test
    fun `row 16 - a mission timeout pauses the mission, blocked until the operator is told`() {
        val consequence = MissionAbortPolicy.consequenceOf(
            GuidedStickEngine.DisengageReason.TIMEOUT, MissionAbortPolicy.DETAIL_MISSION_TIMEOUT,
        )
        assertEquals(MissionConsequence.PauseBlocked(ResumeBlock.MISSION_TIMEOUT), consequence)
    }

    @Test
    fun `row 17 - a bridge stop abandons the plan, because nothing is persisted`() {
        // A plan resuming into a session nobody armed is the failure this whole design prevents.
        assertEquals(
            MissionConsequence.Abandon,
            MissionAbortPolicy.consequenceOf(GuidedStickEngine.DisengageReason.STOPPED, null),
        )
    }

    @Test
    fun `row 18 - a re-uploaded or cleared plan abandons the paused cursor`() {
        assertEquals(
            MissionConsequence.Abandon,
            MissionAbortPolicy.consequence(MissionPauseCause.PLAN_CHANGED),
        )
    }

    @Test
    fun `row 19 - QGC's Pause button pauses the mission, resumably`() {
        // Q3 already named Pause the natural GCS-side convenience abort — a withdrawal, not a new
        // intent.
        assertEquals(
            MissionConsequence.PauseResumable,
            MissionAbortPolicy.consequence(MissionPauseCause.GCS_PAUSE),
        )
    }

    @Test
    fun `beyond the table - a refused tag landing pauses the plan, blocked until the operator is told`() {
        // The row that is not an abort of anything: nothing seized the aircraft and the engagement is
        // intact — the run reached its last item and could not do the one thing that item asked for.
        // SPENT_ON_REFUSAL, on the leg-timeout argument: the message is *look at the aircraft, or at
        // the plan*, and one refusal that makes the operator think beats a lockout they restart around.
        assertEquals(
            MissionConsequence.PauseBlocked(ResumeBlock.LAND_TAG_REFUSED),
            MissionAbortPolicy.consequence(MissionPauseCause.LAND_TAG_REFUSED),
        )
        assertEquals(BlockPersistence.SPENT_ON_REFUSAL, ResumeBlock.LAND_TAG_REFUSED.persistence)
        // And it is deliberately unreachable from `causeOf`: no disengagement produces it, so no
        // gesture nobody made can be put on the record by translating one.
        for (reason in GuidedStickEngine.DisengageReason.entries) {
            for (detail in listOf(null, "MODE_GO_HOME", MissionAbortPolicy.DETAIL_LEG_TIMEOUT)) {
                assertNotEquals(
                    MissionPauseCause.LAND_TAG_REFUSED,
                    MissionAbortPolicy.causeOf(reason, detail),
                )
            }
        }
    }

    @Test
    fun `beyond the table - the idle window expiring on a hold leaves the plan resumable`() {
        // Not in §6.2 because §6.2 predates FINISHED meaning "holding in the air". A mission that
        // ends holding is holding authority, and the idle window is one of the things that ends it.
        assertEquals(
            MissionConsequence.PauseResumable,
            MissionAbortPolicy.consequenceOf(GuidedStickEngine.DisengageReason.IDLE, null),
        )
    }

    @Test
    fun `THE LADDER - no gesture anywhere leaves the mission running`() {
        for (cause in MissionPauseCause.values()) {
            val consequence = MissionAbortPolicy.consequence(cause)
            val stops = consequence != MissionConsequence.None
            // Rows 10 and 11 are the only two that do not stop the run, and they are the two that
            // degrade an axis rather than ending the engagement.
            val degradesAnAxis =
                cause == MissionPauseCause.VELOCITY_STALE || cause == MissionPauseCause.ALTITUDE_STALE
            assertEquals("cause $cause", !degradesAnAxis, stops)
        }
    }

    @Test
    fun `every engine disengagement reason maps to a row, with and without a detail`() {
        for (reason in GuidedStickEngine.DisengageReason.values()) {
            for (detail in listOf(null, "", "SOMETHING_DJI_SAID")) {
                val cause = MissionAbortPolicy.causeOf(reason, detail)
                // None of them may be a row that means "nothing happened": an engagement that
                // ended has ended the run.
                assertNotEquals(
                    "reason $reason detail $detail",
                    MissionConsequence.None,
                    MissionAbortPolicy.consequence(cause),
                )
            }
        }
    }

    @Test
    fun `a block that is spent on refusal is distinguishable from one that stands`() {
        // The three persistences exist because the nineteen rows need three, and the distinction is
        // what makes "the operator has been told" a legitimate way for a block to end.
        assertEquals(BlockPersistence.SPENT_ON_REFUSAL, ResumeBlock.LEG_TIMEOUT.persistence)
        assertEquals(BlockPersistence.WHILE_CONDITION, ResumeBlock.MODE_SEIZED.persistence)
        assertEquals(BlockPersistence.SESSION, ResumeBlock.BATTERY_DEPLETED.persistence)
        // And every block has a reason word short enough to survive the 50-byte STATUSTEXT framing.
        for (block in ResumeBlock.values()) {
            assertTrue(
                "block ${block.name} reason too long: ${block.reason}",
                block.reason.toByteArray(Charsets.UTF_8).size <= 24,
            )
        }
    }

    private companion object {
        /**
         * The whole table, written by hand so that a *wrong* answer is caught as well as a missing
         * one. Null means the pair is illegal — the caller ignores the event and logs it, which is
         * not the same thing as a legal no-change.
         */
        val EXPECTED: Map<Pair<MissionLifecycle, MissionEvent>, MissionLifecycle?> = mapOf(
            (MissionLifecycle.NO_MISSION to MissionEvent.PLAN_COMMITTED) to MissionLifecycle.LOADED,
            (MissionLifecycle.NO_MISSION to MissionEvent.PLAN_CLEARED) to MissionLifecycle.NO_MISSION,
            (MissionLifecycle.NO_MISSION to MissionEvent.START_ACCEPTED) to null,
            (MissionLifecycle.NO_MISSION to MissionEvent.START_REFUSED) to MissionLifecycle.NO_MISSION,
            (MissionLifecycle.NO_MISSION to MissionEvent.LAST_ITEM_COMPLETE) to null,
            (MissionLifecycle.NO_MISSION to MissionEvent.RUN_PAUSED) to null,
            (MissionLifecycle.NO_MISSION to MissionEvent.PLAN_GENERATION_CHANGED) to MissionLifecycle.NO_MISSION,
            (MissionLifecycle.NO_MISSION to MissionEvent.BRIDGE_STOPPED) to MissionLifecycle.NO_MISSION,

            (MissionLifecycle.LOADED to MissionEvent.PLAN_COMMITTED) to MissionLifecycle.LOADED,
            (MissionLifecycle.LOADED to MissionEvent.PLAN_CLEARED) to MissionLifecycle.NO_MISSION,
            (MissionLifecycle.LOADED to MissionEvent.START_ACCEPTED) to MissionLifecycle.RUNNING,
            (MissionLifecycle.LOADED to MissionEvent.START_REFUSED) to MissionLifecycle.LOADED,
            (MissionLifecycle.LOADED to MissionEvent.LAST_ITEM_COMPLETE) to null,
            (MissionLifecycle.LOADED to MissionEvent.RUN_PAUSED) to null,
            (MissionLifecycle.LOADED to MissionEvent.PLAN_GENERATION_CHANGED) to MissionLifecycle.LOADED,
            (MissionLifecycle.LOADED to MissionEvent.BRIDGE_STOPPED) to MissionLifecycle.NO_MISSION,

            (MissionLifecycle.RUNNING to MissionEvent.PLAN_COMMITTED) to MissionLifecycle.RUNNING,
            (MissionLifecycle.RUNNING to MissionEvent.PLAN_CLEARED) to MissionLifecycle.NO_MISSION,
            (MissionLifecycle.RUNNING to MissionEvent.START_ACCEPTED) to null,
            (MissionLifecycle.RUNNING to MissionEvent.START_REFUSED) to MissionLifecycle.RUNNING,
            (MissionLifecycle.RUNNING to MissionEvent.LAST_ITEM_COMPLETE) to MissionLifecycle.FINISHED,
            (MissionLifecycle.RUNNING to MissionEvent.RUN_PAUSED) to MissionLifecycle.PAUSED,
            (MissionLifecycle.RUNNING to MissionEvent.PLAN_GENERATION_CHANGED) to MissionLifecycle.RUNNING,
            (MissionLifecycle.RUNNING to MissionEvent.BRIDGE_STOPPED) to MissionLifecycle.NO_MISSION,

            (MissionLifecycle.PAUSED to MissionEvent.PLAN_COMMITTED) to MissionLifecycle.LOADED,
            (MissionLifecycle.PAUSED to MissionEvent.PLAN_CLEARED) to MissionLifecycle.NO_MISSION,
            (MissionLifecycle.PAUSED to MissionEvent.START_ACCEPTED) to MissionLifecycle.RUNNING,
            (MissionLifecycle.PAUSED to MissionEvent.START_REFUSED) to MissionLifecycle.PAUSED,
            (MissionLifecycle.PAUSED to MissionEvent.LAST_ITEM_COMPLETE) to null,
            (MissionLifecycle.PAUSED to MissionEvent.RUN_PAUSED) to null,
            (MissionLifecycle.PAUSED to MissionEvent.PLAN_GENERATION_CHANGED) to MissionLifecycle.LOADED,
            (MissionLifecycle.PAUSED to MissionEvent.BRIDGE_STOPPED) to MissionLifecycle.NO_MISSION,

            (MissionLifecycle.FINISHED to MissionEvent.PLAN_COMMITTED) to MissionLifecycle.LOADED,
            (MissionLifecycle.FINISHED to MissionEvent.PLAN_CLEARED) to MissionLifecycle.NO_MISSION,
            (MissionLifecycle.FINISHED to MissionEvent.START_ACCEPTED) to MissionLifecycle.RUNNING,
            (MissionLifecycle.FINISHED to MissionEvent.START_REFUSED) to MissionLifecycle.FINISHED,
            (MissionLifecycle.FINISHED to MissionEvent.LAST_ITEM_COMPLETE) to null,
            (MissionLifecycle.FINISHED to MissionEvent.RUN_PAUSED) to null,
            (MissionLifecycle.FINISHED to MissionEvent.PLAN_GENERATION_CHANGED) to MissionLifecycle.LOADED,
            (MissionLifecycle.FINISHED to MissionEvent.BRIDGE_STOPPED) to MissionLifecycle.NO_MISSION,
        )
    }
}
