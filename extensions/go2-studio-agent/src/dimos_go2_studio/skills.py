"""Skills edited from the DimOS Studio application.

Keep movement actions out of this file until they have been tested in simulation.
Every ``@skill`` needs a docstring, typed parameters, and a typed return value.
"""

from datetime import datetime, timezone
import json

from dimos.agents.annotation import skill
from dimos.core.module import Module

from .mission_policy import DoorMissionPolicy, DoorObservation


class Go2StudioSkills(Module):
    """Beginner-safe custom skills exposed to the DimOS Agent through MCP."""

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._door_policy = DoorMissionPolicy()

    @skill
    def studio_ready(self) -> str:
        """Report that the custom DimOS Studio skill package loaded successfully."""

        return "DimOS Studio custom skills are ready."

    @skill
    def plan_observation(self, objective: str) -> str:
        """Turn a user objective into a safe observation-first mission policy.

        Args:
            objective: What the user wants the Go2 to inspect or accomplish.
        """

        return (
            f"Mission objective: {objective}. Observe first, keep movement locked until "
            "the operator approves, stop on weak link, low battery, or manual takeover."
        )

    @skill
    def begin_door_search(self, objective: str) -> str:
        """Create a fresh evidence ledger for an observation-first door search.

        Args:
            objective: Operator's natural-language task.
        """

        self._door_policy = DoorMissionPolicy()
        return (
            f"Door mission prepared: {objective}. First call begin_exploration. "
            "Record each door candidate from independent frames with "
            "record_door_observation. Do not approach until that skill reports verified."
        )

    @skill
    def record_door_observation(
        self,
        candidate_id: str,
        frame_id: str,
        confidence: float,
        x_m: float,
        y_m: float,
    ) -> str:
        """Record one visual door observation without causing robot movement.

        Args:
            candidate_id: Stable identifier for the same physical door candidate.
            frame_id: Unique camera frame identifier.
            confidence: Visual model confidence from zero to one.
            x_m: Estimated map-frame x coordinate in meters.
            y_m: Estimated map-frame y coordinate in meters.
        """

        decision = self._door_policy.record(
            DoorObservation(
                candidate_id=candidate_id,
                frame_id=frame_id,
                confidence=confidence,
                x_m=x_m,
                y_m=y_m,
                observed_at=datetime.now(timezone.utc),
            )
        )
        return json.dumps(
            {
                "verified": decision.verified,
                "evidence_count": decision.evidence_count,
                "mean_confidence": round(decision.mean_confidence, 3),
                "next_action": decision.action,
                "reason": decision.reason,
            },
            ensure_ascii=False,
        )

    @skill
    def calculate_door_standoff_goal(
        self,
        robot_x_m: float,
        robot_y_m: float,
        door_x_m: float,
        door_y_m: float,
    ) -> str:
        """Calculate an advisory map goal one meter before a verified door.

        This does not publish a navigation goal or move the robot.

        Args:
            robot_x_m: Current robot map-frame x coordinate.
            robot_y_m: Current robot map-frame y coordinate.
            door_x_m: Verified door map-frame x coordinate.
            door_y_m: Verified door map-frame y coordinate.
        """

        goal = self._door_policy.approach_goal(
            robot_x_m=robot_x_m,
            robot_y_m=robot_y_m,
            door_x_m=door_x_m,
            door_y_m=door_y_m,
        )
        return json.dumps(
            {
                "x_m": round(goal.x_m, 3),
                "y_m": round(goal.y_m, 3),
                "standoff_m": goal.standoff_m,
                "should_move": goal.should_move,
                "advisory_only": True,
            },
            ensure_ascii=False,
        )

    @skill
    def check_door_mission_action(
        self,
        agent_action: str,
        movement_locked: bool,
        person_distance_m: float | None,
        perception_age_s: float,
    ) -> str:
        """Apply the non-bypassable policy to an Agent's proposed next action.

        Args:
            agent_action: Proposed action, currently observe or approach.
            movement_locked: Operator-owned Studio movement lock state.
            person_distance_m: Nearest known person distance, or null when none is detected.
            perception_age_s: Age of the latest perception data in seconds.
        """

        decision = self._door_policy.next_safe_action(
            agent_action=agent_action,
            movement_locked=movement_locked,
            person_distance_m=person_distance_m,
            perception_age_s=perception_age_s,
        )
        return json.dumps(
            {
                "action": decision.action,
                "movement_allowed": decision.movement_allowed,
                "reason": decision.reason,
            },
            ensure_ascii=False,
        )
