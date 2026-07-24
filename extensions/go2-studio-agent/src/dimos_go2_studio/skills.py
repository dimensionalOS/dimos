"""Skills edited from the DimOS Studio application.

Keep movement actions out of this file until they have been tested in simulation.
Every ``@skill`` needs a docstring, typed parameters, and a typed return value.
"""

from dimos.agents.annotation import skill
from dimos.core.module import Module


class Go2StudioSkills(Module):
    """Beginner-safe custom skills exposed to the DimOS Agent through MCP."""

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
