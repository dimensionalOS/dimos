# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Harness modules for the replay-backed spatial goal-selection evals.

The eval builds a blueprint out of shipping modules (``SpatialMemory``,
``NavigationSkillContainer``, ``McpServer``, ``McpClient``) plus the four
observation-only modules defined here:

``FakeCamera`` / ``FakeOdom``
    Publish nothing; they exist so ``NavigationSkillContainer`` and
    ``SpatialMemory`` have producers for their ``In`` ports. The eval answers
    questions from an already-ingested memory, so no live sensor data is
    needed. (Same trick as ``dimos/agents/skills/test_navigation.py``, which
    cannot be imported from here: it is a test module.)

``RecordingStubNavigation``
    Stands in for the real navigation stack and records the goals the agent
    sets. Recording happens **inside the module's worker process** and is read
    back over ``@rpc get_goals()`` -- modules run in forkserver workers, so a
    plain list read from the test process would always look empty.

``RecordingNavigationSkillContainer``
    The shipping skill container plus a per-call record of the ``query``
    strings the agent passed to ``navigate_with_text``. Pure observation:
    it appends the query and then runs the unmodified shipping body.

``TimedAgentTestRunner``
    ``AgentTestRunner`` with its two hard-coded inner waits made configurable.

Why the modules live in their own file: forkserver workers import module
classes by qualified name, so anything deployed into a blueprint must be
importable, i.e. not defined in a ``__main__`` script or a test function.
"""

from __future__ import annotations

from collections.abc import Callable
import inspect
from typing import Any

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.testing.agent_test_runner import AgentTestRunner, Config as RunnerConfig
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.base import NavigationState

# The undecorated shipping implementation of `navigate_with_text`. The class
# attribute is the `@skill` wrapper (per-call context + timing log); unwrapping
# it lets the recording subclass below run the shipping body exactly once
# inside its OWN skill wrapper, instead of nesting a second wrapper that would
# overwrite the MCP-supplied progress-token context and log every call twice.
# `inspect.unwrap` is a no-op if the shipping method is ever left undecorated.
_SHIPPING_NAVIGATE_WITH_TEXT: Callable[[NavigationSkillContainer, str], str] = inspect.unwrap(
    NavigationSkillContainer.navigate_with_text
)


class FakeCamera(Module):
    """Satisfies the `color_image: In[Image]` refs; publishes nothing."""

    color_image: Out[Image]


class FakeOdom(Module):
    """Satisfies the `odom: In[PoseStamped]` ref; publishes nothing."""

    odom: Out[PoseStamped]


class RecordingStubNavigation(Module):
    """Navigation stand-in that records every goal the agent sets.

    Implements the same surface as ``NavigationInterfaceSpec`` (what
    ``NavigationSkillContainer`` calls) and adds a read-back channel. The eval
    scores the goal the agent *selects*; execution is deliberately stubbed, so
    ``get_state()``/``is_goal_reached()`` report a robot that never moves.

    Goals are returned as plain dicts (``x``/``y``/``z``/``yaw``/``frame_id``)
    so nothing exotic has to survive the RPC pickle.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._goals: list[dict[str, Any]] = []

    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        self._goals.append(
            {
                "x": float(goal.position.x),
                "y": float(goal.position.y),
                "z": float(goal.position.z),
                "yaw": float(goal.orientation.to_euler().z),
                "frame_id": getattr(goal, "frame_id", None),
            }
        )
        return True

    @rpc
    def get_state(self) -> NavigationState:
        return NavigationState.IDLE

    @rpc
    def is_goal_reached(self) -> bool:
        return False

    @rpc
    def cancel_goal(self) -> bool:
        return True

    @rpc
    def get_goals(self) -> list[dict[str, Any]]:
        """Read back the goals recorded in this module's worker process."""
        return list(self._goals)

    @rpc
    def reset_goals(self) -> bool:
        """Drop the recorded goals so one instance can serve several questions."""
        self._goals = []
        return True


class RecordingNavigationSkillContainer(NavigationSkillContainer):
    """Shipping navigation skills, plus a record of the queries the agent sent.

    The eval reports *what the agent asked for*, not just where it ended up:
    the exact ``query`` strings passed to ``navigate_with_text`` and how many
    times it was called (the tool-routing / query-formulation comparison).
    Those are only visible on the callee side, so this subclass records them
    in the worker process and exposes ``@rpc get_queries()``.

    Observation only: the query is appended, then the unmodified shipping body
    runs. Overriding an inherited ``@skill`` drops the marker attributes the
    MCP server scans for (``Module.get_skills`` looks for ``__skill__``), so
    the decorator is re-applied with the same capabilities, and the docstring
    -- which *is* the tool description the model sees -- is inherited verbatim
    rather than copied, so it cannot drift from the shipping skill.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._tool_queries: list[str] = []
        self._tool_errors: list[str] = []

    @skill(uses=[CAP_MOVEMENT])
    def navigate_with_text(self, query: str) -> str:
        self._tool_queries.append(query)
        try:
            return _SHIPPING_NAVIGATE_WITH_TEXT(self, query)
        except Exception as exc:
            # Recorded so a failing tool call is scored as `tool_error`
            # instead of masquerading as "the agent chose no goal", then
            # re-raised untouched: the agent must see what it would normally see.
            self._tool_errors.append(f"{type(exc).__name__}: {exc}")
            raise

    navigate_with_text.__doc__ = NavigationSkillContainer.navigate_with_text.__doc__

    @rpc
    def get_queries(self) -> list[str]:
        """Read back the `navigate_with_text` queries seen in this worker, in order."""
        return list(self._tool_queries)

    @rpc
    def get_tool_errors(self) -> list[str]:
        """Read back the exceptions raised by `navigate_with_text`, in order."""
        return list(self._tool_errors)

    @rpc
    def reset_queries(self) -> bool:
        """Drop the recorded queries and errors so one instance can serve several questions."""
        self._tool_queries = []
        self._tool_errors = []
        return True


class TimedRunnerConfig(RunnerConfig):
    """`AgentTestRunner` config widened with the runner's two internal waits.

    Defaults match the values hard-coded in the stock runner.
    """

    idle_timeout: float = 60.0
    subscription_timeout: float = 5.0


class TimedAgentTestRunner(AgentTestRunner):
    """`AgentTestRunner` whose internal waits are reachable from the blueprint.

    A live eval turn is a real LLM round trip plus a CLIP-backed memory query,
    which can outlast the stock runner's 60 s idle wait. That literal (and the
    5 s subscription wait) live inside ``_thread_loop`` and the stock ``Config``
    carries only ``messages``, so raising only the fixture's own outer wait
    would still trip the inner one. This subclass exists solely to make both
    waits configurable; the loop is otherwise the shipping loop.
    """

    config: TimedRunnerConfig

    def _thread_loop(self) -> None:
        if not self._subscription_ready.wait(self.config.subscription_timeout):
            raise TimeoutError(
                f"Timed out waiting for subscription to be ready "
                f"(subscription_timeout={self.config.subscription_timeout}s)."
            )

        for message in self.config.messages:
            self._idle_event.clear()
            self.agent_spec.add_message(message)
            if not self._idle_event.wait(self.config.idle_timeout):
                raise TimeoutError(
                    f"Timed out waiting for message to be processed "
                    f"(idle_timeout={self.config.idle_timeout}s)."
                )

        self.finished.publish(True)
