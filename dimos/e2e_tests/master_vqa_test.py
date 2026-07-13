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

"""Master VQA (visual question answering) e2e test.

Generalizes the single-question pattern in test_dimos_cli_e2e.py's
test_dimos_skills: a natural-language request goes to the in-graph LLM
agent (McpClient), which must pick and call the right registered skill and
return an answer -- the full MCP tool-calling loop over LCM. This test loops
that pattern over a whole bank of questions against one running robot/scene
and writes every {question, answer} pair to a JSON file.

Flow:
  1. Start `unitree-go2-scene-memory-agentic` (McpServer + McpClient +
     ObjectSceneRegistrationModule + SpatialMemory) in mujoco simulation --
     the fullest currently-wired blueprint for wishlist items 2 and 5.
  2. Drive the robot through a short local loop (see LOCAL_LOOP_WAYPOINTS)
     so perception has a few real viewpoints to answer questions about.
  3. Feed each question from fixtures/vqa_questions.txt over /human_input,
     one at a time, and capture the agent's final textual answer from the
     /agent message stream once McpClient reports agent_idle again.
  4. Write the full Q&A transcript to vqa_results.ignore.json next to this
     file (the `.ignore.` infix keeps run output out of git, see
     .gitignore's `**/*.ignore.*`).

Add more questions by editing fixtures/vqa_questions.txt -- see that file's
header for the current wishlist-skill coverage and how to extend it.
"""

from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path
import pickle
import time
from typing import Any

from langchain_core.messages import AIMessage, BaseMessage
import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.simulation.mujoco.direct_cmd_vel_explorer import DirectCmdVelExplorer
from dimos.utils.testing.waiting import wait_until

QUESTIONS_FILE = Path(__file__).parent / "fixtures" / "vqa_questions.txt"
RESULTS_FILE = Path(__file__).parent / "vqa_results.ignore.json"

AGENT_TOPIC = "/agent"
AGENT_IDLE_TOPIC = "/agent_idle"

DEFAULT_ANSWER_TIMEOUT_S = 90.0

# A short, local loop near spawn rather than the full office tour used by
# movement-focused e2e tests (explore_office(): ~14 waypoints spanning the
# whole scene). When mujoco runs slower than real-time that tour can take
# several minutes of wall-clock time; this test only needs a few distinct
# viewpoints for perception to have something to answer questions about.
MUJOCO_START_POS = "-10.75 -6.78"
LOCAL_LOOP_WAYPOINTS = [
    (-9.5, -6.78),
    (-9.5, -8.5),
    (-10.75, -8.5),
    (-10.75, -6.78),
]


def load_questions(path: Path) -> list[str]:
    """Parse the question template: one question per line.

    Blank lines and lines starting with '#' (comment headers) are ignored.
    """
    questions = []
    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        questions.append(line)
    return questions


def decode_agent_messages(raw_messages: list[bytes]) -> list[BaseMessage]:
    """Unpickle a slice of raw /agent LCM payloads (McpClient uses a pickled
    pLCMTransport for this topic) back into LangChain messages."""
    return [pickle.loads(m) for m in raw_messages]


def final_answer(messages: list[BaseMessage]) -> str:
    """The turn's answer: the last AIMessage carrying real text content.

    A single turn's /agent stream includes intermediate AIMessages that only
    carry tool_calls (no content) and ToolMessages with raw tool output; the
    actual answer to show a human is the final AIMessage after any tool
    calls have resolved, so scan from the end for the first one with content.
    """
    for msg in reversed(messages):
        if isinstance(msg, AIMessage) and isinstance(msg.content, str) and msg.content.strip():
            return msg.content
    return ""


def tool_calls_used(messages: list[BaseMessage]) -> list[str]:
    """Names of every tool the agent invoked while answering one question."""
    names = []
    for msg in messages:
        if isinstance(msg, AIMessage):
            for call in msg.tool_calls or []:
                names.append(call["name"])
    return names


def ask(
    lcm_spy: LcmSpy,
    human_input: Callable[[str], None],
    question: str,
    timeout: float = DEFAULT_ANSWER_TIMEOUT_S,
) -> dict[str, Any]:
    """Send one question over /human_input and capture the agent's answer.

    Waits for McpClient's `agent_idle` output to publish `True` again (see
    McpClient._process_message: it publishes False when it starts working a
    queued message and True once the queue drains), then reads back every
    /agent message published during that turn to extract the final answer.
    """
    start_agent = len(lcm_spy.messages.get(AGENT_TOPIC, []))
    start_idle = len(lcm_spy.messages.get(AGENT_IDLE_TOPIC, []))

    t0 = time.monotonic()
    human_input(question)

    def turn_complete() -> bool:
        idle_msgs = lcm_spy.messages.get(AGENT_IDLE_TOPIC, [])[start_idle:]
        return any(pickle.loads(m) is True for m in idle_msgs)

    wait_until(
        turn_complete,
        timeout=timeout,
        message=f"Timed out waiting for an answer to: {question!r}",
    )
    elapsed_s = time.monotonic() - t0

    new_messages = decode_agent_messages(lcm_spy.messages.get(AGENT_TOPIC, [])[start_agent:])

    return {
        "question": question,
        "answer": final_answer(new_messages),
        "tool_calls": tool_calls_used(new_messages),
        "duration_s": round(elapsed_s, 2),
    }


@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.mujoco
@pytest.mark.timeout(900)
def test_master_vqa(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[..., DimosCliCall],
    human_input: Callable[[str], None],
    direct_cmd_vel_explorer: DirectCmdVelExplorer,
) -> None:
    questions = load_questions(QUESTIONS_FILE)
    assert questions, f"No questions found in {QUESTIONS_FILE}"

    lcm_spy.save_topic(AGENT_TOPIC)
    lcm_spy.save_topic(AGENT_IDLE_TOPIC)
    lcm_spy.save_topic("/rpc/McpClient/on_system_modules/res")

    start_blueprint(
        "--mujoco-start-pos",
        MUJOCO_START_POS,
        "run",
        "unitree-go2-scene-memory-agentic",
    )

    lcm_spy.wait_for_saved_topic("/rpc/McpClient/on_system_modules/res", timeout=120.0)
    time.sleep(5)

    # Drive a short local loop so perception gets a few distinct viewpoints
    # before the Q&A pass (see LOCAL_LOOP_WAYPOINTS).
    direct_cmd_vel_explorer.follow_points(LOCAL_LOOP_WAYPOINTS)

    results = [ask(lcm_spy, human_input, question) for question in questions]

    RESULTS_FILE.write_text(json.dumps(results, indent=2))

    unanswered = [r["question"] for r in results if not r["answer"]]
    assert not unanswered, (
        f"{len(unanswered)}/{len(results)} question(s) got no answer: {unanswered}. "
        f"Full transcript: {RESULTS_FILE}"
    )
