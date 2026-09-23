# Copyright 2026 Dimensional Inc.
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

"""The agent's conversation, shown in the viewer: what was asked, every tool call and
its result, and the reply.

From `andrew/feat/vr_demo`, where the chat panel was written. Mixed into
MemoryWorldModule rather than living in `module.py` because that file is at the repo's
75 KB per-file ceiling with under a hundred bytes to spare; `answers.py`,
`visual_answers.py`, `replay_serving.py` and `world_cache.py` all exist for the same
reason.

The rows come from `dimos.agents.utils.chat_entries`, which is also what the human CLI
prints. That is the point of reading them from the shared helper rather than reshaping
`BaseMessage` here: the two views of one conversation cannot drift into disagreeing
about what the agent did.

This is a ONE-WAY VIEW plus a way in. A question goes out on `human_input` and the
answer comes back on `agent` whenever the agent gets to it -- nothing here waits for a
reply, and nothing correlates one to the other. `answers.py`'s `/ask` is the blocking
HTTP path that scripts and the tour use, and it still exists; this is what a person
typing into the panel gets, and it shows the work rather than only the conclusion.
"""

from __future__ import annotations

from collections import OrderedDict, deque
import json
import subprocess
import threading
from typing import TYPE_CHECKING, Any

from langchain_core.messages.base import BaseMessage
from reactivex.disposable import Disposable

from dimos.agents.utils import ChatEntry, chat_entries
from dimos.core.stream import In, Out
from dimos.teleop.memory_world.messages import encode_text
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Rows kept for a viewer that connects mid-conversation. A transcript is text; the cost
# of holding several hundred rows is nothing beside one thumbnail.
CHAT_HISTORY = 400

# ---- python tool calls, said in words ----------------------------------------------
# A tool whose argument is a program puts a screenful of source in the transcript, and the
# one thing a reader wants from it -- what it actually DID -- is the hardest thing to see
# in it. So a fast model names the calls it makes and the panel shows that, with the source
# behind a disclosure for anyone who wants it.
SUMMARY_MODEL = "claude-haiku-4-5-20251001"
# Shelled out to the `claude` CLI rather than the API: there is no ANTHROPIC_API_KEY on
# this machine, and Jeff's call (2026-09-23) was that shelling out is fine here.
SUMMARY_COMMAND = ("claude", "-p")
SUMMARY_TIMEOUT_S = 30.0
SUMMARY_CHARS = 240
SUMMARY_CACHE = 128
# Argument names that mean "this value is a program", and the tokens that say a value is
# python whatever it is called. Both, because a tool is free to call the field anything and
# a field called `code` is free to hold a shell one-liner.
PYTHON_ARG_KEYS = ("code", "python", "script", "source", "program", "snippet", "expression")
PYTHON_TOKENS = ("import ", "def ", "print(", "return ", "lambda ", "for ", "await ", "= ")
SUMMARY_PROMPT = """Below is the body of one tool call made by an agent. List the main
python functions, methods and libraries it calls, most important first, as a bare
comma-separated list of names -- no prose, no explanation, no code fences, one line, at
most eight names. If it calls nothing recognisable, answer with what it does in at most
eight words.

"""


def python_in_args(args: str) -> tuple[str, str] | None:
    """``(argument name, source)`` of the first argument that is a python program, else None.

    Multi-line and token-bearing, or named like a program and token-bearing. A one-line
    `"a fire extinguisher"` must never read as code, which is why a token alone is not
    enough and a NAME alone is not either.
    """
    try:
        parsed = json.loads(args or "{}")
    except (TypeError, ValueError):
        return None
    if not isinstance(parsed, dict):
        return None
    for key, value in parsed.items():
        if not isinstance(value, str) or len(value) < 24:
            continue
        tokens = sum(token in value for token in PYTHON_TOKENS)
        if not tokens:
            continue
        if str(key).lower() in PYTHON_ARG_KEYS or ("\n" in value and tokens >= 2):
            return str(key), value
    return None


def summarise_python(code: str) -> str | None:
    """The calls *code* makes, named by a fast model. None when it cannot be had.

    Never raises: a missing CLI, a timeout or a non-zero exit all mean the panel shows the
    source as it always did. The summary is a nicety and must not be able to break a chat.
    """
    try:
        done = subprocess.run(
            [*SUMMARY_COMMAND, SUMMARY_PROMPT + code, "--model", SUMMARY_MODEL],
            capture_output=True,
            text=True,
            timeout=SUMMARY_TIMEOUT_S,
        )
    except (OSError, subprocess.SubprocessError) as error:
        logger.info("no python summary (%s): the source will be shown instead", error)
        return None
    if done.returncode != 0:
        logger.info("no python summary (exit %d): %s", done.returncode, done.stderr[-200:])
        return None
    # First non-empty line only: the model is asked for one line and a preamble is the
    # commonest way that goes wrong.
    for line in done.stdout.splitlines():
        said = line.strip().strip("`").strip()
        if said:
            return said[:SUMMARY_CHARS]
    return None


class WorldChat:
    """Needs, from the module: ``_clients_lock``, ``_broadcast``, and the ``agent``,
    ``agent_idle`` and ``human_input`` streams."""

    # The agent's conversation, the same streams the human CLI is on: its messages come
    # in, a viewer's typed questions go out. Declared on the MIXIN, which the framework
    # sees -- `get_type_hints` resolves the whole MRO -- and which keeps `module.py` off
    # the 75 KB ceiling it is already pressed against.
    agent: In[BaseMessage]
    agent_idle: In[bool]
    human_input: Out[str]

    _clients_lock: Any
    _chat_history: deque[ChatEntry]
    _agent_is_idle: bool

    if TYPE_CHECKING:

        def _broadcast(self, message: bytes | str) -> None: ...

    def _init_chat(self) -> None:
        self._chat_history = deque(maxlen=CHAT_HISTORY)
        # One summary per distinct program, and a bound on the memory: an agent in a loop
        # runs the same code again and again, and paying a model for each is silly.
        self._summaries: OrderedDict[str, str] = OrderedDict()
        # Idle until told otherwise: the panel's "thinking…" must not be the first thing
        # a viewer sees on a server whose agent has never been asked anything.
        self._agent_is_idle = True

    def _watch_the_agent(self) -> list[Disposable]:
        """Subscriptions for the module to register, for whichever streams are connected.

        Guarded per stream because `memory-world-module` runs this same module with no
        agent beside it, and subscribing to a stream with no transport raises
        `AttributeError: 'NoneType' has no attribute 'subscribe'` -- a traceback on every
        start of a blueprint that is working exactly as intended.
        """
        watching = []
        if self.agent.transport is not None:
            watching.append(Disposable(self.agent.subscribe(self._on_agent_message)))
        if self.agent_idle.transport is not None:
            watching.append(Disposable(self.agent_idle.subscribe(self._on_agent_idle)))
        return watching

    def _chat_state(self) -> tuple[list[ChatEntry], bool]:
        """The transcript and the agent's state, for a viewer that just connected."""
        with self._clients_lock:
            return list(self._chat_history), self._agent_is_idle

    def _ask_in_chat(self, text: str) -> bool:
        """Put a viewer's typed question to the agent. False when there is no agent.

        The question is NOT echoed into the transcript here. It arrives back on `agent`
        as a HumanMessage like any other, and echoing it too would show it twice -- once
        immediately and once when the agent actually received it.
        """
        if self.human_input.transport is None:
            return False
        with self._clients_lock:
            self._agent_is_idle = False
        self.human_input.publish(text)
        return True

    def _on_agent_message(self, msg: BaseMessage) -> None:
        entries = chat_entries(msg)
        with self._clients_lock:
            self._chat_history.extend(entries)
        for entry in entries:
            found = (
                python_in_args(str(entry.get("args") or ""))
                if entry.get("role") == "tool_call"
                else None
            )
            if found is not None:
                # The row goes out NOW and the summary follows it. Waiting for the model
                # would hold the whole transcript behind a nicety -- and the row is the
                # proof the agent is working, which is exactly what a person watching a
                # slow query is looking for.
                entry["python"] = found[0]  # type: ignore[typeddict-unknown-key]
                self._broadcast(encode_text("chat", **entry))
                self._summarise_in_the_background(entry, found[1])
                continue
            self._broadcast(encode_text("chat", **entry))

    def _summarise_in_the_background(self, entry: ChatEntry, code: str) -> None:
        """Name the calls *code* makes, then tell every viewer, including later ones."""
        call_id = entry.get("call_id")
        cached = self._summaries.get(code)
        if cached is not None:
            self._publish_summary(entry, call_id, cached)
            return

        def work() -> None:
            said = summarise_python(code)
            if not said:
                return
            with self._clients_lock:
                self._summaries[code] = said
                while len(self._summaries) > SUMMARY_CACHE:
                    self._summaries.popitem(last=False)
            self._publish_summary(entry, call_id, said)

        threading.Thread(target=work, daemon=True, name="MemoryWorldChatSummary").start()

    def _publish_summary(self, entry: ChatEntry, call_id: Any, said: str) -> None:
        """Attach a summary to the row a viewer already has, and to the stored one.

        The stored entry is mutated so a viewer connecting AFTERWARDS gets the summary
        with the transcript rather than the raw program -- the row and the history are the
        same dict, so this is one write.
        """
        with self._clients_lock:
            entry["summary"] = said  # type: ignore[typeddict-unknown-key]
        self._broadcast(encode_text("chat_summary", call_id=call_id, summary=said))

    def _on_agent_idle(self, idle: bool) -> None:
        with self._clients_lock:
            self._agent_is_idle = bool(idle)
        self._broadcast(encode_text("agent_idle", idle=bool(idle)))
