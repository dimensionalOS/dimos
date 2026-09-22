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

from collections import deque
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
            self._broadcast(encode_text("chat", **entry))

    def _on_agent_idle(self, idle: bool) -> None:
        with self._clients_lock:
            self._agent_is_idle = bool(idle)
        self._broadcast(encode_text("agent_idle", idle=bool(idle)))
