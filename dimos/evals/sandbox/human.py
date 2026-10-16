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

"""Pretend humans for sandbox cases: generator functions that answer each agent turn.

See :class:`dimos.evals.sandbox.case.Conversation`. A case names one in ``human``, e.g.
``FunctionRef(function="dimos.evals.sandbox.human:until_done", params={"times": 10})``.
"""

from __future__ import annotations

from collections.abc import Iterator

from dimos.evals.sandbox.case import Conversation


def until_done(
    conversation: Conversation,
    *,
    message: str = "You have not called done yet. The task only counts as done once you "
    "call done with valid arguments.",
    times: int = 3,
) -> Iterator[str]:
    """Each time the agent stops without done, *message*, at most *times* times."""
    for _ in range(times):
        if conversation.called_done:
            return
        yield message
