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

"""Paged, multi-fidelity memory layer for agent conversation history.

The memory package turns an unbounded ``list[BaseMessage]`` into a
multi-resolution :class:`PageTable` that the agent draws from each turn.
On every turn the :class:`MemoryEngine` assembles a prompt under a token
budget; older pages are served at lower fidelity (thumbnail, structured
JSON or 1-line pointer) and can be rehydrated on demand via the
``get_artefact`` tool.

Public re-exports are added incrementally as each submodule lands; see
:mod:`dimos.agents.memory.pages`, :mod:`dimos.agents.memory.tokens`,
:mod:`dimos.agents.memory.budget`, :mod:`dimos.agents.memory.faults`,
:mod:`dimos.agents.memory.page_table`, :mod:`dimos.agents.memory.ingestion`,
:mod:`dimos.agents.memory.selector`, :mod:`dimos.agents.memory.artefact_tool`
and :mod:`dimos.agents.memory.engine`.
"""

__all__: list[str] = []
