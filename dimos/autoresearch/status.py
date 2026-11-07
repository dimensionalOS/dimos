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

"""What the lab is doing, as one versioned JSON document on its status stream."""

from __future__ import annotations

import json

from pydantic import BaseModel

from dimos.autoresearch.session import SessionStatus, Split


class AutoresearchStatus(BaseModel):
    """Version 1 of the lab's status. ``state`` follows session.json. ``busy``
    says a trial loop is active, on ``candidate``, ``split`` and ``trial``;
    ``dev_passed`` of ``dev_trials`` are that candidate's dev tallies so far.
    ``error`` is what ended the last loop early, if anything did."""

    ts: float
    state: SessionStatus
    session_dir: str
    busy: bool = False
    candidate: int | None = None
    split: Split | None = None
    trial: int | None = None
    dev_passed: int = 0
    dev_trials: int = 0
    heldout_rate: float | None = None
    error: str | None = None

    def to_json(self) -> str:
        return json.dumps({"schema_version": 1, **self.model_dump(mode="json")}, allow_nan=False)
