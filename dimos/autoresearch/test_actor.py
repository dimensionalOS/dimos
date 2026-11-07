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

"""The actor runner refuses to launch Pi without the provider's key."""

import os
from pathlib import Path
import threading

import pytest
from pytest_mock import MockerFixture

from dimos.autoresearch.actor import run_actor
from dimos.autoresearch.session import Budget, Session


def test_run_actor_names_the_missing_key_before_launching_anything(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    session = Session.create(tmp_path / "s", task="lift the cube", budget=Budget(), model="m")
    adapter = mocker.patch("dimos.autoresearch.actor.PiAdapter")
    mocker.patch.dict(os.environ, {}, clear=True)

    for provider, key in (("anthropic", "ANTHROPIC_API_KEY"), ("google", "GEMINI_API_KEY")):
        with pytest.raises(RuntimeError, match=key):
            run_actor(
                "http://lab/mcp",
                session,
                "m",
                provider,
                timeout_s=1.0,
                run_dir=tmp_path / "actor",
                stop=threading.Event(),
            )

    adapter.assert_not_called()
    assert not (tmp_path / "actor").exists()
