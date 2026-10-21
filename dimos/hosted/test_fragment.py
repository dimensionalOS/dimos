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

import pickle

import pytest

from dimos.core.module import Module, ModuleConfig
from dimos.hosted.fragment import HostFragment, PythonFragmentPayload


class PayloadConfig(ModuleConfig):
    label: str = "default"


class PayloadModule(Module):
    config: PayloadConfig


def test_fragment_round_trips_the_blueprint_and_the_roots_values() -> None:
    blueprint = PayloadModule.blueprint()
    fragment = HostFragment.create(
        run_id="run-1",
        host_id="host-1",
        revision="revision-1",
        payload=PythonFragmentPayload(
            blueprint=blueprint, overrides={"payloadmodule": {"label": "configured"}}
        ),
    )

    payload = fragment.load_payload()

    assert [a.module for a in payload.blueprint.blueprints] == [PayloadModule]
    assert payload.overrides == {"payloadmodule": {"label": "configured"}}


def test_fragment_rejects_a_foreign_payload() -> None:
    fragment = HostFragment("run-1", "host-1", "revision-1", pickle.dumps({"not": "a payload"}))
    with pytest.raises(TypeError, match="PythonFragmentPayload"):
        fragment.load_payload()
