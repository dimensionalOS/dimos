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

from dataclasses import replace

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.module import Module, ModuleConfig
from dimos.hosted.fragment import HostFragment, PythonFragmentPayload


class PayloadConfig(ModuleConfig):
    label: str = "default"


class PayloadModule(Module):
    config: PayloadConfig


def _fragment(label: str) -> HostFragment:
    blueprint = PayloadModule.blueprint()
    config = BlueprintConfigParser(blueprint).parse(
        environ={},
        overrides={"payloadmodule": {"label": label}},
    )
    return HostFragment.create(
        run_id="run-1",
        generation=1,
        host_id="host-1",
        application_name="fragment-test",
        application_revision="revision-1",
        payload=PythonFragmentPayload(blueprint=blueprint, config=config),
    )


def test_fragment_round_trip_preserves_blueprint_config_identity() -> None:
    fragment = _fragment("configured")

    payload = fragment.load_payload()

    payload.config.assert_matches(payload.blueprint)
    assert payload.config.module_kwargs("payloadmodule")["label"] == "configured"


def test_fragment_digest_covers_configuration() -> None:
    first = _fragment("first")
    second = _fragment("second")

    assert first.payload_digest != second.payload_digest


def test_fragment_rejects_tampered_payload_before_deserialization() -> None:
    fragment = _fragment("configured")
    tampered = replace(fragment, payload=fragment.payload + b"tampered")

    with pytest.raises(ValueError, match="digest"):
        tampered.load_payload()
