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

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import TransportSpec, autoconnect
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import ZenohTransport
from dimos.hosted.fragment import run_stream_base_topic, run_stream_key
from dimos.hosted.fragment_compiler import compile_fragments
from dimos.msgs.std_msgs.String import String


class SourceModule(Module):
    messages: Out[String]


class SinkModule(Module):
    messages: In[String]


class ProviderModule(Module):
    pass


class ConsumerModule(Module):
    provider: ProviderModule


def _compile(assignments: dict[str, str]):
    blueprint = autoconnect(SourceModule.blueprint(), SinkModule.blueprint())
    config = BlueprintConfigParser(blueprint).parse(environ={})
    return compile_fragments(
        blueprint,
        config,
        assignments,
        run_id="run-1",
        generation=1,
        application_name="compiler-test",
        application_revision="revision-1",
    )


def test_compiler_splits_modules_and_pins_boundary_stream_to_existing_topic_rule() -> None:
    fragments = _compile(
        {
            "sourcemodule": "host-a",
            "sinkmodule": "host-b",
        }
    )

    source_payload = fragments["host-a"].load_payload()
    sink_payload = fragments["host-b"].load_payload()
    expected_key = run_stream_key("run-1", "messages", String)

    assert {atom.name for atom in source_payload.blueprint.active_blueprints} == {"sourcemodule"}
    assert {atom.name for atom in sink_payload.blueprint.active_blueprints} == {"sinkmodule"}
    assert source_payload.boundary_streams[0].key_expr == expected_key
    assert sink_payload.boundary_streams[0].key_expr == expected_key
    assert source_payload.config.global_config["transport"] == "zenoh"
    assert sink_payload.config.global_config["transport"] == "zenoh"

    transport = source_payload.blueprint.transport_map["messages", String]
    assert isinstance(transport, TransportSpec)
    assert transport.cls is ZenohTransport
    assert transport.args == (run_stream_base_topic("run-1", "messages"), String)


def test_compiler_keeps_same_host_stream_local() -> None:
    fragments = _compile(
        {
            "sourcemodule": "host-a",
            "sinkmodule": "host-a",
        }
    )

    payload = fragments["host-a"].load_payload()

    assert payload.boundary_streams == ()
    assert ("messages", String) not in payload.blueprint.transport_map


@pytest.mark.parametrize(
    ("assignments", "message"),
    [
        ({"sourcemodule": "host-a"}, "missing modules"),
        (
            {
                "sourcemodule": "host-a",
                "sinkmodule": "host-b",
                "unknown": "host-c",
            },
            "unknown modules",
        ),
    ],
)
def test_compiler_rejects_incomplete_placement(
    assignments: dict[str, str],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        _compile(assignments)


def test_compiler_rejects_module_references_across_hosts() -> None:
    blueprint = autoconnect(ProviderModule.blueprint(), ConsumerModule.blueprint())
    config = BlueprintConfigParser(blueprint).parse(environ={})

    with pytest.raises(ValueError, match="crosses Hosts"):
        compile_fragments(
            blueprint,
            config,
            {
                "providermodule": "host-a",
                "consumermodule": "host-b",
            },
            run_id="run-1",
            generation=1,
            application_name="compiler-test",
            application_revision="revision-1",
        )
