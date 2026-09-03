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
from dimos.hosted.fragment import (
    RemoteModuleReference,
    run_module_rpc_name,
    run_stream_base_topic,
    run_stream_key,
)
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


def test_compiler_emits_cross_host_module_reference_and_provider_rpc_name() -> None:
    blueprint = autoconnect(ProviderModule.blueprint(), ConsumerModule.blueprint())
    config = BlueprintConfigParser(blueprint).parse(environ={})

    fragments = compile_fragments(
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

    provider_payload = fragments["host-a"].load_payload()
    consumer_payload = fragments["host-b"].load_payload()
    rpc_name = run_module_rpc_name("run-1", "host-a", "providermodule")

    provider_atom = provider_payload.blueprint.active_blueprints[0]
    assert provider_atom.name == "providermodule"
    assert provider_atom.kwargs["rpc_name"] == rpc_name
    assert provider_payload.remote_module_references == ()
    assert consumer_payload.remote_module_references == (
        RemoteModuleReference(
            consumer_name="consumermodule",
            reference_name="provider",
            provider_name="providermodule",
            provider_host_id="host-a",
            provider_type=ProviderModule,
            rpc_name=rpc_name,
        ),
    )


def test_compiler_keeps_same_host_module_reference_local_with_hosted_rpc_names() -> None:
    blueprint = autoconnect(ProviderModule.blueprint(), ConsumerModule.blueprint())
    config = BlueprintConfigParser(blueprint).parse(environ={})

    fragment = compile_fragments(
        blueprint,
        config,
        {
            "providermodule": "host-a",
            "consumermodule": "host-a",
        },
        run_id="run-1",
        generation=1,
        application_name="compiler-test",
        application_revision="revision-1",
    )["host-a"]

    payload = fragment.load_payload()
    provider = next(
        atom for atom in payload.blueprint.active_blueprints if atom.name == "providermodule"
    )
    consumer = next(
        atom for atom in payload.blueprint.active_blueprints if atom.name == "consumermodule"
    )

    assert payload.remote_module_references == ()
    assert provider.kwargs["rpc_name"] == run_module_rpc_name("run-1", "host-a", "providermodule")
    assert consumer.kwargs["rpc_name"] == run_module_rpc_name("run-1", "host-a", "consumermodule")
