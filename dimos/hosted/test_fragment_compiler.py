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
from dimos.hosted.daemon import HostDescriptor, HostState
from dimos.hosted.fragment import (
    RemoteModuleReference,
    run_module_rpc_name,
    run_stream_base_topic,
    run_stream_key,
)
from dimos.hosted.fragment_compiler import compile_fragments, resolve_hosted_assignments
from dimos.msgs.std_msgs.String import String


class SourceModule(Module):
    messages: Out[String]


class SinkModule(Module):
    messages: In[String]


class ProviderModule(Module):
    pass


class ConsumerModule(Module):
    provider: ProviderModule


def _host(
    host_id: str,
    *,
    name: str | None = None,
    tags: frozenset[str] = frozenset(),
    state: HostState = "available",
    active_run_ids: tuple[str, ...] = (),
    revision: str = "revision-1",
) -> HostDescriptor:
    return HostDescriptor(
        host_id=host_id,
        epoch=f"epoch-{host_id}",
        name=name or host_id,
        tags=tags,
        versions={"application_revision": revision},
        state=state,
        active_run_ids=active_run_ids,
    )


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


def test_compiler_automatically_assigns_hosted_and_local_modules() -> None:
    blueprint = autoconnect(
        SourceModule.blueprint().hosted(tags={"gpu"}),
        SinkModule.blueprint(),
    )
    config = BlueprintConfigParser(blueprint).parse(environ={})

    fragments = compile_fragments(
        blueprint,
        config,
        run_id="run-1",
        generation=1,
        application_name="compiler-test",
        application_revision="revision-1",
        hosts=(_host("gpu-1", tags=frozenset({"gpu"})),),
        local_host_id="controller",
    )

    assert set(fragments) == {"controller", "gpu-1"}
    assert {
        atom.name for atom in fragments["controller"].load_payload().blueprint.active_blueprints
    } == {"sinkmodule"}
    assert {
        atom.name for atom in fragments["gpu-1"].load_payload().blueprint.active_blueprints
    } == {"sourcemodule"}


def test_hosted_fragment_is_co_located_on_least_committed_matching_host() -> None:
    blueprint = autoconnect(SourceModule.blueprint(), SinkModule.blueprint()).hosted(tags={"gpu"})

    assignments = resolve_hosted_assignments(
        blueprint,
        (
            _host("host-a", tags=frozenset({"gpu"}), active_run_ids=("other-run",)),
            _host("host-b", tags=frozenset({"gpu"})),
        ),
        local_host_id="controller",
        application_revision="revision-1",
    )

    assert assignments == {
        "sourcemodule": "host-b",
        "sinkmodule": "host-b",
    }


def test_independent_hosted_units_are_spread_deterministically() -> None:
    blueprint = autoconnect(
        SourceModule.blueprint().hosted(),
        SinkModule.blueprint().hosted(),
    )

    assignments = resolve_hosted_assignments(
        blueprint,
        (_host("host-a"), _host("host-b")),
        local_host_id="controller",
        application_revision="revision-1",
    )

    assert assignments == {
        "sinkmodule": "host-a",
        "sourcemodule": "host-b",
    }


def test_module_references_force_automatic_co_location() -> None:
    blueprint = autoconnect(
        ProviderModule.blueprint(),
        ConsumerModule.blueprint().hosted(host="compute"),
    )

    assignments = resolve_hosted_assignments(
        blueprint,
        (_host("host-compute", name="compute"),),
        local_host_id="controller",
        application_revision="revision-1",
    )

    assert assignments == {
        "consumermodule": "host-compute",
        "providermodule": "host-compute",
    }


def test_hosted_local_constraint_keeps_the_whole_fragment_on_controller() -> None:
    blueprint = autoconnect(SourceModule.blueprint(), SinkModule.blueprint()).hosted(local=True)

    assignments = resolve_hosted_assignments(
        blueprint,
        (_host("remote"),),
        local_host_id="controller",
        application_revision="revision-1",
    )

    assert assignments == {
        "sinkmodule": "controller",
        "sourcemodule": "controller",
    }


def test_automatic_placement_reports_unsatisfied_constraints() -> None:
    blueprint = SourceModule.blueprint().hosted(tags={"gpu"})

    with pytest.raises(ValueError, match=r"No Host satisfies placement.*missing tags"):
        resolve_hosted_assignments(
            blueprint,
            (_host("cpu-1", tags=frozenset({"cpu"})),),
            local_host_id="controller",
            application_revision="revision-1",
        )


def test_automatic_placement_rejects_ambiguous_exact_host_name() -> None:
    blueprint = SourceModule.blueprint().hosted(host="robot")

    with pytest.raises(ValueError, match="Exact Host name 'robot'.*is ambiguous"):
        resolve_hosted_assignments(
            blueprint,
            (_host("robot-a", name="robot"), _host("robot-b", name="robot")),
            local_host_id="controller",
            application_revision="revision-1",
        )
