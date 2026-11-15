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
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out, TransportSpec
from dimos.core.transport import ZenohTransport
from dimos.hosted.fragment import RemoteModuleReference
from dimos.hosted.fragment_compiler import compile_fragments, resolve_hosted_assignments
from dimos.hosted.protocol import HostDescriptor, run_module_rpc_name
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.String import String


class SourceModule(Module):
    messages: Out[String]


class SinkModule(Module):
    messages: In[String]


class StringChannelConsumer(Module):
    ch: In[String]


class BoolChannelProducer(Module):
    ch: Out[Bool]


class ProviderModule(Module):
    pass


class ConsumerModule(Module):
    provider: ProviderModule


def _host(
    host_id: str,
    *,
    name: str | None = None,
    tags: frozenset[str] = frozenset(),
    active_run_ids: tuple[str, ...] = (),
    revision: str = "revision-1",
) -> HostDescriptor:
    return HostDescriptor(
        host_id=host_id,
        name=name or host_id,
        tags=dict.fromkeys(tags, ""),
        revision=revision,
        active_run_ids=active_run_ids,
    )


def _compile(assignments: dict[str, str]):
    blueprint = autoconnect(SourceModule.blueprint(), SinkModule.blueprint())
    return compile_fragments(
        blueprint,
        assignments,
        run_id="run-1",
        revision="revision-1",
    )


def test_compiler_splits_modules_and_scopes_every_fragment_to_the_run() -> None:
    fragments = _compile(
        {
            "sourcemodule": "host-a",
            "sinkmodule": "host-b",
        }
    )

    source_payload = fragments["host-a"].load_payload()
    sink_payload = fragments["host-b"].load_payload()

    assert {atom.name for atom in source_payload.blueprint.active_blueprints} == {"sourcemodule"}
    assert {atom.name for atom in sink_payload.blueprint.active_blueprints} == {"sinkmodule"}
    for payload in (source_payload, sink_payload):
        # No pinned transport: the Host pins zenoh_domain, which reaches every stream.
        assert ("messages", String) not in payload.blueprint.transport_map
        assert payload.overrides == {}


def test_compiler_keeps_the_blueprints_own_transports_for_local_streams() -> None:
    blueprint = autoconnect(SourceModule.blueprint(), SinkModule.blueprint()).transports(
        {("messages", String): ZenohTransport.spec("dimos/messages", String)}
    )
    fragments = compile_fragments(
        blueprint,
        {"sourcemodule": "host-a", "sinkmodule": "host-a"},
        run_id="run-1",
        revision="revision-1",
    )

    transport = fragments["host-a"].load_payload().blueprint.transport_map["messages", String]
    assert isinstance(transport, TransportSpec) and transport.cls is ZenohTransport


def test_compiler_rejects_cross_host_ch_type_conflict_before_splitting() -> None:
    blueprint = autoconnect(
        StringChannelConsumer.blueprint(),
        BoolChannelProducer.blueprint(),
    )

    with pytest.raises(ValueError, match="'ch' has conflicting types"):
        compile_fragments(
            blueprint,
            {
                "stringchannelconsumer": "host-a",
                "boolchannelproducer": "host-b",
            },
            run_id="run-1",
            revision="revision-1",
        )


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

    fragments = compile_fragments(
        blueprint,
        {
            "providermodule": "host-a",
            "consumermodule": "host-b",
        },
        run_id="run-1",
        revision="revision-1",
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
            rpcs=frozenset(ProviderModule.rpcs),
            rpc_name=rpc_name,
        ),
    )


def test_compiler_keeps_same_host_module_reference_local_with_hosted_rpc_names() -> None:
    blueprint = autoconnect(ProviderModule.blueprint(), ConsumerModule.blueprint())

    fragment = compile_fragments(
        blueprint,
        {
            "providermodule": "host-a",
            "consumermodule": "host-a",
        },
        run_id="run-1",
        revision="revision-1",
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
        SourceModule.blueprint().hosted(tags={"gpu": True}),
        SinkModule.blueprint(),
    )

    assignments = resolve_hosted_assignments(
        blueprint,
        (_host("gpu-1", tags=frozenset({"gpu"})), _host("root")),
        local_host_id="root",
        revision="revision-1",
    )
    fragments = compile_fragments(
        blueprint,
        assignments,
        run_id="run-1",
        revision="revision-1",
        overrides={"sourcemodule": {"a": 1}, "sinkmodule": {"b": 2}, "g": {"n_workers": 1}},
        global_overrides={"viewer": "none"},
    )

    assert set(fragments) == {"root", "gpu-1"}
    gpu_payload = fragments["gpu-1"].load_payload()
    # Each fragment carries its own modules' sections and the shared ones.
    assert gpu_payload.overrides == {"sourcemodule": {"a": 1}, "g": {"n_workers": 1}}
    assert gpu_payload.global_overrides == {"viewer": "none"}
    assert {atom.name for atom in fragments["root"].load_payload().blueprint.active_blueprints} == {
        "sinkmodule"
    }
    assert {
        atom.name for atom in fragments["gpu-1"].load_payload().blueprint.active_blueprints
    } == {"sourcemodule"}


def test_hosted_fragment_is_co_located_on_least_committed_matching_host() -> None:
    blueprint = autoconnect(SourceModule.blueprint(), SinkModule.blueprint()).hosted(
        tags={"gpu": True}
    )

    assignments = resolve_hosted_assignments(
        blueprint,
        (
            _host("host-a", tags=frozenset({"gpu"}), active_run_ids=("other-run",)),
            _host("host-b", tags=frozenset({"gpu"})),
        ),
        local_host_id="root",
        revision="revision-1",
    )

    assert assignments == {
        "sourcemodule": "host-b",
        "sinkmodule": "host-b",
    }


def test_nested_hosted_placements_combine_constraints() -> None:
    source = SourceModule.blueprint().hosted(tags={"gpu": True})
    blueprint = autoconnect(source, SinkModule.blueprint()).hosted(host="compute")

    assignments = resolve_hosted_assignments(
        blueprint,
        (_host("host-compute", name="compute", tags=frozenset({"gpu"})),),
        local_host_id="root",
        revision="revision-1",
    )

    assert assignments == {
        "sourcemodule": "host-compute",
        "sinkmodule": "host-compute",
    }


def test_independent_hosted_units_are_spread_deterministically() -> None:
    blueprint = autoconnect(
        SourceModule.blueprint().hosted(),
        SinkModule.blueprint().hosted(),
    )

    assignments = resolve_hosted_assignments(
        blueprint,
        (_host("host-a"), _host("host-b")),
        local_host_id="root",
        revision="revision-1",
    )

    assert assignments == {
        "sinkmodule": "host-a",
        "sourcemodule": "host-b",
    }


def test_module_references_place_each_side_by_its_own_constraints() -> None:
    blueprint = autoconnect(
        ProviderModule.blueprint(),
        ConsumerModule.blueprint().hosted(host="compute"),
    )

    assignments = resolve_hosted_assignments(
        blueprint,
        (_host("host-compute", name="compute"), _host("root")),
        local_host_id="root",
        revision="revision-1",
    )

    assert assignments == {
        "consumermodule": "host-compute",
        "providermodule": "root",
    }


def test_a_provider_and_consumer_pinned_apart_compile_into_a_remote_reference() -> None:
    blueprint = autoconnect(
        ProviderModule.blueprint().hosted(host="a"),
        ConsumerModule.blueprint().hosted(host="b"),
    )

    assignments = resolve_hosted_assignments(
        blueprint, (_host("a"), _host("b")), local_host_id=None, revision="revision-1"
    )
    fragments = compile_fragments(blueprint, assignments, run_id="run-1", revision="revision-1")

    assert assignments == {"providermodule": "a", "consumermodule": "b"}
    (reference,) = fragments["b"].load_payload().remote_module_references
    assert (reference.provider_host_id, reference.rpc_name) == (
        "a",
        run_module_rpc_name("run-1", "a", "providermodule"),
    )


def test_host_none_keeps_the_whole_hosted_fragment_on_the_root() -> None:
    blueprint = autoconnect(SourceModule.blueprint(), SinkModule.blueprint()).hosted(local=True)

    assignments = resolve_hosted_assignments(
        blueprint,
        (_host("remote"), _host("root")),
        local_host_id="root",
        revision="revision-1",
    )

    assert assignments == {
        "sinkmodule": "root",
        "sourcemodule": "root",
    }


def test_a_local_unit_needs_a_live_local_host_on_this_revision() -> None:
    blueprint = SourceModule.blueprint()
    with pytest.raises(ValueError, match="needs a live local Host"):
        resolve_hosted_assignments(
            blueprint, (_host("remote"),), local_host_id=None, revision="revision-1"
        )
    with pytest.raises(ValueError, match="cannot take sourcemodule: revision is old"):
        resolve_hosted_assignments(
            blueprint,
            (_host("root", revision="old"),),
            local_host_id="root",
            revision="revision-1",
        )


def test_automatic_placement_reports_unsatisfied_constraints() -> None:
    blueprint = SourceModule.blueprint().hosted(tags={"gpu": True})

    with pytest.raises(ValueError, match=r"No Host satisfies placement.*missing tags"):
        resolve_hosted_assignments(
            blueprint,
            (_host("cpu-1", tags=frozenset({"cpu"})),),
            local_host_id="root",
            revision="revision-1",
        )


def test_automatic_placement_rejects_ambiguous_exact_host_name() -> None:
    blueprint = SourceModule.blueprint().hosted(host="robot")

    with pytest.raises(ValueError, match="Exact Host name 'robot'.*is ambiguous"):
        resolve_hosted_assignments(
            blueprint,
            (_host("robot-a", name="robot"), _host("robot-b", name="robot")),
            local_host_id="root",
            revision="revision-1",
        )


class MountedConfig(ModuleConfig):
    mount: str = "SF"


class MountedModule(Module):
    config: MountedConfig


def test_the_roots_explicit_config_survives_the_trip_to_a_host() -> None:
    blueprint = MountedModule.blueprint().hosted(host="host-a")
    parsed = BlueprintConfigParser(blueprint).parse(
        ["--mountedmodule.mount", "ATHENS", "--viewer=none"], environ={}
    )

    fragments = compile_fragments(
        blueprint,
        {"mountedmodule": "host-a"},
        run_id="run-1",
        revision="revision-1",
        overrides=parsed.file_and_cli,
        global_overrides={},
    )

    payload = fragments["host-a"].load_payload()
    assert payload.overrides == {"mountedmodule": {"mount": "ATHENS"}, "g": {"viewer": "none"}}
