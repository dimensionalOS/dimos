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

from dimos.deps.catalog import Catalog, CatalogError, catalog_names, default_catalog
from dimos.deps.selectors import SelectorInput

DATA = {
    "version": 2,
    "defaults": {
        "simulation": "",
        "local_relay": False,
        "relay_url": None,
        "unitree_connection_type": "webrtc",
    },
    "scenarios": {"mujoco": ["eq", "simulation", "mujoco"]},
    "rules": {
        "relay": {
            "when": ["any", ["truthy", "local_relay"], ["truthy", "relay_url"]],
            "extras": {"web": ["web.relay_bridge.relay_bridge_module"]},
            "tools": ["deno"],
        }
    },
    "registries": {
        "adapter": {
            "mock": {},
            "xarm": {"extras": {"control": ["hardware.manipulators.xarm.adapter"]}},
        },
        "task": {"trajectory": {}, "g1_groot_wbc": {"backends": ["onnxruntime"]}},
        "connection": {
            "webrtc": {"extras": {"unitree": ["robot.unitree.connection"]}},
            "mujoco": {
                "extras": {"sim": ["robot.unitree.mujoco_connection"]},
                "backends": ["onnxruntime"],
            },
        },
        "simulation": {"mujoco": {"extras": {"sim": ["simulation.engines.mujoco_sim_module"]}}},
    },
    "backends": {"onnxruntime": {"cpu": ["cpu"], "cuda": ["cuda"]}},
    "blueprints": {
        "go2": {
            "extras": {"unitree": ["robot.unitree.connection"], "web": ["web.vis"]},
            "selectors": {"g": {"unitree_connection_type": "connection"}},
            "variants": {
                "mujoco": {"extras": {"sim": ["simulation"]}, "backends": ["onnxruntime"]}
            },
        },
        "coordinator-mock": {
            "selectors": {"controlcoordinator": {"hardware": "adapter", "tasks": "task"}}
        },
        "arm": {"selectors": {"manipulation": {"hardware": "adapter"}}},
        "core-only": {},
    },
    "modules": {"zed-camera": {"system": ["pyzed"], "native": ["zed_native"]}},
}


@pytest.fixture
def catalog() -> Catalog:
    return Catalog(DATA)


def test_names_and_entries(catalog: Catalog) -> None:
    assert catalog.names == {"go2", "coordinator-mock", "arm", "core-only", "zed-camera"}
    assert catalog.entry("core-only") == {}
    assert catalog.entry("nope") is None
    assert catalog.selector_fields() == {"hardware", "tasks"}


def test_plan_union_and_defaults(catalog: Catalog) -> None:
    plan = catalog.plan_for(["go2", "zed-camera", "core-only"])
    assert plan.extras == {"unitree", "web"} and plan.complete
    assert plan.reasons["web"] == {"web.vis"}
    assert plan.system == {"pyzed"} and plan.native == {"zed_native"}
    assert not plan.backends and not plan.tools and plan.external == ()


def test_variants_and_rules_activate_from_config(catalog: Catalog) -> None:
    plan = catalog.plan_for(["go2"], {"simulation": "mujoco", "relay_url": "http://x"})
    assert plan.extras == {"unitree", "web", "sim"}
    assert plan.reasons["web"] == {"web.vis", "web.relay_bridge.relay_bridge_module"}
    assert plan.backends == {"onnxruntime": ""}
    assert plan.tools == {"deno"}


def test_global_selector_resolves_through_the_registry(catalog: Catalog) -> None:
    plan = catalog.plan_for(["go2"], {"unitree_connection_type": "mujoco"})
    assert plan.extras == {"unitree", "web", "sim"} and plan.complete
    assert plan.reasons["sim"] == {"robot.unitree.mujoco_connection"}
    assert plan.backends == {"onnxruntime": ""}
    plan = catalog.plan_for(["go2"], {"unitree_connection_type": "genesis"})
    assert plan.extras == {"unitree", "web"}
    assert plan.incomplete == (
        "unitree_connection_type='genesis' selects a connection the planner does not know: "
        "'genesis' (known: mujoco, webrtc)",
    )


def test_module_inputs_resolve_adapters_and_tasks(catalog: Catalog) -> None:
    hardware = SelectorInput(
        "--controlcoordinator.hardware",
        "controlcoordinator",
        "hardware",
        '[{"adapter_type": "xarm"}]',
    )
    tasks = SelectorInput(
        "config file section 'controlcoordinator'",
        "controlcoordinator",
        "tasks",
        [{"type": "trajectory"}, {"type": "g1_groot_wbc"}],
    )
    plan = catalog.plan_for(["coordinator-mock"], inputs=[hardware, tasks])
    assert plan.complete and plan.extras == {"control"}
    assert plan.backends == {"onnxruntime": ""}
    relative = SelectorInput("--hardware", None, "hardware", '[{"adapter_type": "xarm"}]')
    assert catalog.plan_for(["coordinator-mock", "go2"], inputs=[relative]).extras == {
        "control",
        "unitree",
        "web",
    }


def test_inputs_the_planner_cannot_attribute_make_the_plan_incomplete(catalog: Catalog) -> None:
    def reasons(names: list[str], *inputs: SelectorInput) -> tuple[str, ...]:
        return catalog.plan_for(names, inputs=inputs).incomplete

    assert reasons(
        ["coordinator-mock"],
        SelectorInput("--left.hardware", "left", "hardware", "[]"),
    ) == (
        "--left.hardware addresses a module instance the planner does not know (instances "
        "with a 'hardware' selector: controlcoordinator)",
    )
    assert reasons(
        ["coordinator-mock", "arm"], SelectorInput("--hardware", None, "hardware", "[]")
    ) == ("--hardware is ambiguous (controlcoordinator, manipulation); use --<instance>.hardware",)
    assert reasons(["go2"], SelectorInput("--hardware", None, "hardware", "[]")) == (
        "--hardware names a field no requested blueprint selects implementations with",
    )
    assert (
        reasons(
            ["coordinator-mock"],
            SelectorInput(
                "--controlcoordinator.hardware", "controlcoordinator", "hardware", "[{}]"
            ),
        )
        == ()
    )  # the adapter type defaults to mock
    assert reasons(
        ["coordinator-mock"],
        SelectorInput("--controlcoordinator.hardware", "controlcoordinator", "hardware", "oops"),
    ) == ("--controlcoordinator.hardware: expected a JSON list",)
    assert reasons(
        ["coordinator-mock"],
        SelectorInput("CONTROLCOORDINATOR__TASKS", "controlcoordinator", "tasks", '[{"a": 1}]'),
    ) == ("CONTROLCOORDINATOR__TASKS: an item has no 'type'",)
    assert reasons(
        ["coordinator-mock"],
        SelectorInput(
            "--controlcoordinator.hardware",
            "controlcoordinator",
            "hardware",
            '[{"adapter_type": "xarm7"}]',
        ),
    ) == (
        "--controlcoordinator.hardware selects a adapter the planner does not know: 'xarm7' "
        "(known: mock, xarm)",
    )


def test_backend_resolution(catalog: Catalog) -> None:
    plan = catalog.plan_for(["go2"], {"simulation": "mujoco"}, accelerator="cuda")
    assert "cuda" in plan.extras and plan.backends == {"onnxruntime": "cuda"}
    assert plan.reasons["cuda"] == {"backend:onnxruntime"}
    assert plan.extras_argument == "cuda,sim,unitree,web"


def test_external_and_unknown_names(catalog: Catalog) -> None:
    plan = catalog.plan_for(["go2", "my-stack.teleop"])
    assert plan.external == ("my-stack.teleop",) and not plan.complete
    assert plan.incomplete == (
        "external blueprint 'my-stack.teleop' publishes no dependency metadata (entry point "
        "group 'dimos.catalog'); its requirements are unknown",
    )
    with pytest.raises(CatalogError, match="no entry for 'missing'"):
        catalog.plan_for(["missing"])


def test_version_check() -> None:
    with pytest.raises(CatalogError):
        Catalog({"version": 1})


def test_shipped_catalog_plans_the_reviewed_stacks() -> None:
    assert "unitree-go2" in catalog_names()
    catalog = default_catalog()
    assert "unitree" in catalog.plan_for(["unitree-go2"]).extras
    assert "perception" in catalog.plan_for(["unitree-go2-detection"]).extras
    assert "sim" in catalog.plan_for(["unitree-g1-agentic-sim"]).extras
    go2_sim = catalog.plan_for(["unitree-go2"], {"unitree_connection_type": "mujoco"})
    assert "sim" in go2_sim.extras and go2_sim.backends == {"onnxruntime": ""}
    xarm = SelectorInput(
        "--controlcoordinator.hardware",
        "controlcoordinator",
        "hardware",
        '[{"adapter_type": "xarm"}]',
    )
    assert "control" in catalog.plan_for(["coordinator-mock"], inputs=[xarm]).extras
    assert catalog.plan_for(["coordinator-mock"]).complete
