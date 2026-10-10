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

import importlib
import os
import signal
import subprocess
import sys

import pytest

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator, stream_name_types
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.rpc_client import RPCClient
from dimos.protocol.rpc.spec import RPCServer
from dimos.robot.external_blueprints import _target_to_blueprint


@pytest.fixture
def external_modules(tmp_path, monkeypatch, mocker):
    mocker.patch.dict("sys.modules")
    monkeypatch.syspath_prepend(str(tmp_path))
    modules = []
    for package_name, direction in (("external_test_a", "Out"), ("external_test_b", "In")):
        package = tmp_path / package_name
        package.mkdir()
        (package / "__init__.py").touch()
        (package / "worker.py").write_text(f"""from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import {direction}
from dimos.experimental.isolated_python.module import IsolatedPythonModule

class WorkerConfig(ModuleConfig):
    speed: float = 1.0

class Worker(Module):
    config: WorkerConfig
    data: {direction}[int]

    @rpc
    def identity(self) -> str:
        return self.config.instance_name

class Child(Worker):
    pass

class IsolatedWorker(IsolatedPythonModule):
    implementation = "{package_name}.worker:Worker"
    project_dir = "fixture/runtime"
""")
        (package / "alias.py").write_text(
            "from .worker import Worker\nfrom dimos.core.module import Module as InternalModule\n"
        )
        modules.append(importlib.import_module(f"{package_name}.worker"))
    return modules


@pytest.fixture
def module_factory():
    modules = []

    def create(cls, **kwargs):
        module = cls(**kwargs)
        modules.append(module)
        return module

    yield create
    for module in reversed(modules):
        module.stop()


def test_external_same_class_names_coexist_without_isolating_streams(external_modules):
    first, second = external_modules
    composition = autoconnect(first.Worker.blueprint(), second.Worker.blueprint())

    assert [atom.name for atom in composition.blueprints] == [
        "external_test_a.worker.Worker",
        "external_test_b.worker.Worker",
    ]
    assert stream_name_types(composition) == {("data", int)}
    assert not composition.remapping_map
    assert all("frame_id_prefix" not in atom.kwargs for atom in composition.blueprints)


def test_external_same_class_last_configuration_wins(external_modules):
    worker = external_modules[0].Worker
    composition = autoconnect(worker.blueprint(speed=1), worker.blueprint(speed=2))

    assert [(atom.name, atom.kwargs) for atom in composition.blueprints] == [
        ("external_test_a.worker.Worker", {"speed": 2})
    ]


def test_explicit_instance_name_overrides_external_default_and_guards_collisions(external_modules):
    first, second = external_modules
    composition = autoconnect(
        first.Worker.blueprint(instance_name="producer"),
        second.Worker.blueprint(instance_name="consumer"),
    )
    assert [atom.name for atom in composition.blueprints] == ["producer", "consumer"]

    with pytest.raises(ValueError, match="Module instance name 'shared' is shared"):
        autoconnect(
            first.Worker.blueprint(instance_name="shared"),
            second.Worker.blueprint(instance_name="shared"),
        )


def test_definition_origin_controls_names_including_subclasses_and_reexports(external_modules):
    first = external_modules[0]
    alias = importlib.import_module("external_test_a.alias")

    class InternalSubclass(first.Worker):
        __module__ = "dimos.test_modules"

    assert first.Child.name == "external_test_a.worker.Child"
    assert InternalSubclass.name == "internalsubclass"
    assert alias.Worker is first.Worker
    assert alias.Worker.name == "external_test_a.worker.Worker"
    assert alias.InternalModule.name == "module"
    assert _target_to_blueprint("external-package.internal", Module).blueprints[0].name == "module"


@pytest.mark.parametrize("origin", ["dimos", "dimos.some_module", "dimos_extensions.worker"])
def test_internal_namespace_boundary_is_exact(origin):
    worker = type("Worker", (Module,), {"__module__": origin})
    assert worker.name == (
        "dimos_extensions.worker.Worker" if origin == "dimos_extensions.worker" else "worker"
    )


def test_script_module_rpc_identity_survives_worker_import(tmp_path):
    script = tmp_path / "worker_script.py"
    script.write_text("""from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import GlobalConfig

class Worker(Module):
    @rpc
    def identity(self):
        return self.config.instance_name

if __name__ == "__main__":
    coordinator = ModuleCoordinator(g=GlobalConfig(n_workers=1, viewer="none"))
    try:
        coordinator.start()
        worker = coordinator.deploy(Worker)
        coordinator.start_all_modules()
        assert Worker.name == worker.remote_name == worker.identity() == "__main__.Worker"
        assert coordinator.list_module_names() == ["__main__.Worker"]
    finally:
        coordinator.stop()
""")
    with (tmp_path / "worker.log").open("w+") as output:
        process = subprocess.Popen(
            [sys.executable, str(script)],
            stdout=output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            return_code = process.wait(timeout=30)
            output.seek(0)
            assert return_code == 0, output.read()
        finally:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)


@pytest.mark.parametrize("source", ["cli", "environment", "overrides"])
def test_external_config_supports_qualified_and_shell_safe_addresses(external_modules, source):
    first, second = external_modules
    parser = BlueprintConfigParser(autoconnect(first.Worker.blueprint(), second.Worker.blueprint()))
    cli = []
    environ = {}
    overrides = {}
    if source == "cli":
        cli = [
            "--external_test_a.worker.Worker.speed",
            "2",
            "--external_test_b_worker_Worker.speed",
            "3",
        ]
    elif source == "environment":
        environ = {
            "EXTERNAL_TEST_A_WORKER_WORKER__SPEED": "2",
            "EXTERNAL_TEST_B_WORKER_WORKER__SPEED": "3",
        }
    else:
        overrides = {
            "external_test_a.worker.Worker": {"speed": 2},
            "external_test_b_worker_Worker": {"speed": 3},
        }

    parsed = parser.parse(cli, environ=environ, overrides=overrides)

    assert parsed.module_kwargs("external_test_a.worker.Worker")["speed"] == 2
    assert parsed.module_kwargs("external_test_b.worker.Worker")["speed"] == 3
    assert parsed.module_kwargs("external_test_a_worker_Worker")["speed"] == 2


def test_config_encoding_collisions_are_rejected(external_modules):
    parser = BlueprintConfigParser(
        autoconnect(
            external_modules[0].Worker.blueprint(),
            Module.blueprint(instance_name="external_test_a_worker_Worker"),
        )
    )
    with pytest.raises(BlueprintConfigError, match="instance-key collision"):
        parser.parse([], environ={})


def test_rpc_defaults_and_explicit_names_reach_actual_module_instances(
    external_modules,
    module_factory,
):
    first = module_factory(external_modules[0].Worker, default_rpc_timeout=2)
    second = module_factory(external_modules[1].Worker, default_rpc_timeout=2)
    named = module_factory(
        external_modules[0].Worker, instance_name="chosen", default_rpc_timeout=2
    )
    internal = module_factory(Module)
    clients = [
        RPCClient.remote(type(first), rpc=first.rpc),
        RPCClient.remote(type(second), rpc=second.rpc),
        RPCClient.remote(type(named), remote_name="chosen", rpc=named.rpc),
    ]
    try:
        assert [client.identity() for client in clients] == [
            "external_test_a.worker.Worker",
            "external_test_b.worker.Worker",
            "chosen",
        ]
        assert internal.config.instance_name is None
        assert RPCClient.remote(Module, rpc=internal.rpc).remote_name == "Module"
        assert first.frame_id == "Worker"
        assert first.config.frame_id_prefix is None
    finally:
        for client in clients:
            client.stop_rpc_client()


def test_rpc_server_default_matches_external_client(external_modules, module_factory, mocker):
    module = module_factory(external_modules[0].Worker)
    server = mocker.Mock(spec=RPCServer)

    RPCServer.serve_module_rpc(server, module)

    handlers = {call.args[1]: call.args[0] for call in server.serve_rpc.call_args_list}
    assert handlers["external_test_a.worker.Worker/identity"]() == "external_test_a.worker.Worker"


def test_coordinator_advertises_external_and_explicit_rpc_identities(external_modules):
    coordinator = ModuleCoordinator(g=GlobalConfig(n_workers=0, viewer="none"))
    try:
        coordinator.start()
        first = coordinator.deploy(external_modules[0].Worker, default_rpc_timeout=2)
        second = coordinator.deploy(external_modules[1].Worker, default_rpc_timeout=2)
        explicit = coordinator.deploy(Module, instance_name="module")

        assert first.identity() == "external_test_a.worker.Worker"
        assert second.identity() == "external_test_b.worker.Worker"
        assert set(coordinator.list_module_names()) == {
            "external_test_a.worker.Worker",
            "external_test_b.worker.Worker",
            "module",
        }
        assert {descriptor.rpc_name for descriptor in coordinator.list_modules()} == {
            "external_test_a.worker.Worker",
            "external_test_b.worker.Worker",
            "module",
        }
        assert explicit.remote_name == "module"
    finally:
        coordinator.stop()


def test_isolated_runtime_keeps_the_qualified_or_explicit_public_identity(
    external_modules,
    module_factory,
    monkeypatch,
):
    monkeypatch.setattr("dimos.experimental.isolated_python.module.short_id", lambda: "run")
    first = module_factory(external_modules[0].IsolatedWorker)
    second = module_factory(external_modules[1].IsolatedWorker)
    explicit = module_factory(external_modules[0].IsolatedWorker, instance_name="chosen")

    assert [module._new_runtime_name() for module in (first, second, explicit)] == [
        "__isolated_python__/external_test_a.worker.IsolatedWorker/run",
        "__isolated_python__/external_test_b.worker.IsolatedWorker/run",
        "__isolated_python__/chosen/run",
    ]
