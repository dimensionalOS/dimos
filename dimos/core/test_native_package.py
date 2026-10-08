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

"""Package source builds are lazy, isolated, recoverable and serialized."""

from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

from pydantic import ValidationError
import pytest

from dimos.core import native_package
from dimos.core.native_module import NativeModule, NativeModuleConfig


@pytest.fixture
def module_factory():
    modules = []

    def create(**config):
        module = NativeModule(**config)
        modules.append(module)
        return module

    yield create
    for module in reversed(modules):
        module.stop()


@pytest.fixture
def package_source(tmp_path, monkeypatch):
    package = tmp_path / "native_fixture"
    source = package / "native"
    source.mkdir(parents=True)
    (package / "__init__.py").write_text("")
    (source / "input.txt").write_text("first")
    (source / "build.py").write_text(
        "from pathlib import Path\n"
        "import os\n"
        "counter = Path(os.environ['COUNTER'])\n"
        "with counter.open('a') as stream: stream.write('build\\n')\n"
        "Path('bin').mkdir(exist_ok=True)\n"
        "artifact = Path('bin/probe')\n"
        "artifact.write_text(Path('input.txt').read_text())\n"
        "artifact.chmod(0o755)\n"
        "if Path(os.environ['FAIL_FLAG']).exists(): raise RuntimeError('requested failure')\n"
    )
    monkeypatch.setattr(native_package, "files", lambda name: package)
    monkeypatch.setattr(native_package, "CACHE_DIR", tmp_path / "cache")
    config = {
        "source_package": "native_fixture",
        "source_dir": "native",
        "executable": "bin/probe",
        "build_command": f"{shlex.quote(sys.executable)} build.py",
        "extra_env": {"COUNTER": str(tmp_path / "count"), "FAIL_FLAG": str(tmp_path / "fail")},
    }
    return source, config


def test_package_source_is_lazy_and_reused_then_invalidated(package_source, module_factory):
    source, config = package_source
    module = module_factory(**config)
    counter = Path(config["extra_env"]["COUNTER"])
    assert not counter.exists()
    assert "source_package" not in module.config.to_config_dict()
    assert "--source_package" not in module.config.to_cli_args()

    module._prepare_native()
    first = Path(module._executable)
    assert first.read_text() == "first"
    assert not (source / "bin").exists()
    module_factory(**config)._prepare_native()
    assert counter.read_text() == "build\n"

    (source / "input.txt").write_text("second")
    changed = module_factory(**config)
    changed._prepare_native()
    assert Path(changed._executable).read_text() == "second"
    assert changed._executable != str(first)
    assert counter.read_text() == "build\nbuild\n"


def test_concurrent_preparation_builds_once(package_source, module_factory):
    _, config = package_source
    modules = [module_factory(**config), module_factory(**config)]
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = [pool.submit(module._prepare_native) for module in modules]
        for result in results:
            result.result(timeout=20)
    assert modules[0]._executable == modules[1]._executable
    assert Path(config["extra_env"]["COUNTER"]).read_text() == "build\n"


def test_failed_artifact_is_not_reused_and_force_rebuilds(package_source, module_factory):
    _, config = package_source
    failure = Path(config["extra_env"]["FAIL_FLAG"])
    failure.touch()
    with pytest.raises(RuntimeError, match="Build command failed"):
        module_factory(**config)._prepare_native()
    failure.unlink()
    recovered = module_factory(**config)
    recovered._prepare_native()
    module_factory(**config)._prepare_native()
    assert Path(config["extra_env"]["COUNTER"]).read_text() == "build\nbuild\n"
    forced = module_factory(**config, auto_build=True)
    forced._prepare_native()
    assert forced._executable == recovered._executable
    assert Path(config["extra_env"]["COUNTER"]).read_text() == "build\nbuild\nbuild\n"


def test_recipe_and_environment_changes_invalidate(package_source, module_factory):
    _, config = package_source
    first = module_factory(**config)
    first._prepare_native()
    changed = module_factory(**{**config, "build_command": config["build_command"] + " # changed"})
    changed._prepare_native()
    environment = {**config["extra_env"], "RUSTFLAGS": "-C opt-level=1"}
    third = module_factory(**{**config, "extra_env": environment})
    third._prepare_native()
    assert len({first._executable, changed._executable, third._executable}) == 3


@pytest.mark.parametrize(
    "changes",
    [
        {"source_package": ""},
        {"source_package": "../package"},
        {"source_dir": None},
        {"source_dir": "../native"},
        {"build_command": None},
        {"executable": "/bin/probe"},
        {"executable": "../probe"},
    ],
)
def test_invalid_package_source_contract_is_rejected(package_source, changes):
    _, config = package_source
    with pytest.raises(ValidationError):
        NativeModuleConfig(**{**config, **changes})


def test_symlink_cannot_escape_package_source(package_source, tmp_path, module_factory):
    source, config = package_source
    outside = tmp_path / "outside"
    outside.write_text("not packaged")
    (source / "linked").symlink_to(outside)
    with pytest.raises(ValueError, match="symlinks"):
        module_factory(**config)._prepare_native()


def test_independent_processes_share_one_completed_build(package_source, tmp_path):
    source, config = package_source
    env = {
        **os.environ,
        "PYTHONPATH": str(source.parent.parent)
        + os.pathsep
        + str(Path(__file__).resolve().parents[2]),
        "XDG_CACHE_HOME": str(tmp_path / "process-cache"),
    }
    script = (
        "import json, sys; from dimos.core.native_module import NativeModule; "
        "module = NativeModule(**json.loads(sys.argv[1])); "
        "\ntry: module._prepare_native()\nfinally: module.stop()\n"
    )
    processes = []
    try:
        for _ in range(2):
            processes.append(
                subprocess.Popen(
                    [sys.executable, "-c", script, json.dumps(config)],
                    cwd=tmp_path,
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
            )
        for process in processes:
            stdout, stderr = process.communicate(timeout=30)
            assert process.returncode == 0, stdout + stderr
    finally:
        for process in processes:
            if process.poll() is None:
                process.kill()
                process.communicate(timeout=10)
    assert Path(config["extra_env"]["COUNTER"]).read_text() == "build\n"
