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

"""The plans of representative stacks are sufficient to install and validate them.

These prepare multi-gigabyte managed environments (perception, simulation,
agents) from this checkout, so they run on the large self-hosted job only:
``uv run pytest -m self_hosted_large dimos/deps/test_managed_stacks.py``.
A warm uv cache makes each preparation a matter of minutes. Set
``DIMOS_TEST_ENVS_DIR`` to a directory on the same file system as the cache.
"""

from collections.abc import Iterator
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest
from typer.testing import CliRunner

from dimos.cli.dimos import main
from dimos.core.global_config import GlobalConfig, global_config
from dimos.deps import managed
from dimos.deps.catalog import default_catalog
from dimos.deps.managed import detect_source, ensure_environment, environment_key
from dimos.deps.profiles import resolve_profile

pytestmark = [
    pytest.mark.self_hosted_large,
    pytest.mark.skipif(shutil.which("uv") is None, reason="uv is not installed"),
]

STACKS = [
    pytest.param(
        "unitree-go2-detection",
        [],
        {"perception", "unitree"},
        "import ultralytics",
        id="go2-detection",
    ),
    pytest.param(
        "unitree-g1-agentic-sim",
        ["--simulation", "mujoco"],
        {"sim", "perception", "agents"},
        "import mujoco, ultralytics, langchain",
        id="g1-agentic-sim",
    ),
]


@pytest.fixture
def envs_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[Path]:
    configured = os.environ.get("DIMOS_TEST_ENVS_DIR")
    root = (Path(configured) / tmp_path.name if configured else tmp_path) / "envs"
    monkeypatch.setattr(managed, "ENVS_DIR", root)
    yield root
    shutil.rmtree(root, ignore_errors=True)


@pytest.fixture
def restore_global_config() -> Iterator[None]:
    snapshot = global_config.model_dump()
    yield
    global_config.update(**snapshot)


@pytest.mark.parametrize(("name", "flags", "extras", "probe"), STACKS)
def test_prepared_stack_imports_its_packages_and_passes_doctor(
    name: str,
    flags: list[str],
    extras: set[str],
    probe: str,
    envs_dir: Path,
    restore_global_config: None,
) -> None:
    profile, overridden = resolve_profile("linux-x86_64-cpu" if sys.platform == "linux" else None)
    values = GlobalConfig(**dict(zip(flags[::2], flags[1::2], strict=True))).model_dump()
    config = {
        key.lstrip("-").replace("-", "_"): value
        for key, value in zip(flags[::2], flags[1::2], strict=True)
    }
    plan = default_catalog().plan_for(
        [name], GlobalConfig.planning_values(values), accelerator=profile.accelerator
    )
    assert plan.complete and extras <= plan.extras, plan
    key = environment_key(plan, profile, detect_source())
    _stamp, lease = ensure_environment(
        key,
        plan,
        profile,
        overridden_profile=overridden,
        blueprints=(name,),
        global_config=config,
        offline=False,
        echo=lambda message: None,
    )
    with lease:
        completed = subprocess.run(
            [str(key.python_executable), "-c", probe], capture_output=True, text=True
        )
        assert completed.returncode == 0, completed.stderr
        result = CliRunner().invoke(main, [*flags, "doctor", name, "--environment", "managed"])
        assert result.exit_code == 0, result.output
        assert f"Blueprint {name}: satisfied" in result.output
