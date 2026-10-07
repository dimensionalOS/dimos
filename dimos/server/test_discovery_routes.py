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

"""The discovery, docs, extras and jobs routes, with a discovery cache filled in by hand, a fake checkout (docs,
pyproject.toml, uv.lock) and a fake uv."""

from __future__ import annotations

import asyncio
from collections.abc import Iterator
import json
import os
from pathlib import Path
import sys
import textwrap
import time
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.server import desktop, docs, events, extras
from dimos.server.app import ServerState, create_app
from dimos.server.discovery import Discovery
from dimos.server.jobs import Jobs, MissingForJobError, failure_lines
from dimos.server.uploads import Uploads

PYPROJECT = textwrap.dedent(
    """
    [project]
    name = "dimos"
    version = "0.0.14"
    [project.optional-dependencies]
    sim = ["mujoco>=3", "pygame>=2.6"]
    unitree-dds = ["unitree-sdk2py-dimos>=1.0.2", "cyclonedds>=0.10.5"]
    cuda = ["cupy-cuda12x==13.6.0; sys_platform == 'nonexistent'"]
    all = ["dimos[sim,unitree-dds]"]
    """
)
UV_LOCK = textwrap.dedent(
    """
    version = 1
    [[package]]
    name = "dimos"
    version = "0.0.14"
    dependencies = [{ name = "numpy" }]
    [package.optional-dependencies]
    sim = [{ name = "mujoco" }, { name = "pygame" }]
    unitree-dds = [{ name = "unitree-sdk2py-dimos" }, { name = "cyclonedds" }]
    [[package]]
    name = "mujoco"
    version = "3.3.4"
    dependencies = [{ name = "glfw" }]
    wheels = [
        { url = "https://x/mujoco-3.3.4-cp312-cp312-macosx_11_0_arm64.whl", size = 1000 },
        { url = "https://x/mujoco-3.3.4-cp312-cp312-manylinux_2_17_x86_64.whl", size = 2000 },
    ]
    [[package]]
    name = "numpy"
    version = "2.0"
    [[package]]
    name = "glfw"
    version = "2.0"
    wheels = [{ url = "https://x/glfw-2.0-py2.py3-none-any.whl", size = 30 }]
    [[package]]
    name = "pygame"
    version = "2.6.1"
    sdist = { url = "https://x/pygame-2.6.1.tar.gz", size = 500 }
    """
)
PROBE = {
    "python": "/venv/bin/python",
    "environment": {
        "platform_system": "Darwin",
        "platform_machine": "arm64",
        "python_version": "3.12",
        "sys_platform": "darwin",
    },
    "packages": {"pygame": "2.6.1", "cyclonedds": "0.10.5", "unitree-sdk2py-dimos": "1.0.2"},
    "dimos_requires": [],
}


@pytest.fixture
def repo(checkout: Path) -> Path:
    """The fake checkout with a pyproject.toml, a uv.lock, an mkdocs.yml and a docs tree."""
    (checkout / "pyproject.toml").write_text(PYPROJECT)
    (checkout / "uv.lock").write_text(UV_LOCK)
    (checkout / "mkdocs.yml").write_text(
        "site_name: X\nsite_url: https://docs.example.org/\nrepo_url: https://github.com/o/r\n"
        "markdown_extensions:\n  - pymdownx.emoji:\n      emoji_index: !!python/name:material.extensions.emoji.twemoji\n"
    )
    pages = {
        "index.md": "# Home\n",
        "usage/configuration.md": "# Configuration\n",
        "usage/blueprints.md": "# Blueprints\n",
        "installation/index.md": "# Install\n",
        "capabilities/arms/adding_a_custom_arm.md": textwrap.dedent(
            """\
            # How to Integrate a New Arm

            See [config](/docs/usage/configuration.md#flags), [blueprints](../../usage/blueprints.md),
            ![diagram](assets/a.png), [code](/dimos/robot/x.py), [web](https://example.com) and [here](#top).

            | a | b |
            |---|---|
            | 1 | 2 |

            <script>alert(1)</script>
            """
        ),
    }
    for name, text in pages.items():
        (checkout / "docs" / name).parent.mkdir(parents=True, exist_ok=True)
        (checkout / "docs" / name).write_text(text)
    return checkout


def module(name: str, *config: dict[str, Any]) -> dict[str, Any]:
    return {
        "name": name,
        "class": f"pkg.{name.replace('-', '_')}.{name.title().replace('-', '')}",
        "doc": "",
        "inputs": [{"name": "cmd_vel", "type": "msgs.Twist"}],
        "outputs": [{"name": "image", "type": "msgs.Image"}],
        "skills": [],
        "config": list(config),
    }


def record(name: str, robot: str | None, *modules: str, **extra: Any) -> dict[str, Any]:
    return {
        "name": name,
        "ref": f"dimos.robot.{robot}.x:{name}",
        "builtin": True,
        "robot": robot,
        "importable": True,
        "optional_dependency": False,
        "import_error": None,
        "import_traceback": None,
        "missing_module": None,
        "modules": [
            {
                "name": m,
                "class": module(m)["class"],
                "module": m,
                "streams": [
                    {"name": "image", "type": "msgs.Image", "direction": "out", "topic": "/image"}
                ],
            }
            for m in modules
        ],
        **extra,
    }


FIELD = {
    "name": "ip",
    "type": "str",
    "default": "192.168.12.1",
    "description": "The robot's IP",
    "required": False,
    "base": False,
    "enum": None,
    "json_compatible": True,
    "reason": None,
}


@pytest.fixture
def filled(repo: Path, server_home: Path) -> Discovery:
    found = Discovery(repo, lambda _: None, cache_dir=server_home / "cache")
    found.current_key = {"digest": "k1"}
    found.data.update(
        key={"digest": "k1"},
        complete=True,
        names={
            "blueprints": ["unitree-go2-basic", "unitree-g1-sdk", "xarm-basic"],
            "modules": ["go2-connection"],
        },
        blueprints={
            "unitree-go2-basic": record(
                "unitree-go2-basic", "go2", "go2-connection", "rerun-bridge"
            ),
            "unitree-g1-sdk": record(
                "unitree-g1-sdk",
                "g1",
                importable=False,
                optional_dependency=True,
                import_error="ModuleNotFoundError: No module named 'unitree_sdk2py'",
                missing_module="unitree_sdk2py",
            ),
            "xarm-basic": record("xarm-basic", "xarm", "xarm-driver", "rerun-bridge"),
        },
        modules={
            m["class"]: m
            for m in (
                module("go2-connection", FIELD),
                module("rerun-bridge"),
                module("xarm-driver"),
            )
        },
    )
    found.status["state"] = "done"
    return found


@pytest.fixture
def client(
    repo: Path, server_home: Path, fake_worker: list[str], filled: Discovery
) -> Iterator[TestClient]:
    bus = events.Bus()
    published: list[tuple[str, dict[str, Any]]] = []
    bus.publishers.append(lambda key, payload: published.append((key, payload)))
    uploads = Uploads(repo, bus, None, server_home / "uploads.log", worker=fake_worker)
    state = ServerState(dimos_dir=repo, bus=bus, uploads=uploads, discovery=filled)
    app = create_app(state, background=False)
    app.state.published = published
    with TestClient(app) as client:
        yield client


def test_discovery_status_and_blueprints(client: TestClient) -> None:
    status = client.get("/dimos/discovery").json()
    assert status["state"] == "done" and status["key"] == "k1" and not status["stale"]
    assert (status["blueprints_total"], status["importable"], status["not_importable"]) == (3, 2, 1)
    found = client.get("/dimos/discovery/blueprints").json()["blueprints"]
    by_name = {b["name"]: b for b in found}
    assert by_name["unitree-g1-sdk"]["suggested_extras"] == ["unitree-dds"]
    assert by_name["unitree-go2-basic"]["modules"][0]["streams"][0]["topic"] == "/image"
    listed = {b["name"]: b for b in client.get("/dimos/blueprints").json()["blueprints"]}
    assert listed["unitree-go2-basic"]["importable"] is True
    unscanned = [b for name, b in listed.items() if name not in by_name]
    assert unscanned and all(b["importable"] is None for b in unscanned)  # not in the (fake) cache
    refreshed = client.post("/dimos/discovery/refresh", json={"full": True})
    assert refreshed.status_code == 200
    assert client.app.state.server.discovery.requested == ("requested", True)  # type: ignore[attr-defined]


def test_modules_config_and_message_types(client: TestClient) -> None:
    modules = {m["name"]: m for m in client.get("/dimos/modules").json()["modules"]}
    assert modules["rerun-bridge"]["blueprint_count"] == 2
    assert modules["rerun-bridge"]["robots"] == ["go2", "xarm"]
    assert "config" not in modules["go2-connection"]
    answer = client.get("/dimos/modules/go2-connection/config").json()
    assert answer == {
        "module": "go2-connection",
        "class": modules["go2-connection"]["class"],
        "fields": [FIELD],
        "error": None,
    }
    by_class = client.get("/dimos/modules/" + modules["go2-connection"]["class"] + "/config")
    assert by_class.json()["module"] == "go2-connection"
    types = {t["type"]: t for t in client.get("/dimos/message-types").json()["types"]}
    assert types["msgs.Image"]["publishers"] == ["go2-connection", "rerun-bridge", "xarm-driver"]


def test_unknown_module_is_404(client: TestClient, monkeypatch: pytest.MonkeyPatch) -> None:
    discovery = client.app.state.server.discovery  # type: ignore[attr-defined]

    async def nothing(name: str) -> None:
        return None

    monkeypatch.setattr(discovery, "module_now", nothing)
    missing = client.get("/dimos/modules/nope/config")
    assert missing.status_code == 404 and "nope" in missing.json()["error"]


def test_robot_modules(client: TestClient) -> None:
    go2 = client.get("/dimos/robots/go2/modules").json()
    assert [m["name"] for m in go2["modules"]] == ["go2-connection", "rerun-bridge"]
    assert go2["modules"][1]["score"] == 0 and go2["robots_total"] == 2
    assert "ln(" in go2["formula"]
    g1 = client.get("/dimos/robots/g1/modules").json()
    assert (g1["blueprints"], g1["blueprints_importable"], g1["modules"]) == (1, 0, [])
    missing = client.get("/dimos/robots/spot/modules")
    assert missing.status_code == 404 and "g1, go2, xarm" in missing.json()["error"]


def test_docs(client: TestClient) -> None:
    guide = client.get("/dimos/docs/custom-robot").json()
    assert guide["title"] == "How to Integrate a New Arm"
    assert guide["source_path"] == "docs/capabilities/arms/adding_a_custom_arm.md"
    assert guide["url"] == "https://docs.example.org/capabilities/arms/adding_a_custom_arm/"
    markdown = guide["markdown"]
    assert "(https://docs.example.org/usage/configuration/#flags)" in markdown
    assert "(https://docs.example.org/usage/blueprints/)" in markdown
    assert "(https://docs.example.org/capabilities/arms/assets/a.png)" in markdown
    assert "(https://github.com/o/r/blob/main/dimos/robot/x.py)" in markdown
    assert "(https://example.com)" in markdown and "(#top)" in markdown
    assert "<table>" in guide["html"] and "<script>" not in guide["html"]
    links = client.get("/dimos/docs/links").json()
    assert (
        links["site"] == "https://docs.example.org/" and links["repo"] == "https://github.com/o/r"
    )
    assert links["configure_robot"] == "https://docs.example.org/usage/configuration/"
    assert links["installation"] == "https://docs.example.org/installation/"
    assert links["custom_robot"] == guide["url"] and links["modules"] is None


def test_every_desktop_link_is_a_real_page() -> None:
    """Renaming or moving a page Desktop links to fails here, not in Desktop."""
    pages = docs.link_pages(DIMOS_PROJECT_ROOT)
    assert set(pages) == {*docs.LINKS, "custom_robot"}
    for name, page in pages.items():
        assert page is not None and page.is_file(), f"no docs page for Desktop's {name} link"
    links = docs.links(DIMOS_PROJECT_ROOT)
    assert links["site"] and links["repo"] and all(links[name] for name in pages)
    guide = docs.custom_robot(DIMOS_PROJECT_ROOT)
    assert guide is not None and guide["markdown"] and guide["url"] == links["custom_robot"]


async def fake_probe(*_: Any) -> dict[str, Any]:
    return PROBE


def test_extras_list(client: TestClient, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(client.app.state.server.discovery, "child_answer", fake_probe)  # type: ignore[attr-defined]
    answer = client.get("/dimos/extras").json()
    assert answer["mode"] == "checkout" and answer["python"] == "/venv/bin/python"
    found = {e["name"]: e for e in answer["extras"]}
    assert found["unitree-dds"]["installed"] and found["unitree-dds"]["download_bytes"] == 0
    assert not found["sim"]["installed"] and found["sim"]["missing"] == ["mujoco"]
    assert (
        found["sim"]["download_bytes"] == 1030
    )  # the arm64 mac wheel and glfw's; pygame is installed
    assert found["cuda"] == {**found["cuda"], "installed": True, "applicable": False}
    assert found["all"]["includes"] == ["sim", "unitree-dds"] and found["all"]["missing"] == [
        "mujoco"
    ]


FAKE_UV = textwrap.dedent(
    """\
    #!{python}
    import sys, time
    print("Resolved 10 packages")
    if "--extra" in sys.argv and sys.argv[sys.argv.index("--extra") + 1] == "sim":
        print("  \\u00d7 Failed to build `mujoco==3.3.4`")
        print("error: no wheel for this platform")
        sys.exit(2)
    if "--extra" in sys.argv and sys.argv[sys.argv.index("--extra") + 1] == "unitree-dds":
        time.sleep(30)
    print("Installed 1 package")
    """
)


def wait_done(client: TestClient, job: str) -> dict[str, Any]:
    for _ in range(200):
        log = client.get(f"/dimos/jobs/{job}/log").json()
        if log["done"]:
            return log  # type: ignore[no-any-return]
        time.sleep(0.05)
    raise AssertionError(f"job {job} never finished")


def test_extras_install_goes_to_desktops_shell_tool(
    client: TestClient, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    uv = tmp_path / "uv"
    uv.write_text(FAKE_UV.format(python=sys.executable))
    uv.chmod(0o755)
    monkeypatch.setattr(extras, "find_uv", lambda: str(uv))
    server = client.app.state.server  # type: ignore[attr-defined]
    monkeypatch.setattr(server.discovery, "child_answer", fake_probe)
    asked: list[tuple[str, list[dict[str, Any]], str | None]] = []
    finish = asyncio.Event()

    async def request_shell(
        title: str, message: str, commands: list[dict[str, Any]], app: str | None = None
    ) -> str:
        asked.append((title, commands, app))
        return "sh-1"

    async def wait_shell(session: str) -> dict[str, Any]:
        await finish.wait()
        return {"status": "succeeded", "commands": []}

    monkeypatch.setattr(desktop, "request_shell", request_shell)
    monkeypatch.setattr(desktop, "wait_shell", wait_shell)
    started = client.post("/dimos/extras/install", json={"extras": ["all"]}).json()
    assert (started["shell"], started["job"]) == ("sh-1", None)
    title, commands, app = asked[0]
    assert app == "launcher" and "all" in title
    # one command, with a note for the user, run in the checkout's venv
    assert commands == [
        {
            "run": f"{uv} sync --locked --inexact --no-progress --extra all",
            "note": "Install the all extra with uv",
            "cwd": str(server.dimos_dir),
            "env": {"VIRTUAL_ENV": str(server.dimos_dir / ".venv")},
        }
    ]
    busy = client.post("/dimos/extras/install", json={"extras": ["sim"]})
    assert busy.status_code == 409 and "sh-1" in busy.json()["error"]


def test_extras_shell_commands_get_cyclonedds_first(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (tmp_path / "pyproject.toml").write_text('[project]\nname = "dimos"\n')
    (tmp_path / "uv.lock").write_text("")
    monkeypatch.delenv("CYCLONEDDS_HOME", raising=False)
    monkeypatch.setattr(extras, "find_nix", lambda: "/nix/bin/nix")
    nix_build, dds, sim = extras.shell_commands(
        tmp_path, ["dds", "sim"], "/v/python", None, "uv", True
    )
    assert nix_build["run"].startswith(
        "/nix/bin/nix --extra-experimental-features 'nix-command flakes' build"
    )
    # one command per extra, each with the library's path
    assert dds["run"].startswith('export CYCLONEDDS_HOME="$(cd ')
    assert dds["run"].endswith("uv sync --locked --inexact --no-progress --extra dds")
    assert sim["run"].startswith('export CYCLONEDDS_HOME="$(cd ')
    assert sim["run"].endswith("--extra sim") and sim["note"] == "Install the sim extra with uv"
    monkeypatch.setattr(extras, "find_nix", lambda: None)
    monkeypatch.setattr(extras, "BREWED_CYCLONEDDS", (tmp_path / "no-brew",))
    with pytest.raises(MissingForJobError):
        extras.shell_commands(tmp_path, ["dds"], "/v/python", None, "uv", True)


def test_a_library_install_gets_one_command_per_extra(tmp_path: Path) -> None:
    perception, cuda = extras.shell_commands(
        tmp_path, ["perception", "cuda"], "/v/python", "0.0.14", "uv", False
    )
    # each its own, with the CUDA torch build the set asked for
    assert perception["run"] == (
        "uv pip install --no-progress --python /v/python --torch-backend cu128 'dimos[perception]==0.0.14'"
    )
    assert cuda["run"].endswith("'dimos[cuda]==0.0.14'")


def test_extras_that_provide_a_missing_module(repo: Path) -> None:
    providers = extras.providing_extras(repo, PROBE["environment"])  # type: ignore[arg-type]
    # by its package's name (`all` includes unitree-dds: the smaller one is enough)
    assert providers("unitree_sdk2py") == ["unitree-dds"]
    assert providers("mujoco") == ["sim"]
    # a dependency of an extra's package (uv.lock)
    assert providers("glfw") == ["sim"]
    # dimos needs it without extras, or nothing has it
    assert providers("numpy") == [] and providers("pyzed") == [] and providers(None) == []


def test_extras_install_is_a_job_without_desktop(
    client: TestClient, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    uv = tmp_path / "uv"
    uv.write_text(FAKE_UV.format(python=sys.executable))
    uv.chmod(0o755)
    monkeypatch.setattr(extras, "find_uv", lambda: str(uv))
    server = client.app.state.server  # type: ignore[attr-defined]
    monkeypatch.setattr(server.discovery, "child_answer", fake_probe)
    assert client.post("/dimos/extras/install", json={"extras": ["nope"]}).status_code == 400
    assert client.post("/dimos/extras/install", json={"extras": []}).status_code == 400

    started = client.post("/dimos/extras/install", json={"extras": ["all"]}).json()
    assert started["command"] == [
        str(uv),
        "sync",
        "--locked",
        "--inexact",
        "--no-progress",
        "--extra",
        "all",
    ]
    log = wait_done(client, started["job"])
    assert log["ok"] and log["lines"][-1] == "Installed 1 package" and log["error"] is None
    assert log["lines"][0].startswith("$ ") and log["next"] == len(log["lines"])
    assert (
        client.get(f"/dimos/jobs/{started['job']}/log?after=2").json()["lines"] == log["lines"][2:]
    )
    assert server.discovery.requested == ("extras installed", False)
    published = client.app.state.published  # type: ignore[attr-defined]
    keys = {key for key, _ in published}
    assert keys == {f"jobs/{started['job']}"}
    assert [p["n"] for _, p in published if p["type"] == "line"] == list(range(len(log["lines"])))
    assert published[-1][1] == {
        "type": "done",
        "ok": True,
        "error": None,
        "failure": [],
        "code": None,
        "lines": len(log["lines"]),
    }

    failed = wait_done(
        client, client.post("/dimos/extras/install", json={"extras": ["sim"]}).json()["job"]
    )
    # the exit code decides; the summary is the output's tail, not lines picked by their wording
    assert not failed["ok"] and failed["error"] == "Install extras: sim failed (exit 2)"
    assert (
        failed["failure"] == failed["lines"]  # fewer than 15
    )

    slow = client.post("/dimos/extras/install", json={"extras": ["unitree-dds"]}).json()["job"]
    busy = client.post("/dimos/extras/install", json={"extras": ["sim"]})
    assert busy.status_code == 409 and slow in busy.json()["error"]
    client.delete(f"/dimos/jobs/{slow}")
    cancelled = wait_done(client, slow)
    assert not cancelled["ok"] and cancelled["error"] == "cancelled"
    assert {j["job"] for j in client.get("/dimos/jobs").json()["jobs"]} >= {started["job"], slow}
    assert client.get("/dimos/jobs/nope/log").status_code == 404


CYCLONEDDS_LOCK = """
[[package]]
name = "cyclonedds"
version = "0.10.5"
sdist = { url = "https://x/cyclonedds-0.10.5.tar.gz", size = 228410 }
wheels = [{ url = "https://x/cyclonedds-0.10.5-cp310-cp310-macosx_11_0_arm64.whl", size = 1000 }]
"""
MAC_312 = {"platform_system": "Darwin", "platform_machine": "arm64", "python_version": "3.12"}


def test_cyclonedds_is_built_where_it_has_no_wheel(tmp_path: Path) -> None:
    (tmp_path / "uv.lock").write_text(CYCLONEDDS_LOCK)
    adding = [
        {"name": "unitree-dds", "missing": ["cyclonedds", "mcap"]},
        {"name": "sim", "missing": []},
    ]
    assert extras.builds_cyclonedds(tmp_path, adding, ["unitree-dds"], MAC_312)
    # python 3.10 has a wheel; an extra that doesn't add it doesn't build it
    assert not extras.builds_cyclonedds(
        tmp_path, adding, ["unitree-dds"], {**MAC_312, "python_version": "3.10"}
    )
    assert not extras.builds_cyclonedds(tmp_path, adding, ["sim"], MAC_312)


def test_nixpkgs_comes_from_flake_lock_alone(tmp_path: Path) -> None:
    assert extras.nixpkgs_ref(tmp_path) == "nixpkgs"
    lock = {
        "root": "root",
        "nodes": {
            "root": {"inputs": {"nixpkgs": "nixpkgs_2"}},
            "nixpkgs_2": {
                "locked": {"type": "github", "owner": "NixOS", "repo": "nixpkgs", "rev": "abc"}
            },
        },
    }
    (tmp_path / "flake.lock").write_text(json.dumps(lock))
    assert extras.nixpkgs_ref(tmp_path) == "github:NixOS/nixpkgs/abc"


def test_cyclonedds_from_env_then_nix_else_a_code(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    built = tmp_path / "store" / "cyclonedds"
    (built / "lib").mkdir(parents=True)
    ran: list[list[str]] = []

    async def nix_builds(command: list[str]) -> int:
        ran.append(command)
        link = Path(command[command.index("--out-link") + 1])
        link.parent.mkdir(parents=True, exist_ok=True)
        link.symlink_to(built)
        return 0

    async def nothing(command: list[str]) -> int:
        raise AssertionError(f"ran {command}")

    monkeypatch.setenv("CYCLONEDDS_HOME", str(built))
    env = asyncio.run(extras.prepare_cyclonedds(tmp_path, nothing))
    assert env["CYCLONEDDS_HOME"] == str(built)

    monkeypatch.delenv("CYCLONEDDS_HOME")
    monkeypatch.setattr(extras, "find_nix", lambda: "nix")
    env = asyncio.run(extras.prepare_cyclonedds(tmp_path, nix_builds))
    assert env["CYCLONEDDS_HOME"] == str(built.resolve())
    assert env["CMAKE_PREFIX_PATH"].split(os.pathsep)[0] == str(built.resolve())
    assert ran[0][-1] == "nixpkgs#cyclonedds" and "build" in ran[0]
    # the out-link lives in the venv: a garbage collection keeps what the built package links
    assert ran[0][ran[0].index("--out-link") + 1] == str(tmp_path / ".venv" / "cyclonedds")

    monkeypatch.setattr(extras, "find_nix", lambda: None)
    monkeypatch.setattr(extras, "BREWED_CYCLONEDDS", (tmp_path / "no-brew",))
    with pytest.raises(MissingForJobError) as missing:
        asyncio.run(extras.prepare_cyclonedds(tmp_path / "other", nothing))
    assert missing.value.code == "cyclonedds_missing" and "brew install cyclonedds" in str(
        missing.value
    )


def test_a_failed_preparation_ends_the_job_with_its_code(tmp_path: Path) -> None:
    published: list[tuple[str, dict[str, Any]]] = []

    async def missing(*_: Any) -> dict[str, str]:
        raise MissingForJobError("cyclonedds_missing", "no CycloneDDS")

    async def go() -> Any:
        jobs = Jobs(lambda _: None, lambda key, payload: published.append((key, payload)))
        job = jobs.start("Install extras: dds", "extras", ["false"], tmp_path, prepare=missing)
        while not job.done:
            await asyncio.sleep(0.01)
        return job

    job = asyncio.run(go())
    assert (job.ok, job.code, job.error) == (False, "cyclonedds_missing", "no CycloneDDS")
    # the command never ran
    assert job.lines == ["no CycloneDDS"] and job.log()["code"] == "cyclonedds_missing"
    assert published[-1][1]["code"] == "cyclonedds_missing"


def test_failure_is_the_last_lines() -> None:
    assert failure_lines(["a", "", "b"]) == ["a", "b"]
    assert failure_lines([str(n) for n in range(40)]) == [str(n) for n in range(25, 40)]


def test_library_install_command(tmp_path: Path) -> None:
    command = extras.install_command(tmp_path, ["cuda", "sim"], "/v/bin/python", "0.0.14", "uv")
    assert command == [
        "uv",
        "pip",
        "install",
        "--no-progress",
        "--python",
        "/v/bin/python",
        "--torch-backend",
        "cu128",
        "dimos[cuda,sim]==0.0.14",
    ]


def test_library_extras_come_from_metadata(tmp_path: Path) -> None:
    requires = [
        "numpy",
        "mujoco>=3; extra == 'sim'",
        "cupy; sys_platform == 'linux' and extra == 'cuda'",
        'dimos[sim]; extra == "all"',
    ]
    probe = {
        "environment": PROBE["environment"],
        "dimos_requires": requires,
        "dimos_extras": ["sim", "cuda", "all"],
    }
    assert extras.declared(tmp_path, probe) == {
        "sim": ["mujoco>=3; extra == 'sim'"],
        "cuda": [],  # linux only: not on this (mac) machine
        "all": ['dimos[sim]; extra == "all"'],
    }
    listed = {
        e["name"]: e for e in extras.status(tmp_path, {**probe, "packages": {"mujoco": "3.3.4"}})
    }
    assert listed["sim"]["installed"] and listed["all"]["includes"] == ["sim"]
    assert listed["cuda"]["applicable"] is False


def test_a_job_line_goes_on_its_own_zenoh_key() -> None:
    from dimos.server.zenoh_events import Publisher

    put: list[tuple[str, bytes]] = []
    Publisher("ns/x", lambda key, payload: put.append((key, payload))).under(
        "jobs/j1", {"type": "line", "n": 0, "line": "hi"}
    )
    assert put == [("ns/x/dimos/jobs/j1", b'{"type": "line", "n": 0, "line": "hi"}')]


def test_robots_come_from_robots_json(client: TestClient, repo: Path) -> None:
    doc = {
        "robots": {
            "go2": {"dirs": ["dimos/robot/go2"], "blueprints": {}},
            "arm": {"dirs": [], "blueprints": {"xarm-basic": {}}},
        }
    }
    (repo / "dimos" / "server").mkdir(parents=True, exist_ok=True)
    (repo / "dimos" / "server" / "robots.json").write_text(__import__("json").dumps(doc))
    found = {
        b["name"]: b["robot"]
        for b in client.get("/dimos/discovery/blueprints").json()["blueprints"]
    }
    assert found == {"unitree-go2-basic": "go2", "unitree-g1-sdk": None, "xarm-basic": "arm"}
    assert client.get("/dimos/robots/arm/modules").json()["modules"][0]["name"] == "xarm-driver"
    assert client.get("/dimos/robots/xarm/modules").status_code == 404


def test_an_account_carries_the_scopes_the_cloud_sends(check_model: Any) -> None:
    """The cloud answers `scopes` as a string ("data"); a list-only model turned every logged-in account into a 500."""
    from dimos.server.models import Account

    for scopes in ("data", ["data"], None):
        account = {
            "loggedIn": True,
            "email": "a@b.c",
            "scopes": scopes,
            "source": "stored",
            "cloudUrl": "x",
            "error": None,
        }
        check_model(Account, account, "the account")
