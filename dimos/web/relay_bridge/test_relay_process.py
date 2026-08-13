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

"""RelayProcess cockpit serving and the ensure_cockpit_dist build policy."""

import json
import os
from pathlib import Path
import shutil
import sys
import threading
import time
import urllib.error
import urllib.request

import pytest

from dimos.web.relay_bridge import relay_process
from dimos.web.relay_bridge.locate import (
    WEB_DIR_BUILD_ENV_VAR,
    WEB_DIR_ENV_VAR,
    find_cockpit_dist,
    relay_run_cmd,
)
from dimos.web.relay_bridge.protocol import PROTOCOL_VERSION
from dimos.web.relay_bridge.relay_process import RelayProcess, ensure_cockpit_dist


def _fetch(url: str) -> tuple[int, bytes]:
    try:
        with urllib.request.urlopen(url) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def _make_fake_dist(root: Path) -> Path:
    dist = root / "dist"
    (dist / "assets").mkdir(parents=True)
    (dist / "index.html").write_text("<html><body>fake cockpit index</body></html>")
    (dist / "assets" / "app.js").write_text("console.log('fake');")
    return dist


def test_relay_run_cmd_cockpit_flags() -> None:
    cmd = relay_run_cmd("deno", Path("/web"), "--port", "0")
    assert "--node-modules-dir=none" in cmd
    assert "--allow-read=/web" in cmd
    assert "--cockpit-dir" not in cmd

    cmd = relay_run_cmd("deno", Path("/web"), "--port", "0", cockpit_dir=Path("/elsewhere/dist"))
    assert "--allow-read=/web,/elsewhere/dist" in cmd
    assert cmd[cmd.index("--cockpit-dir") + 1] == "/elsewhere/dist"


def test_relay_run_cmd_resolves_symlinked_dirs(tmp_path: Path) -> None:
    # The relay realpath-checks served files, so --allow-read must be granted
    # on canonical paths or every read would be denied.
    real = tmp_path / "real_web"
    real.mkdir()
    link = tmp_path / "link_web"
    link.symlink_to(real)
    cmd = relay_run_cmd("deno", link)
    assert f"--allow-read={real.resolve()}" in cmd


def test_relay_serves_cockpit_dist(tmp_path: Path) -> None:
    dist = _make_fake_dist(tmp_path)
    with RelayProcess(port=0, cockpit_dir=dist) as info:
        assert info.cockpit is True
        assert info.open_url == f"http://127.0.0.1:{info.http_port}/"

        status, index = _fetch(info.open_url)
        assert status == 200 and b"fake cockpit index" in index
        status, asset = _fetch(f"{info.open_url}assets/app.js")
        assert status == 200 and b"console.log" in asset
        # The debug page still resolves from relay/static/ behind the cockpit.
        status, debug = _fetch(info.debug_url)
        assert status == 200 and b"DimOS relay debug" in debug

        status, body = _fetch(f"{info.open_url}api/info")
        assert status == 200
        api_info = json.loads(body)
        assert api_info["v"] == PROTOCOL_VERSION
        assert api_info["wtUrl"].startswith("https://127.0.0.1:")
        assert api_info["certHash"] == info.cert_hash


def test_relay_without_cockpit_serves_debug(monkeypatch: pytest.MonkeyPatch) -> None:
    # The checkout may have a built dist; force the "not built" path.
    monkeypatch.setattr(relay_process, "find_cockpit_dist", lambda _: None)
    with RelayProcess(port=0) as info:
        assert info.cockpit is False
        assert info.open_url == info.debug_url
        status, index = _fetch(f"http://127.0.0.1:{info.http_port}/")
        assert status == 200 and b"DimOS relay debug" in index


class _FakeBuild:
    """Stands in for _run_build; writes a dist into the --outDir on success."""

    def __init__(self, ok: bool = True) -> None:
        self.ok = ok
        self.calls: list[Path] = []

    def __call__(self, cmd: list[str], cwd: Path, deadline: float, cancel: threading.Event) -> bool:
        assert cmd[-2] == "--outDir"
        self.calls.append(Path(cwd))
        if self.ok:
            out_dir = Path(cmd[-1])
            (out_dir / "assets").mkdir()
            (out_dir / "index.html").write_text("<html><body>fake cockpit index</body></html>")
            (out_dir / "assets" / "app.js").write_text("console.log('fake');")
        return self.ok


@pytest.fixture
def fake_web(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """A fake checkout web/ tree with cockpit sources and no build tools."""
    (tmp_path / "deno.json").write_text("{}")
    (tmp_path / "deno.lock").write_text("lock-v1")
    (tmp_path / "cockpit" / "src").mkdir(parents=True)
    (tmp_path / "cockpit" / "src" / "main.tsx").write_text("code")
    (tmp_path / "cockpit" / "package.json").write_text("{}")
    (tmp_path / "shared").mkdir()
    (tmp_path / "shared" / "protocol.ts").write_text("code")
    monkeypatch.setattr(relay_process, "ensure_deno", lambda: "deno")
    monkeypatch.delenv(WEB_DIR_ENV_VAR, raising=False)
    monkeypatch.delenv(WEB_DIR_BUILD_ENV_VAR, raising=False)
    return tmp_path


def test_ensure_cockpit_dist_builds_when_missing(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    dist = ensure_cockpit_dist(fake_web)
    assert build.calls == [fake_web / "cockpit"]
    assert dist == (fake_web / "cockpit" / "dist").resolve()
    assert (dist / relay_process._STAMP_NAME).is_file()
    # The temp build dir was published (renamed), not left behind.
    assert list((fake_web / "cockpit").glob(".dist-*")) == []


def test_ensure_cockpit_dist_skips_fresh_dist(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_cockpit_dist(fake_web)
    assert ensure_cockpit_dist(fake_web) == (fake_web / "cockpit" / "dist").resolve()
    assert len(build.calls) == 1


def test_ensure_cockpit_dist_rebuilds_on_source_change(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_cockpit_dist(fake_web)
    source = fake_web / "cockpit" / "src" / "main.tsx"
    stat = source.stat()
    source.write_text("edited")
    os.utime(source, (stat.st_atime, stat.st_mtime))  # content, not mtime, decides
    ensure_cockpit_dist(fake_web)
    assert len(build.calls) == 2


def test_ensure_cockpit_dist_rebuilds_on_source_deletion(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    extra = fake_web / "cockpit" / "src" / "Extra.tsx"
    extra.write_text("code")
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_cockpit_dist(fake_web)
    extra.unlink()
    ensure_cockpit_dist(fake_web)
    assert len(build.calls) == 2


def test_ensure_cockpit_dist_rebuilds_on_lockfile_change(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_cockpit_dist(fake_web)
    (fake_web / "deno.lock").write_text("lock-v2")
    ensure_cockpit_dist(fake_web)
    assert len(build.calls) == 2


def test_ensure_cockpit_dist_ignores_test_and_hidden_files(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_cockpit_dist(fake_web)
    (fake_web / "cockpit" / "src" / "App.test.tsx").write_text("test code")
    (fake_web / "shared" / "protocol_test.ts").write_text("test code")
    (fake_web / "cockpit" / ".env.local").write_text("hidden")
    ensure_cockpit_dist(fake_web)
    assert len(build.calls) == 1


def test_ensure_cockpit_dist_failed_build_keeps_served_dist(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(relay_process, "_run_build", _FakeBuild())
    dist = ensure_cockpit_dist(fake_web)
    assert dist is not None
    index_before = (dist / "index.html").read_bytes()

    (fake_web / "cockpit" / "src" / "main.tsx").write_text("edited")
    monkeypatch.setattr(relay_process, "_run_build", _FakeBuild(ok=False))
    assert ensure_cockpit_dist(fake_web) == dist  # stale but present beats nothing
    assert (dist / "index.html").read_bytes() == index_before  # never clobbered

    shutil.rmtree(dist)
    assert ensure_cockpit_dist(fake_web) is None


def test_ensure_cockpit_dist_wheel_shape_never_builds(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # A wheel ships cockpit/dist but no cockpit/src: return the dist as-is.
    dist = _make_fake_dist(tmp_path / "cockpit")
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    assert ensure_cockpit_dist(tmp_path) == dist.resolve()
    assert build.calls == []
    # No lock or stamp writes either: site-packages may be read-only.
    assert not (tmp_path / "cockpit" / ".build.lock").exists()
    # And a wheel without a cockpit at all yields None.
    assert ensure_cockpit_dist(tmp_path / "nowhere") is None


def test_ensure_cockpit_dist_env_tree_requires_opt_in(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # A tree picked via DIMOS_WEB_DIR is served as-is: its build tooling runs
    # with -A, so building needs the explicit acknowledgment.
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    monkeypatch.setenv(WEB_DIR_ENV_VAR, str(fake_web))
    assert ensure_cockpit_dist(fake_web) is None
    assert build.calls == []
    monkeypatch.setenv(WEB_DIR_BUILD_ENV_VAR, "1")
    assert ensure_cockpit_dist(fake_web) == (fake_web / "cockpit" / "dist").resolve()
    assert len(build.calls) == 1


def test_ensure_cockpit_dist_serializes_concurrent_builds(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build = _FakeBuild()
    gate = threading.Lock()
    active = 0
    peak = 0

    def slow_build(cmd: list[str], cwd: Path, deadline: float, cancel: threading.Event) -> bool:
        nonlocal active, peak
        with gate:
            active += 1
            peak = max(peak, active)
        time.sleep(0.3)
        try:
            return build(cmd, cwd, deadline, cancel)
        finally:
            with gate:
                active -= 1

    monkeypatch.setattr(relay_process, "_run_build", slow_build)
    results: list[Path | None] = []
    threads = [
        threading.Thread(target=lambda: results.append(ensure_cockpit_dist(fake_web)))
        for _ in range(2)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=30)
    assert peak == 1  # the flock serialized the builders
    assert build.calls == [fake_web / "cockpit"]  # the loser re-checked and found it fresh
    assert results == [(fake_web / "cockpit" / "dist").resolve()] * 2


def test_run_build_cancel_kills_child_group(tmp_path: Path) -> None:
    marker = tmp_path / "started"
    cmd = [
        sys.executable,
        "-c",
        "import pathlib,sys,time; pathlib.Path(sys.argv[1]).write_text('x'); time.sleep(60)",
        str(marker),
    ]
    cancel = threading.Event()
    result: list[bool] = []
    worker = threading.Thread(
        target=lambda: result.append(
            relay_process._run_build(cmd, tmp_path, time.monotonic() + 60, cancel)
        )
    )
    worker.start()
    try:
        deadline = time.monotonic() + 10
        while not marker.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert marker.exists()  # the child really started
        cancelled_at = time.monotonic()
        cancel.set()
        worker.join(timeout=10)
        assert not worker.is_alive()
        # Bounded by the kill grace, nowhere near the child's 60 s sleep.
        assert time.monotonic() - cancelled_at < relay_process._BUILD_KILL_GRACE_S + 3
        assert result == [False]
    finally:
        cancel.set()
        worker.join(timeout=10)


def test_run_build_deadline_kills_child(tmp_path: Path) -> None:
    cmd = [sys.executable, "-c", "import time; time.sleep(60)"]
    started = time.monotonic()
    ok = relay_process._run_build(cmd, tmp_path, started + 0.3, threading.Event())
    assert ok is False
    assert time.monotonic() - started < relay_process._BUILD_KILL_GRACE_S + 3


def test_find_cockpit_dist_requires_index(tmp_path: Path) -> None:
    assert find_cockpit_dist(tmp_path) is None
    (tmp_path / "cockpit" / "dist").mkdir(parents=True)
    assert find_cockpit_dist(tmp_path) is None
    (tmp_path / "cockpit" / "dist" / "index.html").write_text("x")
    assert find_cockpit_dist(tmp_path) == (tmp_path / "cockpit" / "dist").resolve()


def test_find_cockpit_dist_resolves_symlinks(tmp_path: Path) -> None:
    real = tmp_path / "real_web"
    _make_fake_dist(real / "cockpit")
    link = tmp_path / "link_web"
    link.symlink_to(real)
    assert find_cockpit_dist(link) == (real / "cockpit" / "dist").resolve()
