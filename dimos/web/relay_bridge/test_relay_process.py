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

"""RelayProcess serving and the ensure_web_dist build policy."""

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
    find_sdk_dist,
    relay_run_cmd,
)
from dimos.web.relay_bridge.protocol import PROTOCOL_VERSION
from dimos.web.relay_bridge.relay_process import RelayProcess, ensure_web_dist


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


def _make_fake_sdk_dist(root: Path) -> Path:
    dist = root / "dist"
    dist.mkdir(parents=True)
    (dist / "sdk.js").write_text("// fake sdk bundle")
    return dist


def test_relay_run_cmd_dir_flags() -> None:
    cmd = relay_run_cmd("deno", Path("/web"), "--port", "0")
    assert "--node-modules-dir=none" in cmd
    assert "--allow-read=/web" in cmd
    assert "--cockpit-dir" not in cmd

    cmd = relay_run_cmd("deno", Path("/web"), "--port", "0", cockpit_dir=Path("/elsewhere/dist"))
    assert "--allow-read=/web,/elsewhere/dist" in cmd
    assert cmd[cmd.index("--cockpit-dir") + 1] == "/elsewhere/dist"

    cmd = relay_run_cmd(
        "deno",
        Path("/web"),
        cockpit_dir=Path("/elsewhere/dist"),
        sdk_dir=Path("/sdk/dist"),
        serve_dir=Path("/my/ui"),
    )
    assert "--allow-read=/web,/elsewhere/dist,/sdk/dist,/my/ui" in cmd
    assert cmd[cmd.index("--sdk-dir") + 1] == "/sdk/dist"
    assert cmd[cmd.index("--serve-dir") + 1] == "/my/ui"


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

        status, body = _fetch(f"{info.open_url}api/info")
        assert status == 200
        api_info = json.loads(body)
        assert api_info["v"] == PROTOCOL_VERSION
        assert api_info["wtUrl"].startswith("https://127.0.0.1:")
        assert api_info["certHash"] == info.cert_hash


def test_relay_serves_sdk_and_serve_dir(tmp_path: Path) -> None:
    cockpit_dist = _make_fake_dist(tmp_path / "cockpit")
    sdk_dist = _make_fake_sdk_dist(tmp_path / "sdk")
    user_dir = tmp_path / "my-ui"
    user_dir.mkdir()
    (user_dir / "index.html").write_text("<html><body>fake user index</body></html>")
    (user_dir / "data.txt").write_text("user data")
    (tmp_path / "outside.txt").write_text("must never be served")
    (user_dir / "escape.txt").symlink_to(tmp_path / "outside.txt")

    with RelayProcess(
        port=0, cockpit_dir=cockpit_dist, sdk_dir=sdk_dist, serve_dir=user_dir
    ) as info:
        # The user directory replaces the cockpit at /.
        status, index = _fetch(info.open_url)
        assert status == 200 and b"fake user index" in index
        status, data = _fetch(f"{info.open_url}data.txt")
        assert status == 200 and data == b"user data"

        # /sdk.js and /api/* keep precedence over the user root, and /sdk.js
        # carries the local CORS wildcard plus no-cache.
        with urllib.request.urlopen(f"{info.open_url}sdk.js") as response:
            assert response.headers["Access-Control-Allow-Origin"] == "*"
            assert response.headers["Cache-Control"] == "no-cache"
            assert response.read() == b"// fake sdk bundle"
        status, body = _fetch(f"{info.open_url}api/info")
        assert status == 200 and json.loads(body)["v"] == PROTOCOL_VERSION

        # The traversal and symlink-escape guards apply to the user root.
        status, _ = _fetch(f"http://127.0.0.1:{info.http_port}//etc/passwd")
        assert status == 400
        status, body = _fetch(f"{info.open_url}escape.txt")
        assert status == 400 and b"never be served" not in body


def test_relay_without_cockpit_serves_build_hint(monkeypatch: pytest.MonkeyPatch) -> None:
    # The checkout may have built dists; force the "not built" path.
    monkeypatch.setattr(relay_process, "find_cockpit_dist", lambda _: None)
    monkeypatch.setattr(relay_process, "find_sdk_dist", lambda _: None)
    with RelayProcess(port=0) as info:
        assert info.cockpit is False
        status, index = _fetch(info.open_url)
        assert status == 404 and b"cockpit dist not built" in index
        status, sdk = _fetch(f"{info.open_url}sdk.js")
        assert status == 404 and b"sdk bundle not built" in sdk


class _FakeBuild:
    """Stands in for _run_build; writes a per-package dist into the --outDir.

    `fail` names the package (by directory name) whose build fails; every
    other package builds successfully.
    """

    def __init__(self, fail: str | None = None) -> None:
        self.fail = fail
        self.calls: list[Path] = []

    def __call__(self, cmd: list[str], cwd: Path, deadline: float, cancel: threading.Event) -> bool:
        assert cmd[-2] == "--outDir"
        self.calls.append(Path(cwd))
        if cwd.name == self.fail:
            return False
        out_dir = Path(cmd[-1])
        if cwd.name == "sdk":
            (out_dir / "sdk.js").write_text("// fake sdk bundle")
        else:
            (out_dir / "assets").mkdir()
            (out_dir / "index.html").write_text("<html><body>fake cockpit index</body></html>")
            (out_dir / "assets" / "app.js").write_text("console.log('fake');")
        return True


@pytest.fixture
def fake_web(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """A fake checkout web/ tree with cockpit+sdk sources and no build tools."""
    (tmp_path / "deno.json").write_text("{}")
    (tmp_path / "deno.lock").write_text("lock-v1")
    (tmp_path / "cockpit" / "src").mkdir(parents=True)
    (tmp_path / "cockpit" / "src" / "main.tsx").write_text("code")
    (tmp_path / "cockpit" / "package.json").write_text("{}")
    (tmp_path / "shared").mkdir()
    (tmp_path / "shared" / "protocol.ts").write_text("code")
    (tmp_path / "sdk" / "src").mkdir(parents=True)
    (tmp_path / "sdk" / "src" / "index.ts").write_text("code")
    monkeypatch.setattr(relay_process, "ensure_deno", lambda: "deno")
    monkeypatch.delenv(WEB_DIR_ENV_VAR, raising=False)
    monkeypatch.delenv(WEB_DIR_BUILD_ENV_VAR, raising=False)
    return tmp_path


def _dists(web_dir: Path) -> tuple[Path | None, Path | None]:
    return find_sdk_dist(web_dir), find_cockpit_dist(web_dir)


def test_ensure_web_dist_builds_both_when_missing(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_web_dist(fake_web)
    assert build.calls == [fake_web / "sdk", fake_web / "cockpit"]
    sdk_dist, cockpit_dist = _dists(fake_web)
    assert sdk_dist == (fake_web / "sdk" / "dist").resolve()
    assert cockpit_dist == (fake_web / "cockpit" / "dist").resolve()
    # One shared stamp in both dists; the temp build dirs were published
    # (renamed), not left behind.
    stamps = {relay_process._read_stamp(dist) for dist in (sdk_dist, cockpit_dist)}
    assert len(stamps) == 1 and None not in stamps
    assert list((fake_web / "sdk").glob(".dist-*")) == []
    assert list((fake_web / "cockpit").glob(".dist-*")) == []


def test_ensure_web_dist_skips_fresh_dists(fake_web: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_web_dist(fake_web)
    ensure_web_dist(fake_web)
    assert len(build.calls) == 2


def test_ensure_web_dist_rebuilds_when_one_dist_missing(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_web_dist(fake_web)
    shutil.rmtree(fake_web / "sdk" / "dist")
    ensure_web_dist(fake_web)
    assert len(build.calls) == 4
    assert None not in _dists(fake_web)


def test_ensure_web_dist_rebuilds_on_source_change(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_web_dist(fake_web)
    source = fake_web / "cockpit" / "src" / "main.tsx"
    stat = source.stat()
    source.write_text("edited")
    os.utime(source, (stat.st_atime, stat.st_mtime))  # content, not mtime, decides
    ensure_web_dist(fake_web)
    assert len(build.calls) == 4


def test_ensure_web_dist_rebuilds_on_sdk_change(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # The cockpit bundles the SDK sources, so an sdk/ edit must invalidate
    # the stamp too.
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_web_dist(fake_web)
    source = fake_web / "sdk" / "src" / "index.ts"
    stat = source.stat()
    source.write_text("edited")
    os.utime(source, (stat.st_atime, stat.st_mtime))  # content, not mtime, decides
    ensure_web_dist(fake_web)
    assert len(build.calls) == 4


def test_ensure_web_dist_rebuilds_on_source_deletion(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    extra = fake_web / "cockpit" / "src" / "Extra.tsx"
    extra.write_text("code")
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_web_dist(fake_web)
    extra.unlink()
    ensure_web_dist(fake_web)
    assert len(build.calls) == 4


def test_ensure_web_dist_rebuilds_on_lockfile_change(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_web_dist(fake_web)
    (fake_web / "deno.lock").write_text("lock-v2")
    ensure_web_dist(fake_web)
    assert len(build.calls) == 4


def test_ensure_web_dist_ignores_test_and_hidden_files(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_web_dist(fake_web)
    (fake_web / "cockpit" / "src" / "App.test.tsx").write_text("test code")
    (fake_web / "shared" / "protocol_test.ts").write_text("test code")
    (fake_web / "cockpit" / ".env.local").write_text("hidden")
    ensure_web_dist(fake_web)
    assert len(build.calls) == 2


def test_ensure_web_dist_failed_sdk_build_keeps_both_dists(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(relay_process, "_run_build", _FakeBuild())
    ensure_web_dist(fake_web)
    sdk_dist, cockpit_dist = _dists(fake_web)
    assert sdk_dist is not None and cockpit_dist is not None
    sdk_before = (sdk_dist / "sdk.js").read_bytes()
    index_before = (cockpit_dist / "index.html").read_bytes()

    (fake_web / "sdk" / "src" / "index.ts").write_text("edited")
    failing = _FakeBuild(fail="sdk")
    monkeypatch.setattr(relay_process, "_run_build", failing)
    ensure_web_dist(fake_web)
    # The cockpit build was not even attempted after the sdk failure, and
    # neither previously valid dist was touched (stale but present beats
    # nothing).
    assert failing.calls == [fake_web / "sdk"]
    assert (sdk_dist / "sdk.js").read_bytes() == sdk_before
    assert (cockpit_dist / "index.html").read_bytes() == index_before


def test_ensure_web_dist_failed_cockpit_build_keeps_both_dists(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(relay_process, "_run_build", _FakeBuild())
    ensure_web_dist(fake_web)
    sdk_dist, cockpit_dist = _dists(fake_web)
    assert sdk_dist is not None and cockpit_dist is not None
    sdk_before = (sdk_dist / "sdk.js").read_bytes()
    index_before = (cockpit_dist / "index.html").read_bytes()

    (fake_web / "sdk" / "src" / "index.ts").write_text("edited")
    monkeypatch.setattr(relay_process, "_run_build", _FakeBuild(fail="cockpit"))
    ensure_web_dist(fake_web)
    # The successful sdk temp build was not swapped in: both or neither.
    assert (sdk_dist / "sdk.js").read_bytes() == sdk_before
    assert (cockpit_dist / "index.html").read_bytes() == index_before
    assert list((fake_web / "sdk").glob(".dist-*")) == []

    shutil.rmtree(cockpit_dist)
    ensure_web_dist(fake_web)
    assert find_cockpit_dist(fake_web) is None  # still failing; nothing appears


def test_ensure_web_dist_wheel_shape_never_builds(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # A wheel ships both dists but no src trees: serve them as-is.
    _make_fake_dist(tmp_path / "cockpit")
    _make_fake_sdk_dist(tmp_path / "sdk")
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    ensure_web_dist(tmp_path)
    assert build.calls == []
    # No lock or stamp writes either: site-packages may be read-only.
    assert not (tmp_path / ".build.lock").exists()
    # A tree with only one src dir cannot build both products: also as-is.
    (tmp_path / "sdk" / "src").mkdir()
    ensure_web_dist(tmp_path)
    assert build.calls == []
    # And a wheel without any web tree is a no-op.
    ensure_web_dist(tmp_path / "nowhere")


def test_ensure_web_dist_env_tree_requires_opt_in(
    fake_web: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # A tree picked via DIMOS_WEB_DIR is served as-is: its build tooling runs
    # with -A, so building needs the explicit acknowledgment.
    build = _FakeBuild()
    monkeypatch.setattr(relay_process, "_run_build", build)
    monkeypatch.setenv(WEB_DIR_ENV_VAR, str(fake_web))
    ensure_web_dist(fake_web)
    assert build.calls == []
    monkeypatch.setenv(WEB_DIR_BUILD_ENV_VAR, "1")
    ensure_web_dist(fake_web)
    assert build.calls == [fake_web / "sdk", fake_web / "cockpit"]


def test_ensure_web_dist_serializes_concurrent_builds(
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
    threads = [threading.Thread(target=lambda: ensure_web_dist(fake_web)) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=30)
    assert peak == 1  # the flock serialized the builders
    # The loser re-checked and found both dists fresh.
    assert build.calls == [fake_web / "sdk", fake_web / "cockpit"]
    assert None not in _dists(fake_web)


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


def test_find_sdk_dist_requires_bundle(tmp_path: Path) -> None:
    assert find_sdk_dist(tmp_path) is None
    (tmp_path / "sdk" / "dist").mkdir(parents=True)
    assert find_sdk_dist(tmp_path) is None
    (tmp_path / "sdk" / "dist" / "sdk.js").write_text("x")
    assert find_sdk_dist(tmp_path) == (tmp_path / "sdk" / "dist").resolve()


def test_find_cockpit_dist_resolves_symlinks(tmp_path: Path) -> None:
    real = tmp_path / "real_web"
    _make_fake_dist(real / "cockpit")
    link = tmp_path / "link_web"
    link.symlink_to(real)
    assert find_cockpit_dist(link) == (real / "cockpit" / "dist").resolve()
