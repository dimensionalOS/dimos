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

import hashlib
from pathlib import Path

import pytest
from textual.widgets import DataTable

from dimos.cloud import data as cd
from dimos.cloud.data import CloudData, DataApi, MultipartBackend
from dimos.cloud.test_data import FakeTransport
from dimos.cloud.tui import DataBrowser, DetailScreen


def _seeded_cloud() -> tuple[CloudData, bytes]:
    """One recording with topics, one manifest-less blob (the zero-topic case)."""
    t = FakeTransport()
    blob = b"hello lidar"
    t.uploads["u0"] = {
        "filename": "session_go2_1.db",
        "created_at": "2026-08-30T12:00:00+00:00",
        "kind": "recording",
        "state": "complete",
        "size": len(blob),
        "sha256": hashlib.sha256(blob).hexdigest(),
        "content_encoding": None,
        "robot_id": "go2",
        "uploader_email": "stash@dimensional.org",
        "manifest": {"streams": [{"name": "/lidar"}, {"name": "/tf"}]},
        "blob": blob,
    }
    t.uploads["u1"] = dict(
        t.uploads["u0"], filename="chunk.db.lz4", kind="blob", manifest=None, sha256="x"
    )
    t.uploads["u2"] = dict(
        t.uploads["u0"],
        filename="one.db",
        sha256="y",
        manifest={"streams": [{"name": "/global_costmap_updates_throttled_full"}]},
    )
    return CloudData(MultipartBackend(DataApi(t), "", None, retries=1)), blob


async def test_browser(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(cd, "DOWNLOADS_DIR", tmp_path)
    cloud, blob = _seeded_cloud()
    app = DataBrowser(cloud)
    async with app.run_test(size=(120, 24)) as pilot:
        await app.workers.wait_for_complete()
        await pilot.pause()
        table = app.query_one(DataTable)
        assert table.row_count == 3
        assert str(table.get_row_at(0)[4]) == "stash@dimensional.org"
        assert str(table.get_row_at(0)[7]) == "2 topics"

        await pilot.press("enter")  # expand topics on the cursor row
        assert str(table.get_row_at(0)[7]) == "/lidar\n/tf"
        await pilot.press("space")  # collapse again
        assert str(table.get_row_at(0)[7]) == "2 topics"

        # zero-topic row: toggling is a no-op — a rebuild whose geometry is
        # unchanged used to leave the post-clear blank frame on screen
        await pilot.press("down", "enter")
        assert not app._expanded
        assert str(table.get_row_at(1)[7]) == "0 topics"

        # one wrapping topic: heights must be valid immediately (sync measure),
        # without waiting for the idle pass
        await pilot.press("down")
        heights = [r.height for r in table.rows.values()]
        await pilot.press("enter")
        assert str(table.get_row_at(2)[7]) == "/global_costmap_updates_throttled_full"
        assert [r.height for r in table.rows.values()][2] > heights[2]
        assert all(r.height >= 1 for r in table.rows.values())
        await pilot.press("space", "up")  # collapse, back to zero-topic row

        await pilot.press("d")
        assert isinstance(app.screen, DetailScreen)
        await pilot.press("escape")
        assert not isinstance(app.screen, DetailScreen)

        await pilot.press("up", "p")  # pull the recording
        await app.workers.wait_for_complete()
    pulled = list(tmp_path.iterdir())
    assert len(pulled) == 1
    assert pulled[0].read_bytes() == blob


async def test_pull_cancel(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    import time

    monkeypatch.setattr(cd, "DOWNLOADS_DIR", tmp_path)
    cloud, _ = _seeded_cloud()
    t = cloud.backend.api.t
    real = t.download

    def slow(url: str, dst: Path, progress=None) -> None:  # type: ignore[no-untyped-def]
        real(url, dst)
        for i in range(200):  # ~4s of ticks; the cancel lands in the first few
            if progress:
                progress(i, 1000)
            time.sleep(0.02)

    t.download = slow  # type: ignore[method-assign]
    app = DataBrowser(cloud)
    async with app.run_test(size=(120, 24)) as pilot:
        await app.workers.wait_for_complete()
        await pilot.pause()
        await pilot.press("p")
        while app._pull_cancel is None:  # worker not started yet
            await pilot.pause(0.01)
        await pilot.press("x")
        await app.workers.wait_for_complete()
        assert app._pull_cancel is None
        assert not app.query_one("#pull-status").display
    assert not list(tmp_path.iterdir())  # nothing kept from the aborted pull
