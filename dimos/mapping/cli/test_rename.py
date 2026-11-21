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

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import typer
from typer.testing import CliRunner

from dimos.mapping.cli.rename import main
from dimos.memory.blobstore.file import FileBlobStore
from dimos.memory.blobstore.sqlite import SqliteBlobStore
from dimos.memory.codecs.base import codec_for
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.vectorstore.sqlite import SqliteVectorStore
from dimos.models.embedding.base import Embedding
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


@pytest.mark.parametrize(
    ("seek", "duration", "expected"),
    [(0.5, 0.1, []), (1.0, 0.5, [101.0, 101.25])],
)
def test_rename_uses_recording_window_before_dropping_streams(
    tmp_path: Path, seek: float, duration: float, expected: list[float]
) -> None:
    source = tmp_path / "source.db"
    destination = tmp_path / "destination.db"
    with SqliteStore(path=str(source)) as store:
        store.stream("reference", str).append("start", ts=100.0)
        for ts in (100.25, 101.0, 101.25, 101.5, 102.0):
            store.stream("lidar", str).append(f"scan-{ts}", ts=ts)
        for ts in (100.75, 101.0, 101.25, 101.5, 102.0):
            store.stream("aux", str).append(f"aux-{ts}", ts=ts)
    app = typer.Typer()
    app.command()(main)

    result = CliRunner().invoke(
        app,
        [
            str(source),
            "--out",
            str(destination),
            "--rename",
            "lidar=renamed",
            "--drop",
            "reference",
            "--seek",
            str(seek),
            "--duration",
            str(duration),
        ],
    )

    assert result.exit_code == 0, result.output
    with SqliteStore(path=str(destination), must_exist=True) as store:
        assert store.list_streams() == ["aux", "renamed"]
        assert [obs.ts for obs in store.stream("renamed").order_by("ts")] == expected
        assert [obs.ts for obs in store.stream("aux").order_by("ts")] == expected


@pytest.mark.parametrize("fmt", [ImageFormat.RGB, ImageFormat.DEPTH16])
def test_rename_preserves_recorded_image_bytes(tmp_path, fmt):
    source = tmp_path / "source.db"
    destination = tmp_path / "copy.db"
    if fmt == ImageFormat.RGB:
        pixels = np.random.default_rng(0).integers(0, 256, (16, 16, 3), dtype=np.uint8)
    else:
        pixels = np.arange(256, dtype=np.uint16).reshape(16, 16) * 64
    image = Image(data=pixels, format=fmt)
    with SqliteStore(path=source) as src:
        blobs = FileBlobStore(root=str(tmp_path / "blobs"))
        stream = src.stream("image", Image, codec=codec_for(Image, sample=image), blob_store=blobs)
        stream.append(image, ts=1.0, tags={"frame": 1})

    app = typer.Typer()
    app.command()(main)
    result = CliRunner().invoke(
        app, [str(source), "--out", str(destination), "--rename", "image=frame"]
    )

    assert result.exit_code == 0, result.output
    with SqliteStore(path=source) as src, SqliteStore(path=destination) as dst:
        assert dst.list_streams() == ["frame"]
        original = src.stream("image").first()
        copy = dst.stream("frame").first()
        assert (copy.ts, copy.tags, copy.data.format) == (original.ts, original.tags, fmt)
        np.testing.assert_array_equal(copy.data.data, original.data.data)
        with SqliteBlobStore(path=str(destination)) as output_blobs:
            assert output_blobs.get("frame", copy.id) == blobs.get("image", original.id)


def test_rename_preserves_selected_external_embeddings_at_new_ids(tmp_path: Path) -> None:
    source, destination = tmp_path / "source.db", tmp_path / "copy.db"
    x = Embedding(vector=np.array([1.0, 0.0], dtype=np.float32))
    y = Embedding(vector=np.array([0.0, 1.0], dtype=np.float32))
    with SqliteStore(path=source) as store:
        vectors = SqliteVectorStore(path=str(tmp_path / "vectors.db"))
        text = store.stream("text", str, vector_store=vectors)
        text.append("skipped", ts=1.0, embedding=x)
        text.append("plain", ts=2.0)
        text.append("selected", ts=3.0, embedding=y)

    app = typer.Typer()
    app.command()(main)
    result = CliRunner().invoke(
        app, [str(source), "--rename", "text=notes", "--seek", "1", "--out", str(destination)]
    )
    assert result.exit_code == 0, result.output
    with SqliteStore(path=destination, must_exist=True) as copy:
        notes = copy.stream("notes")
        assert [obs.data for obs in notes] == ["plain", "selected"]
        hits = notes.search(y, k=3).to_list()
        assert [obs.data for obs in hits] == ["selected"]
        assert hits[0].similarity == pytest.approx(1.0)
