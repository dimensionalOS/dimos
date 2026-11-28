# Copyright 2025-2026 Dimensional Inc.
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

from pathlib import Path
import pickle

import pytest

from dimos.types.timestamped import Timestamped
from dimos.utils import data
from dimos.utils.testing.legacy_pickle import LegacyPickleStore


def test_save_after_read_keeps_packaged_frames_intact(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    packaged = tmp_path / "data"
    recording = packaged / "trial"
    recording.mkdir(parents=True)
    frame = recording / "000.pickle"
    original = pickle.dumps((1.0, "packaged"))
    frame.write_bytes(original)
    monkeypatch.setenv("DIMOS_DATA_DIR", str(packaged))
    monkeypatch.setattr(data, "STATE_DIR", tmp_path / "state")
    store = LegacyPickleStore[Timestamped]("trial")
    assert store.load_one(0) == (1.0, "packaged")

    store.save(Timestamped(ts=2.0))
    ts, saved = LegacyPickleStore[Timestamped]("trial").load_one(0)
    assert ts == saved.ts == 2.0
    assert list(recording.iterdir()) == [frame]
    assert frame.read_bytes() == original

    with pytest.raises(PermissionError, match="DIMOS_DATA_DIR"):
        LegacyPickleStore[Timestamped](recording).save(Timestamped(ts=3.0))
