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

"""Tests for Stream.save() and Notifier integration."""

from __future__ import annotations

import pytest

from dimos.memory2.backend import Backend
from dimos.memory2.codecs.pickle import PickleCodec
from dimos.memory2.notifier.base import Notifier
from dimos.memory2.observationstore.memory import ListObservationStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import FnTransformer
from dimos.memory2.type.observation import Observation


def _make_backend(name: str = "test") -> Backend[int]:
    return Backend(metadata_store=ListObservationStore[int](name=name), codec=PickleCodec())


def make_stream(n: int = 5, start_ts: float = 0.0) -> Stream[int]:
    backend = _make_backend()
    for i in range(n):
        backend.append(Observation(id=-1, ts=start_ts + i, _data=i * 10))
    return Stream(source=backend)


# ═══════════════════════════════════════════════════════════════════
#  Protocol checks
# ═══════════════════════════════════════════════════════════════════


class TestProtocol:
    def test_backend_has_notifier(self) -> None:
        b = _make_backend("x")
        assert isinstance(b.notifier, Notifier)


# ═══════════════════════════════════════════════════════════════════
#  .save()
# ═══════════════════════════════════════════════════════════════════


class TestSave:
    def test_save_populates_target(self) -> None:
        source = make_stream(3)
        target = Stream(source=_make_backend("target"))

        source.save(target)

        results = target.fetch()
        assert len(results) == 3
        assert [o.data for o in results] == [0, 10, 20]

    def test_save_returns_target_stream(self) -> None:
        source = make_stream(2)
        target = Stream(source=_make_backend("target"))

        result = source.save(target)

        assert result is target

    def test_save_preserves_data(self) -> None:
        backend = _make_backend("src")
        backend.append(Observation(id=-1, ts=1.0, pose=(1, 2, 3), tags={"label": "cat"}, _data=42))
        source = Stream(source=backend)

        target = Stream(source=_make_backend("dst"))
        source.save(target)

        obs = target.first()
        assert obs.data == 42
        assert obs.ts == 1.0
        assert obs.pose == (1, 2, 3)
        assert obs.tags == {"label": "cat"}

    def test_save_with_transform(self) -> None:
        source = make_stream(3)  # data: 0, 10, 20
        doubled = source.transform(FnTransformer(lambda obs: obs.derive(data=obs.data * 2)))

        target = Stream(source=_make_backend("target"))
        doubled.save(target)

        assert [o.data for o in target.fetch()] == [0, 20, 40]

    def test_save_rejects_transform_target(self) -> None:
        source = make_stream(2)
        base = make_stream(2)
        transform_stream = base.transform(FnTransformer(lambda obs: obs.derive(obs.data)))

        with pytest.raises(TypeError, match="Cannot save to a transform stream"):
            source.save(transform_stream)

    def test_save_target_queryable(self) -> None:
        source = make_stream(5, start_ts=0.0)  # ts: 0,1,2,3,4

        target = Stream(source=_make_backend("target"))
        result = source.save(target)

        after_2 = result.after(2.0).fetch()
        assert [o.data for o in after_2] == [30, 40]

    def test_save_empty_source(self) -> None:
        source = make_stream(0)
        target = Stream(source=_make_backend("target"))

        result = source.save(target)

        assert result.count() == 0
        assert result.fetch() == []
