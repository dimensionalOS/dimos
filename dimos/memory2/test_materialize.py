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

from dimos.memory2.backend import Backend
from dimos.memory2.codecs.pickle import PickleCodec
from dimos.memory2.observationstore.memory import ListObservationStore
from dimos.memory2.stream import Stream
from dimos.memory2.type.observation import Observation


def make_stream(n: int = 5, start_ts: float = 0.0) -> Stream[int]:
    backend: Backend[int] = Backend(
        metadata_store=ListObservationStore[int](name="test"), codec=PickleCodec()
    )
    for i in range(n):
        backend.append(Observation(id=-1, ts=start_ts + i, _data=i * 10))
    return Stream(source=backend)


class TestMaterialize:
    def test_returns_same_data(self) -> None:
        materialized = make_stream(3).materialize()
        assert [o.data for o in materialized] == [0, 10, 20]

    def test_replayable(self) -> None:
        materialized = make_stream(3).materialize()
        first = [o.data for o in materialized]
        second = [o.data for o in materialized]
        assert first == second == [0, 10, 20]

    def test_with_transform(self) -> None:
        materialized = make_stream(3).map(lambda obs: obs.derive(data=obs.data * 2)).materialize()
        assert [o.data for o in materialized] == [0, 20, 40]
        assert [o.data for o in materialized] == [0, 20, 40]

    def test_queryable(self) -> None:
        materialized = make_stream(5, start_ts=0.0).materialize()
        assert [o.data for o in materialized.after(2.0)] == [30, 40]
