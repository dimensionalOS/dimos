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

import sqlite3

from pytest_mock import MockerFixture

from dimos.memory.codecs.pickle import PickleCodec
from dimos.memory.observationstore import sqlite as sqlite_module
from dimos.memory.observationstore.sqlite import SqliteObservationStore
from dimos.memory.type.observation import Observation


def test_sqlite_observation_store_uses_json_fallback(mocker: MockerFixture) -> None:
    connection = sqlite3.connect(":memory:")
    mocker.patch.object(sqlite_module, "_JSON_FUNCTION", "json")
    store = SqliteObservationStore(
        conn=connection,
        name="events",
        codec=PickleCodec(),
    )

    store.start()
    row_id = store.insert(Observation(id=-1, ts=1.0, tags={"kind": "demo"}, _data=1))
    store.commit()

    tags = connection.execute(
        "SELECT json_extract(tags, '$.kind') FROM events WHERE id = ?",
        (row_id,),
    ).fetchone()
    assert tags == ("demo",)
