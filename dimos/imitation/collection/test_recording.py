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


import pytest

from dimos.imitation.collection.recording import RecordingSchema


@pytest.mark.parametrize("payload", ["../elsewhere.db", "/tmp/elsewhere.mcap"])
def test_schema_cannot_redirect_preparation_outside_directory(payload):
    with pytest.raises(ValueError, match="payload"):
        RecordingSchema(name="test", robot_type="test", payload=payload)


def test_missing_payload_is_reported(tmp_path):
    (tmp_path / "schema.json").write_text(
        RecordingSchema(name="test", robot_type="test").model_dump_json()
    )
    with pytest.raises(FileNotFoundError, match="payload is missing"):
        RecordingSchema.read(tmp_path)
