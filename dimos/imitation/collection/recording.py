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

"""Portable collection directories: one raw artifact and its dataset schema."""

from pathlib import Path
from typing import Literal

from dimos.imitation.dataprep.core import DataPrepConfig, DatasetSchema, OutputConfig


class RecordingSchema(DatasetSchema):
    """JSON snapshot of the collection profile; no Python classes or absolute paths."""

    name: str
    robot_type: str
    payload: Literal["recording.mcap", "recording.db"] = "recording.mcap"

    @classmethod
    def read(cls, directory: Path) -> "RecordingSchema":
        schema = cls.model_validate_json((directory / "schema.json").read_text())
        if not (directory / schema.payload).is_file():
            raise FileNotFoundError(f"Recording payload is missing: {directory / schema.payload}")
        return schema

    def dataprep_config(self, directory: Path, output: OutputConfig) -> DataPrepConfig:
        output = output.model_copy(
            update={
                "metadata": {
                    "repo_id": f"local/{self.name}",
                    "robot_type": self.robot_type,
                    **output.metadata,
                }
            }
        )
        return DataPrepConfig(
            **self.model_dump(exclude={"name", "robot_type", "payload"}),
            source=str(directory / self.payload),
            output=output,
        )
