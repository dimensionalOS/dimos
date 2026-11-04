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

"""Remote manipulator settings are local and preserve explicit endpoint/session IDs."""

from dimos.hardware.manipulators.zenoh.config import ManipulatorConnection


def test_connection_reads_only_local_manipulator_environment(monkeypatch):
    monkeypatch.setenv("DIMOS_MANIPULATOR_ENDPOINT", "tcp/127.0.0.1:7450")
    monkeypatch.setenv("DIMOS_MANIPULATOR_RUN", "robot")
    monkeypatch.setenv("DIMOS_MANIPULATOR_EPISODE", "motion")
    monkeypatch.setenv("DIMOS_MUJOCO_EVAL_ENDPOINT", "tcp/127.0.0.1:9999")
    connection = ManipulatorConnection()
    assert (connection.endpoint, connection.run, connection.episode) == (
        "tcp/127.0.0.1:7450",
        "robot",
        "motion",
    )
