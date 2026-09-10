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

"""Prevent overlapping demos from mixing observations or sharing motor buffers."""

import os

import pytest

from dimos.robot.galaxea.r1pro.sim_session import DemoSessionInUseError, reserve_demo_session


def test_same_messaging_address_is_rejected_before_touching_the_second_output(tmp_path):
    first, second = tmp_path / "first", tmp_path / "second"
    with reserve_demo_session("224.0.0.224:19467", first, lock_directory=tmp_path / "locks"):
        with pytest.raises(DemoSessionInUseError, match=f"PID {os.getpid()}.*messaging address"):
            with reserve_demo_session(
                "224.0.0.224:19467", second, lock_directory=tmp_path / "locks"
            ):
                pytest.fail("A second demo acquired an active messaging address")
        assert not second.exists()


def test_output_conflict_releases_the_partially_acquired_messaging_lock(tmp_path):
    output = tmp_path / "scene"
    with reserve_demo_session("224.0.0.224:19467", output, lock_directory=tmp_path / "locks"):
        with pytest.raises(DemoSessionInUseError, match="output directory"):
            with reserve_demo_session(
                "224.0.0.224:19468", output, lock_directory=tmp_path / "locks"
            ):
                pytest.fail("A second demo acquired active shared-memory resources")
        # The failed attempt must not retain its otherwise-unused address.
        independent = tmp_path / "independent"
        with reserve_demo_session(
            "224.0.0.224:19468", independent, lock_directory=tmp_path / "locks"
        ):
            independent.write_text("ran")
        assert independent.read_text() == "ran"


def test_failed_run_releases_resources_for_the_next_attempt(tmp_path):
    output = tmp_path / "scene"
    with pytest.raises(RuntimeError, match="simulation failed"):
        with reserve_demo_session("224.0.0.224:19467", output, lock_directory=tmp_path / "locks"):
            raise RuntimeError("simulation failed")
    with reserve_demo_session("224.0.0.224:19467", output, lock_directory=tmp_path / "locks"):
        output.write_text("restarted")
    assert output.read_text() == "restarted"


def test_output_alias_cannot_bypass_the_shared_memory_reservation(tmp_path):
    output = tmp_path / "scene"
    alias = tmp_path / "subdir" / ".." / "scene"
    with reserve_demo_session("224.0.0.224:19467", output, lock_directory=tmp_path / "locks"):
        with pytest.raises(DemoSessionInUseError, match="output directory"):
            with reserve_demo_session(
                "224.0.0.224:19468", alias, lock_directory=tmp_path / "locks"
            ):
                pytest.fail("An output path alias bypassed the active reservation")
