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

import os
from pathlib import Path
import threading

import pytest

from .scheduler_models import AttemptContext, JobIdentity, TerminalOutcome
from .scheduler_plan import job_id
from .scheduler_store import FilesystemExperimentStore
from .test_scheduler_hardening import manifest_for, plan


def _attempt(tmp_path: Path) -> tuple[FilesystemExperimentStore, AttemptContext, Path]:
    value = plan()
    store = FilesystemExperimentStore(tmp_path / "exp")
    store.create(manifest_for(value), value)
    identity = JobIdentity(
        experiment_id="exp",
        case_id="case",
        condition_name="condition",
        job_id=job_id(value, value.cases[0], value.conditions[0]),
    )
    context = AttemptContext(
        identity=identity, attempt_id="attempt-1", attempt_number=1, directory_name="attempt-1"
    )
    with store.coordinator_lock():
        directory = store.create_attempt(context, value.cases[0], value.conditions[0])
    return store, context, directory


def test_observer_sees_absent_until_atomic_publish(monkeypatch, tmp_path: Path) -> None:
    store, context, directory = _attempt(tmp_path)
    durable = threading.Event()
    publish = threading.Event()
    original_link = os.link

    def blocked_link(
        src: str,
        dst: str,
        *,
        src_dir_fd: int | None = None,
        dst_dir_fd: int | None = None,
        follow_symlinks: bool = True,
    ) -> None:
        durable.set()
        assert publish.wait(timeout=5)
        original_link(
            src,
            dst,
            src_dir_fd=src_dir_fd,
            dst_dir_fd=dst_dir_fd,
            follow_symlinks=follow_symlinks,
        )

    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.scheduler_store.os.link", blocked_link)
    failure: list[BaseException] = []

    def publish_outcome() -> None:
        try:
            with store.coordinator_lock():
                store.write_outcome(context, TerminalOutcome(status="succeeded", reason="done"))
        except BaseException as error:
            failure.append(error)

    thread = threading.Thread(target=publish_outcome)
    thread.start()
    assert durable.wait(timeout=5)
    assert store.read_outcome(context) is None
    observed = store.observe_attempts_read_only("exp", context.identity.job_id)
    assert len(observed) == 1 and observed[0].outcome is None
    assert not (directory / "outcome.v1.json").exists()
    publish.set()
    thread.join(timeout=5)
    assert not failure
    assert store.read_outcome(context) == TerminalOutcome(status="succeeded", reason="done")
    observed = store.observe_attempts_read_only("exp", context.identity.job_id)
    assert observed[0].outcome == TerminalOutcome(status="succeeded", reason="done")
    assert not list(directory.glob(".outcome.v1.json.*.tmp"))


def test_second_publisher_cannot_replace_and_temporary_is_removed(tmp_path: Path) -> None:
    store, context, directory = _attempt(tmp_path)
    first = TerminalOutcome(status="failed", reason="first")
    second = TerminalOutcome(status="succeeded", reason="second")
    with store.coordinator_lock():
        store.write_outcome(context, first)
    with pytest.raises(FileExistsError):
        with store.coordinator_lock():
            store.write_outcome(context, second)
    assert store.read_outcome(context) == first
    assert not list(directory.glob(".outcome.v1.json.*.tmp"))


def test_outcome_symlink_is_not_followed_or_replaced(tmp_path: Path) -> None:
    store, context, directory = _attempt(tmp_path)
    target = tmp_path / "outside.json"
    target.write_text("outside")
    (directory / "outcome.v1.json").symlink_to(target)
    with pytest.raises(FileExistsError):
        with store.coordinator_lock():
            store.write_outcome(context, TerminalOutcome(status="failed", reason="blocked"))
    assert target.read_text() == "outside"
    assert not list(directory.glob(".outcome.v1.json.*.tmp"))
