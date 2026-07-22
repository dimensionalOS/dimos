"""Direct coverage for the daemonized execution-effect runner."""

from dataclasses import FrozenInstanceError
import subprocess
import sys
import threading

import pytest

from dimos.manipulation.execution_effects import (
    AuxiliaryDone,
    EffectDone,
    ExecutionEffectRunner,
    StopDone,
)


def test_four_blocked_effects_use_four_daemon_execution_rpc_workers() -> None:
    runner = ExecutionEffectRunner()
    entered = [threading.Event() for _ in range(4)]
    release = threading.Event()
    try:
        for event in entered:
            runner.submit_action(
                "blocked",
                lambda event=event: (event.set(), release.wait())[1],
                lambda _: None,
            )

        assert all(event.wait(1) for event in entered)
        workers = runner._executor._threads
        assert len(workers) == 4
        assert {thread.name for thread in workers} == {
            f"execution-rpc_{index}" for index in range(4)
        }
        assert all(thread.daemon for thread in workers)
    finally:
        release.set()
        runner.shutdown()


def test_completion_envelopes_are_immutable_and_correlated() -> None:
    runner = ExecutionEffectRunner()
    completions: list[object] = []
    complete = threading.Event()

    def enqueue(value: object) -> None:
        completions.append(value)
        if len(completions) == 4:
            complete.set()

    try:
        runner.submit_action("action-7", lambda: "done", enqueue)
        runner.submit_auxiliary("aux-3", lambda: 42, enqueue)
        runner.submit_stop(lambda: None, enqueue)
        runner.submit_action("action-8", lambda: None, enqueue)

        assert complete.wait(1)
        assert EffectDone("action-7", "done") in completions
        assert AuxiliaryDone("aux-3", 42) in completions
        assert StopDone(True) in completions
        assert EffectDone("action-8", None) in completions
        with pytest.raises(FrozenInstanceError):
            completions[0].action_id = "changed"  # type: ignore[attr-defined]
    finally:
        runner.shutdown()


def test_permanently_blocked_effect_does_not_hold_interpreter_exit() -> None:
    code = """
import threading
from dimos.manipulation.execution_effects import ExecutionEffectRunner

runner = ExecutionEffectRunner()
runner.submit_action("never", threading.Event().wait, lambda _: None)
"""
    process = subprocess.Popen(
        [sys.executable, "-c", code],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        _, stderr = process.communicate(timeout=2)
    except subprocess.TimeoutExpired:
        process.kill()
        process.communicate()
        process.wait()
        pytest.fail("permanently blocked effect held interpreter exit")
    process.wait()
    assert process.returncode == 0, stderr
