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

"""`dimos arm`: the walk-through on a scripted module, with no process, bus or arm."""

from __future__ import annotations

import io
import os
from pathlib import Path
import signal
import time
from types import SimpleNamespace
from typing import Any

import pytest
from typer.testing import CliRunner

from dimos.cli.commands import arm as arm_command
from dimos.cli.commands.arm_run import OurRun, calibrate_lock
from dimos.cli.dimos import main
from dimos.robot.manipulators.calibration.schema import calibration_path, save_calibration
from dimos.robot.manipulators.calibration.test_schema import TOY_MODEL, TOY_URDF, calibration

ARM_ID = "dual_openyam/left"
RUN_ID = "20261118-101010-abcd-dual-openyam-calibrate"
STEPS = [
    {"id": "check", "title": "Check the arm"},
    {"id": "gravity_fit", "title": "Fit gravity"},
    {"id": "float_check", "title": "Float check"},
]


def status(
    state: str, *, prompt: str | None = None, prompt_id: int | None = None, **states: str
) -> dict[str, Any]:
    steps = [
        {
            "id": s["id"],
            "title": s["title"],
            "status": states.get(s["id"], "pending"),
            "message": f"{s['id']} ok",
        }
        for s in STEPS
    ]
    return {
        "state": state,
        "arm_id": ARM_ID,
        "run_id": "r1",
        "step": "check",
        "message": f"message of {state}",
        "prompt": prompt,
        "prompt_id": prompt_id,
        "prompt_deadline": None,
        "progress": None,
        "steps": steps,
        "park": None,
        "updated": 0.0,
    }


RESULTS = {
    "gravity_fit": {
        "holdout_rms_nm": {"left_joint2": {"before": 0.5, "after": 0.1}},
        "at_bounds": ["left_link3 mass at its upper bound"],
        "refusals": [],
        "saved": "/state/arms/dual_openyam/left/calibration.json",
        "applied": True,
    },
    "float_check": {
        "pose": "gravity pose pose_01",
        "joints": {
            "left_joint2": {"passed": True, "fingertip_drift_mm": 0.8},
            "left_joint3": {"passed": False, "reason": "left_joint3 drifted 9.0 deg"},
        },
    },
}


def home_script(end: str = "done", message: str = "the arm is at home") -> list[dict[str, Any]]:
    return [
        status("running") | {"step": "home"},
        status("waiting_confirm", prompt="The arm will ramp to home", prompt_id=50),
        status(end) | {"message": message, "step": None},
    ]


class FakeModule:
    """Plays a list of statuses; a waiting one stays until it is answered. Idle until run."""

    def __init__(self, script: list[dict[str, Any]]) -> None:
        self.script = script
        self.index = 0
        self.answered = False
        self.started = False
        self.calls: list[tuple[str, Any]] = []
        self.home_script = home_script()
        self.home_errors: list[Exception] = []
        self.abort_holds = False
        self.run_id = RUN_ID
        self.arm_id = ARM_ID

    def plan(self) -> list[dict[str, Any]]:
        return [
            {"id": "check", "title": "Check the arm", "supported": True, "reason": None},
            {
                "id": "gravity_fit",
                "title": "Fit gravity",
                "supported": False,
                "reason": "no recording",
            },
            {"id": "float_check", "title": "Float check", "supported": True, "reason": None},
        ]

    def dimos_run_id(self) -> str:
        return self.run_id

    def run(self, steps: list[str] | None = None) -> dict[str, Any]:
        self.calls.append(("run", steps))
        self.started = True
        return self.script[0]

    def status(self) -> dict[str, Any]:
        if not self.started:
            return status("idle") | {"arm_id": self.arm_id, "run_id": None}
        current = self.script[self.index]
        if current["state"] == "waiting_confirm" and not self.answered:
            return current
        if self.index + 1 < len(self.script):
            self.index += 1
            self.answered = False
        return self.script[self.index]

    def results(self) -> dict[str, Any]:
        return RESULTS

    def confirm(self, prompt_id: int) -> bool:
        self.calls.append(("confirm", prompt_id))
        self.answered = True
        return True

    def skip(self, prompt_id: int) -> bool:
        self.calls.append(("skip", prompt_id))
        self.answered = True
        return True

    def abort(self) -> bool:
        self.calls.append(("abort", None))
        if not self.abort_holds:
            self.script = [status("aborted", gravity_fit="aborted") | {"message": "aborted"}]
            self.index = 0
        return True

    def go_home(self) -> dict[str, Any]:
        self.calls.append(("go_home", None))
        if self.home_errors:
            raise self.home_errors.pop(0)
        self.script, self.index, self.answered = self.home_script, 0, False
        return self.script[0]

    def named(self, *names: str) -> list[tuple[str, Any]]:
        return [call for call in self.calls if call[0] in names]


class ScriptedKeys:
    """Each line answers the next prompt shown; a line starting "!" is typed while it runs.

    Once the lines run out, the next prompt sees the end of input.
    """

    def __init__(self, lines: list[str]) -> None:
        self.lines = list(lines)
        self.open = False

    def drain(self) -> None:
        self.open = True

    def get(self) -> object:
        if self.lines and self.lines[0].startswith("!"):
            return self.lines.pop(0)[1:]
        if not self.open:
            return None
        self.open = False
        return self.lines.pop(0) if self.lines else arm_command.EOF


class Rig:
    def __init__(self) -> None:
        self.module = FakeModule([])
        self.keys: list[str] = []
        self.busy: list[arm_command.ActiveRun] = []
        self.bus_busy = False
        self.alive = True
        self.launched: list[list[str]] = []
        self.globals: list[tuple[list[str], dict[str, Any]]] = []
        self.stopped: list[str] = []
        self.connect: Any = None


@pytest.fixture
def rig(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Rig:
    rig = Rig()
    monkeypatch.setattr(arm_command, "POLL_S", 0.0)
    monkeypatch.setattr(arm_command, "ABORT_WAIT_S", 0.2)
    monkeypatch.setattr(arm_command, "dimos_command", lambda: ["dimos"])
    monkeypatch.setattr(arm_command, "active_runs", lambda: rig.busy)
    monkeypatch.setattr(arm_command, "coordinator_answers", lambda: rig.bus_busy)
    monkeypatch.setattr(
        arm_command,
        "use_run_globals",
        lambda tokens, overrides: rig.globals.append((list(tokens), dict(overrides))),
    )

    def launch(argv: list[str], *, token: str) -> OurRun:
        assert token.startswith("dual-openyam-calibrate:")
        rig.launched.append(argv)
        return OurRun(RUN_ID, 4242, token, 0.0, tmp_path / "launch.log")

    monkeypatch.setattr(arm_command, "launch_run", launch)

    def stop(run: OurRun) -> str:
        rig.stopped.append(run.run_id)
        return f"stopped {run.run_id}"

    monkeypatch.setattr(arm_command, "stop_run", stop)
    monkeypatch.setattr(arm_command, "run_alive", lambda run: rig.alive)

    def connect(timeout: float, alive: Any) -> FakeModule:
        if rig.connect is not None:
            rig.connect()
        return rig.module

    monkeypatch.setattr(arm_command, "connect_module", connect)
    monkeypatch.setattr(arm_command, "make_keys", lambda: ScriptedKeys(rig.keys))
    monkeypatch.setattr(arm_command, "arms_root", lambda: tmp_path / "arms")
    return rig


def happy_script() -> list[dict[str, Any]]:
    return [
        status("running"),
        status("waiting_confirm", prompt="Move to home?", prompt_id=7, check="done"),
        status("running", check="done"),
        status("waiting_confirm", prompt="Save and apply?", prompt_id=8, check="done"),
        status("running", check="done", gravity_fit="done"),
        status("done", check="done", gravity_fit="done", float_check="done"),
    ]


def invoke(*args: str, input: str = "") -> Any:
    return CliRunner().invoke(main, ["arm", *args], input=input)


def test_calibrate_answers_the_right_prompts_goes_home_and_only_then_stops_its_run(
    rig: Rig,
) -> None:
    rig.module.script = happy_script()
    # Move to home?, Save and apply? (skip), the final offer (Enter: go home), its confirmation.
    rig.keys = ["", "s", "", ""]

    result = invoke("calibrate", "dual-openyam")

    out = result.output
    assert result.exit_code == 0, out
    assert "✓ check: Check the arm" in out
    assert "x gravity_fit: Fit gravity  (cannot run: no recording)" in out
    assert rig.module.named("confirm", "skip", "go_home") == [
        ("confirm", 7),
        ("skip", 8),
        ("go_home", None),
        ("confirm", 50),
    ]
    assert rig.module.calls[0] == ("run", None)
    assert "Move to home?" in out and "Save and apply?" in out
    # The results of a finished step come as a table.
    assert "left_joint2  0.500 -> 0.100" in out
    assert "at a bound" in out and "left_joint3" in out and "FAIL" in out
    assert "Calibration file: /state/arms/dual_openyam/left/calibration.json" in out
    assert "history/" in out and "calibration off" in out
    assert "the arm is at home" in out
    assert rig.stopped == [RUN_ID]
    [argv] = rig.launched
    assert argv == [
        "dimos",
        "run",
        "dual-openyam-calibrate",
        "--daemon",
        "--calibrationmodule.arm",
        "left",
    ]
    # The run's global options were resolved for the walk-through's own connection too.
    assert rig.globals == [([], {})]


def test_q_at_a_prompt_aborts_once_then_the_arm_goes_home_before_the_stop(rig: Rig) -> None:
    rig.module.script = happy_script()
    rig.keys = ["q", "", ""]

    result = invoke("calibrate", "dual-openyam", "--steps", "check,float_check")

    assert result.exit_code == 130, result.output
    assert ("run", ["check", "float_check"]) in rig.module.calls
    assert rig.module.named("abort", "go_home", "confirm") == [
        ("abort", None),
        ("go_home", None),
        ("confirm", 50),
    ]
    assert "Run aborted" in result.output
    assert "q: aborting; the arm stops and holds where it is, its motors on" in result.output
    assert rig.stopped == [RUN_ID]


@pytest.mark.parametrize("key", ["q", "p"])
def test_q_or_p_while_the_arm_moves_aborts_without_waiting_for_a_prompt(rig: Rig, key: str) -> None:
    rig.module.script = happy_script()
    rig.keys = [f"!{key}", "", ""]

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 130, result.output
    assert rig.module.named("abort", "confirm")[0] == ("abort", None)
    assert ("confirm", 7) not in rig.module.calls
    assert len(rig.module.named("abort")) == 1
    assert rig.stopped == [RUN_ID]


def test_eof_at_a_prompt_aborts_exactly_once_and_leaves_the_run_up_without_a_yes(
    rig: Rig,
) -> None:
    rig.module.script = happy_script()
    rig.keys = []

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 130, result.output
    assert rig.module.named("abort", "go_home", "confirm", "skip") == [("abort", None)]
    assert "input closed: aborting" in result.output
    assert "Stop anyway: the arm will go limp where it is" in result.output
    assert f"Left {RUN_ID} running" in result.output
    assert rig.stopped == []


def test_keeping_the_arm_away_from_home_needs_an_explicit_yes(rig: Rig) -> None:
    rig.module.script = happy_script()
    # Abort; keep the arm where it is; Enter (No) at the stop prompt; k again; yes.
    rig.keys = ["q", "k", "", "k", "yes"]

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 130, result.output
    assert rig.module.named("go_home") == []
    assert result.output.count("Stop anyway: the arm will go limp where it is") == 2
    assert "backstop park" in result.output
    assert rig.stopped == [RUN_ID]


def test_a_failed_run_exits_non_zero_and_h_goes_home_after_its_own_confirmation(rig: Rig) -> None:
    rig.module.script = [
        status("running"),
        status("failed", check="done") | {"message": "Gravity poses failed: stalled"},
    ]
    rig.keys = ["h", ""]

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 1, result.output
    assert "Run failed: Gravity poses failed: stalled" in result.output
    assert rig.module.named("go_home", "confirm") == [("go_home", None), ("confirm", 50)]
    assert "the arm is at home" in result.output
    assert rig.stopped == [RUN_ID]


def test_going_home_that_fails_needs_a_yes_and_exits_1(rig: Rig) -> None:
    rig.module.script = happy_script()
    rig.module.home_script = home_script("failed", "going home failed: stalled")
    rig.keys = ["", "", "", "", "yes"]

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 1, result.output
    assert "Not at home: going home failed: stalled" in result.output
    assert "Stop anyway" in result.output and rig.stopped == [RUN_ID]


def test_go_home_raising_is_handled_and_already_running_is_retried(rig: Rig) -> None:
    rig.module.script = happy_script()
    rig.module.home_errors = [
        RuntimeError("the procedure is already running"),
        RuntimeError("the procedure is shutting down"),
    ]
    # Prompts 7 and 8; go home: busy, settles, refused; then yes.
    rig.keys = ["", "", "", "yes"]

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 1, result.output
    assert rig.module.named("go_home") == [("go_home", None), ("go_home", None)]
    assert "still busy" in result.output
    assert "Going home was refused: the procedure is shutting down" in result.output
    assert rig.stopped == [RUN_ID]


def test_an_rpc_error_mid_poll_aborts_then_goes_home_then_stops(
    rig: Rig, monkeypatch: pytest.MonkeyPatch
) -> None:
    rig.module.script = happy_script()
    rig.keys = ["", "", ""]
    calls = {"n": 0}
    real_status = rig.module.status

    def flaky() -> dict[str, Any]:
        calls["n"] += 1
        if calls["n"] == 4:
            raise TimeoutError("RPC call to 'CalibrationModule/status' timed out after 2.0 seconds")
        return real_status()

    monkeypatch.setattr(rig.module, "status", flaky)

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 1, result.output
    assert "lost touch with the calibration module (TimeoutError" in result.output
    assert [c[0] for c in rig.module.calls if c[0] in ("abort", "go_home")] == [
        "abort",
        "go_home",
    ]
    assert rig.stopped == [RUN_ID]


def test_an_abort_the_engine_never_confirms_is_not_reported_as_aborted(
    rig: Rig, monkeypatch: pytest.MonkeyPatch
) -> None:
    rig.module.script = [status("running")]
    rig.module.abort_holds = True
    rig.keys = ["!q"]

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 1, result.output
    assert "did not confirm the abort" in result.output
    assert "Run aborted" not in result.output
    assert len(rig.module.named("abort")) == 1
    assert f"Left {RUN_ID} running" in result.output and rig.stopped == []


def test_ctrl_c_aborts_waits_for_the_engine_and_goes_home_before_the_stop(
    rig: Rig, monkeypatch: pytest.MonkeyPatch
) -> None:
    rig.module.script = happy_script()
    rig.keys = ["", "", ""]
    seen: list[str] = []
    real_status = rig.module.status

    def interrupted() -> dict[str, Any]:
        current = real_status()
        seen.append(current["state"])
        if len(seen) == 4:
            raise KeyboardInterrupt
        return current

    monkeypatch.setattr(rig.module, "status", interrupted)

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 130, result.output
    assert [c[0] for c in rig.module.calls if c[0] in ("abort", "go_home")] == [
        "abort",
        "go_home",
    ]
    assert "Ctrl-C: aborting; the arm stops and holds where it is" in result.output
    assert rig.stopped == [RUN_ID]


def test_a_closed_terminal_sighup_takes_the_abort_path(
    rig: Rig, monkeypatch: pytest.MonkeyPatch
) -> None:
    rig.module.script = happy_script()
    rig.keys = [""]
    calls = {"n": 0}
    real_status = rig.module.status

    def hangup() -> dict[str, Any]:
        calls["n"] += 1
        if calls["n"] == 4:
            os.kill(os.getpid(), signal.SIGHUP)
            time.sleep(0.5)
        return real_status()

    monkeypatch.setattr(rig.module, "status", hangup)
    before = signal.getsignal(signal.SIGHUP)

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 130, result.output
    assert "SIGHUP (the terminal closed): aborting" in result.output
    assert len(rig.module.named("abort")) == 1
    # Nobody can type yes on a closed terminal: the run stays up, the arm held.
    assert rig.stopped == [] and f"Left {RUN_ID} running" in result.output
    assert signal.getsignal(signal.SIGHUP) is before


@pytest.mark.parametrize(
    "field,value,said",
    [
        ("arm_id", "dual_openyam/right", "calibrates 'dual_openyam/right'"),
        ("run_id", "20261118-090000-ffff-xarm", "belongs to 20261118-090000-ffff-xarm"),
    ],
)
def test_a_module_of_another_run_or_arm_is_refused_and_never_driven(
    rig: Rig, field: str, value: str, said: str
) -> None:
    setattr(rig.module, field, value)
    rig.module.script = happy_script()
    rig.keys = ["yes"]

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 1, result.output
    assert "connected to a different run" in result.output and said in result.output
    assert rig.module.calls == []
    assert "has not moved the arm" in result.output and rig.stopped == [RUN_ID]


def test_ctrl_c_while_connecting_is_a_clean_message_and_the_stop_still_asks(rig: Rig) -> None:
    def interrupt() -> None:
        raise KeyboardInterrupt

    rig.connect = interrupt
    rig.keys = []

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 130, result.output
    assert "Ctrl-C while connecting; the calibration has not moved the arm." in result.output
    assert "Traceback" not in result.output
    assert rig.stopped == [] and f"Left {RUN_ID} running" in result.output


def test_a_run_whose_process_is_gone_is_cleaned_up_without_asking(rig: Rig) -> None:
    def gone() -> None:
        rig.alive = False
        raise arm_command.ModuleConnectError("the run's process exited")

    rig.connect = gone

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 1, result.output
    assert "Stop anyway" not in result.output and rig.stopped == [RUN_ID]


def test_the_lock_keeps_a_second_walk_through_of_the_robot_out(rig: Rig, tmp_path: Path) -> None:
    with calibrate_lock(tmp_path / "arms" / "dual-openyam.calibrate.lock"):
        result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 1
    assert "another `dimos arm calibrate` of this robot is running" in result.output
    assert f"pid {os.getpid()}" in result.output
    assert rig.launched == [] and rig.stopped == []


def test_a_coordinator_on_the_bus_is_refused_even_without_a_registry_entry(rig: Rig) -> None:
    rig.bus_busy = True

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 1
    assert "already answers on the" in result.output and "still starting" in result.output
    assert rig.launched == []


def test_dry_run_prints_the_command_and_plan_and_launches_nothing(rig: Rig) -> None:
    result = invoke(
        "calibrate",
        "dual-openyam",
        "--dry-run",
        "--poses",
        "6",
        "--no-confirm-each",
        "--dualopenyamconnection.left-can-port",
        "follower_l",
    )

    assert result.exit_code == 0, result.output
    assert (
        "dimos run dual-openyam-calibrate --daemon --calibrationmodule.arm left "
        "--calibrationmodule.poses 6 --calibrationmodule.confirm-each=false "
        "--dualopenyamconnection.left-can-port follower_l"
    ) in result.output
    assert "✓ check: Check the arm" in result.output and "Nothing was started" in result.output
    assert rig.launched == [] and rig.stopped == [] and rig.globals == []


def test_global_options_reach_the_run_and_the_walk_through_connection(rig: Rig) -> None:
    rig.module.script = happy_script()
    rig.keys = ["", "", "", ""]

    result = CliRunner().invoke(
        main,
        [
            "--transport",
            "lcm",
            "--no-zenoh-multicast",
            "arm",
            "calibrate",
            "dual-openyam",
            "--zenoh-connect",
            "tcp/127.0.0.1:7447",
        ],
    )

    assert result.exit_code == 0, result.output
    [argv] = rig.launched
    assert argv[:6] == [
        "dimos",
        "--no-zenoh-multicast",
        "--transport",
        "lcm",
        "run",
        "dual-openyam-calibrate",
    ]
    assert argv[-2:] == ["--zenoh-connect", "tcp/127.0.0.1:7447"]
    assert rig.globals == [
        (["--zenoh-connect", "tcp/127.0.0.1:7447"], {"zenoh_multicast": False, "transport": "lcm"})
    ]


def test_the_run_globals_are_resolved_as_dimos_run_resolves_them() -> None:
    from dimos.core.global_config import global_config

    arm_command.use_run_globals(
        ["--calibrationmodule.arm", "left", "--zenoh-connect", "tcp/127.0.0.1:7447"],
        {"transport": "lcm"},
    )

    assert global_config.transport == "lcm"
    assert global_config.zenoh_connect == "tcp/127.0.0.1:7447"


def test_a_foreign_active_run_is_refused_with_its_id_and_owner(rig: Rig) -> None:
    rig.busy = [
        arm_command.ActiveRun(
            "20261118-090000-ffff-xarm", 4242, "xarm", "tom", "2026-11-18T09:00", "/logs/x"
        )
    ]

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 1
    assert "20261118-090000-ffff-xarm" in result.output and "tom" in result.output
    assert rig.launched == [] and rig.stopped == []


def test_a_bad_robot_lists_the_known_ones_and_a_guardless_arm_needs_the_bench_numbers(
    rig: Rig,
) -> None:
    bad = invoke("calibrate", "nope")
    assert bad.exit_code == 2 and "dual-openyam" in bad.output and "openarm" in bad.output

    result = invoke("calibrate", "openyam", "--dry-run")
    assert result.exit_code == 2
    assert "--floor-z-m" in result.output
    unknown = invoke("calibrate", "dual-openyam", "--steps", "nope")
    assert unknown.exit_code == 2 and "Unknown steps" in unknown.output
    no_arm = invoke("calibrate", "dual-openyam", "--arm", "up", "--dry-run")
    assert no_arm.exit_code == 2 and "has no arm 'up'" in no_arm.output
    assert rig.launched == []


@pytest.mark.parametrize(
    "flags,said",
    [
        (["--poses", "0"], "--poses: Input should be greater than or equal to 1"),
        (["--speed-deg-s", "0"], "--speed-deg-s: Input should be greater than 0"),
        (["--speed-deg-s", "45"], "--speed-deg-s: Input should be less than or equal to 30"),
    ],
)
def test_poses_and_speed_are_checked_before_anything_starts(
    rig: Rig, flags: list[str], said: str
) -> None:
    result = invoke("calibrate", "dual-openyam", *flags)

    assert result.exit_code == 2, result.output
    assert said in result.output
    assert rig.launched == []


def test_stdin_keys_reads_lines_and_forgets_what_was_typed_ahead_except_q_and_eof() -> None:
    keys = arm_command.StdinKeys(io.StringIO("\nS\nq\n"))
    for _ in range(100):
        if keys._lines.qsize() >= 4:
            break
        time.sleep(0.01)
    keys.drain()
    assert keys.get() == "q"
    assert keys.get() is arm_command.EOF
    assert keys.get() is arm_command.EOF


def test_the_walk_through_calls_get_short_timeouts() -> None:
    client = SimpleNamespace(rpc_timeouts={"start": 1200.0})
    module = SimpleNamespace(rpc=client, remote_name="CalibrationModule")

    arm_command.set_call_timeouts(module)

    timeouts = client.rpc_timeouts
    assert timeouts["CalibrationModule/status"] == 2.0
    assert timeouts["CalibrationModule/confirm"] == 2.0
    assert timeouts["CalibrationModule/abort"] == 5.0
    assert timeouts["CalibrationModule/go_home"] == 10.0
    assert timeouts["start"] == 1200.0
    arm_command.set_call_timeouts(object())  # a fake with no client is left alone


def test_each_failed_connect_attempt_is_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    from dimos.porcelain.dimos import Dimos

    module = SimpleNamespace(plan=lambda: [])
    apps: list[Any] = []

    class App:
        def __init__(self, fails: bool) -> None:
            self.fails = fails
            self.stopped = False

        def get_module(self, name: str) -> Any:
            if self.fails:
                raise LookupError("not deployed yet")
            return module

        def stop(self) -> None:
            self.stopped = True

    def connect(*, timeout: float) -> Any:
        apps.append(App(fails=len(apps) == 0))
        return apps[-1]

    monkeypatch.setattr(Dimos, "connect", connect)
    monkeypatch.setattr(arm_command, "CONNECT_RETRY_S", 0.0)

    assert arm_command.connect_module(5.0, lambda: True) is module
    assert [app.stopped for app in apps] == [True, False]
    with pytest.raises(arm_command.ModuleConnectError, match="process exited"):
        arm_command.connect_module(5.0, lambda: False)


def test_show_prints_the_saved_file_or_none(rig: Rig, tmp_path: Path) -> None:
    root = tmp_path / "arms"
    save_calibration(
        calibration_path(ARM_ID, root),
        calibration(arm_id=ARM_ID, robot_model=TOY_MODEL),
    )
    save_calibration(
        calibration_path(ARM_ID, root),
        calibration(arm_id=ARM_ID, robot_model=TOY_MODEL, notes="second"),
    )

    result = invoke("show", "dual-openyam")

    assert result.exit_code == 0, result.output
    out = result.output
    assert f"{root}/{ARM_ID}/calibration.json" in out
    assert "2026-11-18 09:30:00 UTC" in out and f"toy ({TOY_MODEL.sha256[:12]})" in out
    assert "shoulder  80  5" in out and "arm" in out and "2.000" in out
    assert "check float" in out and "1 earlier file(s)" in out
    assert "dual_openyam/right" in out and "  none" in out
    for spelling in ("left", "left_arm", ARM_ID):
        only = invoke("show", "dual-openyam", "--arm", spelling)
        assert only.exit_code == 0 and ARM_ID in only.output
        assert "dual_openyam/right" not in only.output
    assert invoke("show", "dual-openyam", "--arm", "up").exit_code == 2


def test_bake_writes_the_urdf_with_the_fitted_masses_and_refuses_a_foreign_urdf(
    rig: Rig, tmp_path: Path
) -> None:
    save_calibration(
        calibration_path(ARM_ID, tmp_path / "arms"),
        calibration(arm_id=ARM_ID, robot_model=TOY_MODEL),
    )
    source, out = tmp_path / "in.urdf", tmp_path / "out.urdf"
    source.write_text(TOY_URDF)

    result = invoke(
        "bake", "dual-openyam", "--arm", "left_arm", "--urdf", str(source), "--out", str(out)
    )

    assert result.exit_code == 0, result.output
    assert 'value="1.5"' in out.read_text()
    source.write_text(TOY_URDF.replace('<mass value="0.3"/>', '<mass value="0.4"/>'))
    refused = invoke(
        "bake",
        "dual-openyam",
        "--arm",
        ARM_ID,
        "--urdf",
        str(source),
        "--out",
        str(tmp_path / "x.urdf"),
    )
    assert refused.exit_code == 1 and "fingerprint" in refused.output
    assert not (tmp_path / "x.urdf").exists()
    missing = invoke(
        "bake", "dual-openyam", "--arm", "right", "--urdf", str(source), "--out", str(out)
    )
    assert missing.exit_code == 1 and "No arm calibration" in missing.output


def test_every_forwarded_flag_parses_on_the_real_blueprint_to_the_value_given() -> None:
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.robot.get_all_blueprints import load_blueprint

    flags = arm_command.calibrate_flags("left", -0.03, 0.12, 6, 10.0, confirm_each=False)
    parser = BlueprintConfigParser(load_blueprint("dual-openyam-calibrate"))
    parsed = parser.parse(
        [
            *flags,
            "--dualopenyamconnection.left-can-port",
            "follower_l",
            "--dualopenyamconnection.right-can-port",
            "follower_r",
        ],
        config_path=None,
        environ={},
        global_overrides={},
    )

    calibration_values = dict(parsed.module_configs["calibrationmodule"])
    assert calibration_values == {
        "robot": "dual-openyam",
        "arm": "left",
        "floor_z_m": -0.03,
        "min_x_m": 0.12,
        "poses": 6,
        "speed_deg_s": 10.0,
        "confirm_each": False,
    }
    connection = parsed.module_configs["dualopenyamconnection"]
    assert (connection["left_can_port"], connection["right_can_port"]) == (
        "follower_l",
        "follower_r",
    )
    assert connection["tuning"] is True
