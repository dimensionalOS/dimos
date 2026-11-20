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

"""``dimos arm``: the plan and command, the operator's keys against a fake module, show and bake."""

from __future__ import annotations

from collections.abc import Callable, Iterator
import os
from pathlib import Path
import queue
import shlex
import signal
import threading
from typing import Any

import pytest
from typer.testing import CliRunner, Result

from dimos.cli.commands import arm as arm_command
from dimos.control.calibration import schema
from dimos.control.calibration._calibration_test_helpers import (
    TOY_MODEL,
    TOY_URDF,
    calibration,
    offline_robot_assets,
    require_model,
)
from dimos.control.calibration.schema import calibration_path, save_calibration
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM

ARM_ID = "dual_openyam/left"
BENCH = ["--arm", "left", "--floor-z-m", "0", "--min-x-m", "0.1"]
WAIT_S = 10.0


def invoke(*args: str) -> Result:
    return CliRunner().invoke(arm_command.arm_app, list(args))


@pytest.fixture
def model_on_disk() -> Iterator[None]:
    require_model(DUAL_OPENYAM.gravity_model, "Dual OpenYAM")
    with offline_robot_assets():
        yield


@pytest.fixture
def no_child(monkeypatch: pytest.MonkeyPatch) -> None:
    def refuse(*args: Any, **kwargs: Any) -> None:
        raise AssertionError("a dry run started a process")

    monkeypatch.setattr(arm_command.subprocess, "Popen", refuse)


def test_dry_run_prints_the_plan_and_the_command_and_starts_nothing(
    model_on_disk: None, no_child: None
) -> None:
    result = invoke("calibrate", "dual-openyam", *BENCH, "--steps", "step_sweep", "--dry-run")

    assert result.exit_code == 0, result.output
    assert f"Plan for {ARM_ID}:" in result.output
    assert f"{'pick_gains':<17} skipped: not selected" in result.output
    assert "check             measure the noise" in result.output
    assert "run dual-openyam-calibrate --calibrationmodule.arm left" in result.output
    assert result.output.rstrip().endswith("Nothing was started.")


def test_every_forwarded_setting_parses_on_the_real_blueprint(
    model_on_disk: None, no_child: None
) -> None:
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.robot.get_all_blueprints import load_blueprint

    result = invoke(
        "calibrate",
        "dual-openyam",
        *BENCH,
        "--kp-scales",
        "1,2",
        "--steps",
        "gravity_poses,gravity_fit",
        "--no-confirm-each",
        "--rest-window-s",
        "0.4",
        "--dry-run",
    )
    assert result.exit_code == 0, result.output
    argv = shlex.split(result.output.split("Would run:\n  ")[1].splitlines()[0])
    flags = argv[argv.index("dual-openyam-calibrate") + 1 :]
    parser = BlueprintConfigParser(autoconnect(load_blueprint("dual-openyam-calibrate")).solidify())
    parsed = parser.parse(
        [
            *flags,
            "--dualopenyamconnection.left-can-port",
            "l",
            "--dualopenyamconnection.right-can-port",
            "r",
        ],
        config_path=None,
        environ={},
        global_overrides={},
    )
    assert dict(parsed.module_configs["calibrationmodule"]) == {
        "robot": "dual-openyam",
        "arm": "left",
        "floor_z_m": 0.0,
        "min_x_m": 0.1,
        "kp_scales": (1.0, 2.0),
        "steps": ("gravity_poses", "gravity_fit"),
        "rest_window_s": 0.4,
        "confirm_each": False,
    }
    assert parsed.module_configs["dualopenyamconnection"]["config_writes"] is True


@pytest.mark.parametrize(
    "args, message",
    [
        (["--kp-scales", "1.5,2"], "include 1"),
        (["--steps", "dance"], "unknown steps"),
        (["--speed-deg-s", "61"], "less than or equal to 60"),
    ],
)
def test_bad_settings_are_refused_before_anything_starts(
    model_on_disk: None, no_child: None, args: list[str], message: str
) -> None:
    result = invoke("calibrate", "dual-openyam", *BENCH, *args)
    assert result.exit_code == 2 and message in result.output
    unguarded = invoke("calibrate", "dual-openyam", "--arm", "left")
    assert unguarded.exit_code == 2 and "floor_z_m and min_x_m" in unguarded.output


class FakeRpc:
    def __init__(self) -> None:
        self.handlers: dict[str, Callable[..., Any]] = {}
        self.aborts = threading.Event()
        self.stopped = False

    def start(self) -> None: ...

    def stop(self) -> None:
        self.stopped = True

    def serve_rpc(self, f: Callable[..., Any], name: str) -> None:
        self.handlers[name.rsplit("/", 1)[-1]] = f

    def call(self, name: str, args: Any, reply: Callable[[Any], None]) -> None:
        assert name == "CalibrationModule/abort"
        self.aborts.set()
        reply(None)

    def answer_next(self, text: str) -> None:
        """Type ``text`` once the operator is asked."""
        operator = self.handlers["ask"].__self__

        def type_it() -> None:
            assert operator._asking.wait(WAIT_S)
            operator.line(text)

        threading.Thread(target=type_it, daemon=True).start()


class FakeChild:
    """``dimos run`` playing the calibration module: ``script(rpc)`` runs once it starts."""

    script: Callable[[FakeRpc], None]
    rpc: FakeRpc
    signals: list[int]

    def __init__(self, argv: list[str], **kwargs: Any) -> None:
        self.argv, FakeChild.signals = argv, []
        script = FakeChild.__dict__["script"]
        self._thread = threading.Thread(target=script, args=(FakeChild.rpc,), daemon=True)
        self._thread.start()

    def poll(self) -> int | None:
        return None if self._thread.is_alive() else 0

    def wait(self) -> int:
        self._thread.join()
        return 0

    def send_signal(self, sig: int) -> None:
        FakeChild.signals.append(sig)

    def kill(self) -> None: ...


@pytest.fixture
def child(monkeypatch: pytest.MonkeyPatch) -> type[FakeChild]:
    FakeChild.rpc = FakeRpc()
    monkeypatch.setattr("dimos.core.transport_factory.rpc_backend", lambda: lambda: FakeChild.rpc)
    monkeypatch.setattr(arm_command.subprocess, "Popen", FakeChild)
    return FakeChild


@pytest.fixture
def never() -> Iterator[Iterator[str]]:
    """Lines the operator never types; they end with the test."""
    typed: queue.Queue[str | None] = queue.Queue()
    yield iter(typed.get, None)
    typed.put(None)


def test_the_module_is_answered_from_the_keys_and_the_run_stopped_when_done(
    child: type[FakeChild],
    never: Iterator[str],
) -> None:
    answers: list[str] = []

    def module(rpc: FakeRpc) -> None:
        rpc.handlers["show"]("hello")
        for key in ("x", "s"):
            rpc.answer_next(key)
        answers.append(rpc.handlers["ask"]("Move home?", ["go", "skip"]))
        rpc.answer_next("")
        answers.append(rpc.handlers["ask"]("Move home?", ["go", "skip"]))
        rpc.handlers["done"](True, "all good", True)

    child.script = module
    assert arm_command.walk(["dimos", "run", "x"], never) is True
    assert answers == ["skip", "go"]
    assert child.signals == [] or child.signals == [signal.SIGINT]
    assert child.rpc.stopped and not child.rpc.aborts.is_set()


def test_ctrl_c_aborts_and_going_home_is_offered(
    child: type[FakeChild], never: Iterator[str]
) -> None:
    answers: list[str] = []

    def module(rpc: FakeRpc) -> None:
        rpc.answer_next("")
        answers.append(rpc.handlers["ask"]("Move to pose_01?", ["go", "skip"]))
        os.kill(os.getpid(), signal.SIGINT)
        assert rpc.aborts.wait(WAIT_S)
        rpc.answer_next("")
        answers.append(rpc.handlers["ask"]("aborted. Go home?", ["home", "stay"]))
        rpc.handlers["done"](False, "aborted", True)

    child.script = module
    assert arm_command.walk(["dimos", "run", "x"], never) is False
    assert answers == ["go", "home"]


def test_q_answers_a_pending_ask_and_aborts(child: type[FakeChild], never: Iterator[str]) -> None:
    answers: list[str] = []

    def module(rpc: FakeRpc) -> None:
        rpc.answer_next("q")
        answers.append(rpc.handlers["ask"]("Move to pose_01?", ["go", "skip"]))
        assert rpc.aborts.wait(WAIT_S)
        rpc.handlers["done"](False, "aborted", True)

    child.script = module
    assert arm_command.walk(["dimos", "run", "x"], never) is False
    assert answers == ["abort"]


def test_the_run_is_stopped_away_from_home_only_once_the_operator_says_so(
    child: type[FakeChild], never: Iterator[str], capsys: pytest.CaptureFixture[str]
) -> None:
    stopped_before_the_answer: list[bool] = []

    def module(rpc: FakeRpc) -> None:
        rpc.answer_next("s")
        rpc.handlers["ask"]("Go home?", ["home", "stay"])
        operator = rpc.handlers["ask"].__self__

        answered = threading.Event()

        def answer_once_asked() -> None:
            assert operator._asking.wait(WAIT_S)
            stopped_before_the_answer.append(bool(FakeChild.signals))
            operator.line("")
            answered.set()

        threading.Thread(target=answer_once_asked, daemon=True).start()
        rpc.handlers["done"](False, "failed", False)
        # The run is still up while the operator is asked.
        assert answered.wait(WAIT_S)

    child.script = module
    assert arm_command.walk(["dimos", "run", "x"], never) is False
    assert stopped_before_the_answer == [False]
    assert "stopping the run turns its motors off" in capsys.readouterr().out


def test_a_run_that_exits_early_ends_the_walk(child: type[FakeChild], never: Iterator[str]) -> None:
    child.script = lambda rpc: rpc.handlers["show"]("starting")
    assert arm_command.walk(["dimos", "run", "x"], never) is False


def test_the_run_globals_are_resolved_as_dimos_run_resolves_them() -> None:
    from dimos.core.global_config import global_config

    arm_command.use_run_globals(
        ["--calibrationmodule.arm", "left", "--zenoh-connect", "tcp/127.0.0.1:7447"],
        {"transport": "lcm"},
    )
    assert global_config.transport == "lcm"
    assert global_config.zenoh_connect == "tcp/127.0.0.1:7447"


@pytest.fixture
def arms_root(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    monkeypatch.setattr(schema, "ARMS_STATE_DIR", tmp_path / "arms")
    return tmp_path / "arms"


def test_show_prints_the_saved_file_and_what_it_replaced(arms_root: Path) -> None:
    path = calibration_path(ARM_ID, arms_root)
    save_calibration(path, calibration(arm_id=ARM_ID, robot_model=TOY_MODEL))
    save_calibration(path, calibration(arm_id=ARM_ID, robot_model=TOY_MODEL, notes="second"))

    result = invoke("show", "dual-openyam")

    assert result.exit_code == 0, result.output
    assert f"{ARM_ID}: {path}" in result.output
    assert "created 2026-11-18 09:30 UTC for toy" in result.output
    assert "shoulder: kp 80 kd 5" in result.output and "arm: 2.000 kg" in result.output
    assert "replaced: " in result.output
    assert "dual_openyam/right" in result.output and "none:" in result.output
    only = invoke("show", "dual-openyam", "--arm", "left")
    assert only.exit_code == 0 and "dual_openyam/right" not in only.output
    assert invoke("show", "dual-openyam", "--arm", "up").exit_code == 2
    path.write_text("{")
    assert invoke("show", "dual-openyam", "--arm", "left").exit_code == 1


def test_bake_writes_the_fitted_masses_and_refuses_a_foreign_urdf(
    arms_root: Path, tmp_path: Path
) -> None:
    save_calibration(
        calibration_path(ARM_ID, arms_root), calibration(arm_id=ARM_ID, robot_model=TOY_MODEL)
    )
    source, out = tmp_path / "in.urdf", tmp_path / "out.urdf"
    source.write_text(TOY_URDF)

    result = invoke(
        "bake", "dual-openyam", "--arm", "left", "--urdf", str(source), "--out", str(out)
    )

    assert result.exit_code == 0, result.output
    assert 'value="1.5"' in out.read_text()
    source.write_text(TOY_URDF.replace('<mass value="0.3"/>', '<mass value="0.4"/>'))
    foreign = tmp_path / "x.urdf"
    refused = invoke(
        "bake", "dual-openyam", "--arm", "left", "--urdf", str(source), "--out", str(foreign)
    )
    assert refused.exit_code == 1 and "fingerprint" in refused.output and not foreign.exists()
    missing = invoke(
        "bake", "dual-openyam", "--arm", "right", "--urdf", str(source), "--out", str(out)
    )
    assert missing.exit_code == 1 and "No arm calibration" in missing.output
