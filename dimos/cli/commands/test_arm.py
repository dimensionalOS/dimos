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
from pathlib import Path
from typing import Any

import pytest
from typer.testing import CliRunner

from dimos.cli.commands import arm as arm_command
from dimos.cli.dimos import main
from dimos.robot.manipulators.calibration.schema import calibration_path, save_calibration
from dimos.robot.manipulators.calibration.test_schema import TOY_MODEL, TOY_URDF, calibration

ARM_ID = "dual_openyam/left"
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
        "progress": None,
        "steps": steps,
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


class FakeModule:
    """Plays a list of statuses; a waiting one stays until it is answered."""

    def __init__(self, script: list[dict[str, Any]]) -> None:
        self.script = script
        self.index = 0
        self.answered = False
        self.calls: list[tuple[str, Any]] = []
        self.home_script = [
            status("running") | {"step": "home"},
            status("waiting_confirm", prompt="The arm will ramp to home", prompt_id=50),
            status("done") | {"message": "the arm is at home"},
        ]

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

    def run(self, steps: list[str] | None = None) -> dict[str, Any]:
        self.calls.append(("run", steps))
        return self.script[0]

    def status(self) -> dict[str, Any]:
        current = self.script[self.index]
        if current["state"] == "waiting_confirm" and not self.answered:
            return current
        if self.index + 1 < len(self.script):
            self.index += 1
            self.answered = False
        return self.script[self.index]

    def results(self) -> dict[str, Any]:
        return RESULTS

    def confirm(self, prompt_id: int | None = None) -> bool:
        self.calls.append(("confirm", prompt_id))
        self.answered = True
        return True

    def skip(self, prompt_id: int | None = None) -> bool:
        self.calls.append(("skip", prompt_id))
        self.answered = True
        return True

    def abort(self) -> bool:
        self.calls.append(("abort", None))
        self.script = [
            status("aborted", gravity_fit="aborted") | {"message": "aborted: the arm holds"}
        ]
        self.index = 0
        return True

    def go_home(self) -> dict[str, Any]:
        self.calls.append(("go_home", None))
        self.script, self.index, self.answered = self.home_script, 0, False
        return self.script[0]


class ScriptedKeys:
    def __init__(self, lines: list[str]) -> None:
        self.lines = list(lines)

    def drain(self) -> None:
        pass

    def get(self) -> object:
        return self.lines.pop(0) if self.lines else arm_command.EOF


class Rig:
    def __init__(self) -> None:
        self.module = FakeModule([])
        self.keys: list[str] = []
        self.busy: list[arm_command.ActiveRun] = []
        self.launched: list[list[str]] = []
        self.stopped: list[str] = []


@pytest.fixture
def rig(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Rig:
    rig = Rig()
    monkeypatch.setattr(arm_command, "POLL_S", 0.0)
    monkeypatch.setattr(arm_command, "dimos_command", lambda: ["dimos"])
    monkeypatch.setattr(arm_command, "active_runs", lambda: rig.busy)

    def launch(argv: list[str]) -> str:
        rig.launched.append(argv)
        return "20261118-101010-abcd-dual-openyam-calibrate"

    monkeypatch.setattr(arm_command, "launch_run", launch)

    def stop(run_id: str) -> str:
        rig.stopped.append(run_id)
        return "stopped"

    monkeypatch.setattr(arm_command, "stop_run", stop)
    monkeypatch.setattr(arm_command, "connect_module", lambda timeout: rig.module)
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


def test_calibrate_prints_the_plan_confirms_the_right_prompt_and_stops_only_its_run(
    rig: Rig,
) -> None:
    rig.module.script = happy_script()
    rig.keys = ["", "s", "", ""]

    result = invoke("calibrate", "dual-openyam")

    out = result.output
    assert result.exit_code == 0, out
    assert "✓ check: Check the arm" in out
    assert "x gravity_fit: Fit gravity  (cannot run: no recording)" in out
    assert [c for c in rig.module.calls if c[0] != "run"] == [("confirm", 7), ("skip", 8)]
    assert rig.module.calls[0] == ("run", None)
    assert "Move to home?" in out and "Save and apply?" in out
    # The results of a finished step come as a table.
    assert "left_joint2  0.500 -> 0.100" in out
    assert "at a bound" in out and "left_joint3" in out and "FAIL" in out
    assert "Calibration file: /state/arms/dual_openyam/left/calibration.json" in out
    assert "history/" in out and "calibration off" in out
    assert rig.stopped == ["20261118-101010-abcd-dual-openyam-calibrate"]
    [argv] = rig.launched
    assert argv == [
        "dimos",
        "run",
        "dual-openyam-calibrate",
        "--daemon",
        "--calibrationmodule.arm",
        "left",
    ]


def test_q_aborts_and_only_our_run_is_stopped(rig: Rig) -> None:
    rig.module.script = happy_script()
    rig.keys = ["q", ""]

    result = invoke("calibrate", "dual-openyam", "--steps", "check,float_check")

    assert result.exit_code == 130, result.output
    assert ("abort", None) in rig.module.calls and (
        "run",
        ["check", "float_check"],
    ) in rig.module.calls
    assert "Run aborted" in result.output
    assert rig.stopped == ["20261118-101010-abcd-dual-openyam-calibrate"]
    assert not any(call[0] == "go_home" for call in rig.module.calls)


def test_a_failed_run_exits_non_zero_and_h_goes_home_after_its_own_confirmation(rig: Rig) -> None:
    rig.module.script = [
        status("running"),
        status("failed", check="done") | {"message": "Gravity poses failed: stalled"},
    ]
    rig.keys = ["h", ""]

    result = invoke("calibrate", "dual-openyam")

    assert result.exit_code == 1, result.output
    assert "Run failed: Gravity poses failed: stalled" in result.output
    assert [c for c in rig.module.calls if c[0] in ("go_home", "confirm")] == [
        ("go_home", None),
        ("confirm", 50),
    ]
    assert "the arm is at home" in result.output
    assert len(rig.stopped) == 1


def test_ctrl_c_aborts_waits_for_the_engine_and_stops_the_run(
    rig: Rig, monkeypatch: pytest.MonkeyPatch
) -> None:
    rig.module.script = happy_script()
    rig.keys = [""]
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
    assert ("abort", None) in rig.module.calls
    assert "Ctrl-C: aborting" in result.output
    assert rig.stopped == ["20261118-101010-abcd-dual-openyam-calibrate"]


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
    assert rig.launched == [] and rig.stopped == []


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
    assert rig.launched == []


def test_stdin_keys_reads_lines_and_forgets_what_was_typed_ahead() -> None:
    keys = arm_command.StdinKeys(io.StringIO("\nS\nq\n"))
    for _ in range(100):
        if keys._lines.qsize() >= 4:
            break
        import time

        time.sleep(0.01)
    keys.drain()
    assert keys.get() is arm_command.EOF
    assert keys.get() is arm_command.EOF


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
    only = invoke("show", "dual-openyam", "--arm", "left")
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
        "bake", "dual-openyam", "--arm", "left", "--urdf", str(source), "--out", str(out)
    )

    assert result.exit_code == 0, result.output
    assert 'value="1.5"' in out.read_text()
    source.write_text(TOY_URDF.replace('<mass value="0.3"/>', '<mass value="0.4"/>'))
    refused = invoke(
        "bake",
        "dual-openyam",
        "--arm",
        "left",
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


def test_every_forwarded_flag_parses_on_the_real_blueprint() -> None:
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.robot.get_all_blueprints import load_blueprint

    parser = BlueprintConfigParser(load_blueprint("dual-openyam-calibrate"))
    parsed = parser.parse(
        [
            "--calibrationmodule.arm",
            "left",
            "--calibrationmodule.floor-z-m",
            "-0.03",
            "--calibrationmodule.min-x-m",
            "0.12",
            "--calibrationmodule.poses",
            "6",
            "--calibrationmodule.speed-deg-s",
            "10.0",
            "--calibrationmodule.confirm-each=false",
            "--dualopenyamconnection.left-can-port",
            "follower_l",
            "--dualopenyamconnection.right-can-port",
            "follower_r",
        ],
        config_path=None,
        environ={},
        global_overrides={},
    )
    assert parsed is not None
