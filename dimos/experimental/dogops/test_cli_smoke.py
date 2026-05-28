from __future__ import annotations

import subprocess
import sys


def test_cli_validate_and_simulate(tmp_path) -> None:
    validate = subprocess.run(
        [sys.executable, "-m", "dimos.experimental.dogops.cli", "validate"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "validated site=dogops_demo_site" in validate.stdout

    run_dir = tmp_path / "latest"
    simulate = subprocess.run(
        [
            sys.executable,
            "-m",
            "dimos.experimental.dogops.cli",
            "simulate",
            "--out",
            str(run_dir),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "state=done" in simulate.stdout
    assert (run_dir / "report.md").is_file()
    assert "PKG-104 wrong zone and blocking COOLING_1" in (run_dir / "report.md").read_text(
        encoding="utf-8"
    )
