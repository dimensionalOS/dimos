"""Native LeRobot learning adapter contracts."""

import sys

from dimos_lerobot.learning_train import emit, load_check
import pytest


def test_native_adapter_loads():
    import dimos_lerobot.learning_train as adapter

    assert adapter.ACTPolicy.__module__ == "lerobot.policies.act.modeling_act"
    assert "dimos.robot" not in sys.modules
    assert "dimos.core.coordination" not in sys.modules


def test_load_check_rejects_missing(tmp_path):
    with pytest.raises(Exception):
        load_check({"path": str(tmp_path)})


def test_progress_message(capsys):
    emit("native progress", completed=1, total=2)
    import json

    assert json.loads(capsys.readouterr().out)["completed"] == 1
