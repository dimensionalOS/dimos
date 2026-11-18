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

import io
import time

import pytest

from dimos.robot.manipulators.calibration.progress import (
    CONFIRM_HINT,
    ParkRequestedError,
    ProgressBar,
    confirm,
    countdown,
)


def test_countdown_returns_the_result_and_updates_in_place() -> None:
    out = io.StringIO()

    def work() -> str:
        time.sleep(0.25)
        return "held"

    assert countdown("step 1/4", 0.3, work, stream=out) == "held"
    text = out.getvalue()
    assert text.count("\r") >= 2
    assert "s remaining" in text
    assert text.rstrip("\n").split("\r")[-1].startswith("step 1/4  done in")


def test_countdown_propagates_exceptions() -> None:
    def fail() -> None:
        raise ValueError("refused")

    with pytest.raises(ValueError, match="refused"):
        countdown("step", 0.1, fail, stream=io.StringIO())


def test_progress_bar_counts_to_total() -> None:
    out = io.StringIO()
    bar = ProgressBar(4, "report", stream=out)
    bar.update(text="a")
    bar.update(text="b")
    bar.close()
    lines = out.getvalue().split("\r")
    assert "[....................] 0/4" in lines[1]
    assert "2/4" in lines[3] and " b" in lines[3]
    assert lines[-1].startswith("report [####################] 4/4")
    assert out.getvalue().endswith("\n")


def test_confirm_enter_runs_s_skips_and_p_parks() -> None:
    prompts: list[str] = []

    def answer(text: str) -> str:
        prompts.append(text)
        return text.split("|")[0]

    assert confirm("|", True, read=answer) is True
    assert confirm("s|", True, read=answer) is False
    assert confirm(" S |", True, read=answer) is False
    with pytest.raises(ParkRequestedError):
        confirm("p|", True, read=answer)
    with pytest.raises(ParkRequestedError):
        confirm("q|", True, read=answer)
    assert all(CONFIRM_HINT in p for p in prompts)
    # Disabled prompts never read anything.
    assert confirm("x", False, read=lambda _: "p") is True
