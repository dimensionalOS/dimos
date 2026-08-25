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

"""SPACE suite wiring — offline except where fidelity needs the real benchmark.

Most tests fake the `space` package in `sys.modules`, which is enough to prove
the raw reply reaches the benchmark's parser and the parsed value is scored
with `exact`. Tests that need the dataset or the checkout skip cleanly when
they are absent.
"""

from __future__ import annotations

import collections
import itertools
import sys
from types import ModuleType
from typing import Any

import pytest

from dimos.evals.scorers import exact
from dimos.evals.suites.space._bench import (
    ROTATION,
    SETUP_HINT,
    SpaceNav,
    SpaceQA,
    load_qas,
    media_path,
    segment_blocks,
    space_parse,
    split_familiarization,
    spread,
)
from dimos.evals.suites.space._config import config
from dimos.evals.test_evals import FakeRig


def _install_fake_space(monkeypatch: pytest.MonkeyPatch, parsed: Any) -> list[str]:
    """Fake `space.agents.qa_agent.QA_Agent`, recording the text it is given."""
    seen: list[str] = []

    class FakeQaAgent:
        @staticmethod
        def parse_answer_from_response(_self: Any, text: str) -> Any:
            seen.append(text)
            return parsed

    package = ModuleType("space")
    agents = ModuleType("space.agents")
    qa_agent = ModuleType("space.agents.qa_agent")
    qa_agent.QA_Agent = FakeQaAgent  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "space", package)
    monkeypatch.setitem(sys.modules, "space.agents", agents)
    monkeypatch.setitem(sys.modules, "space.agents.qa_agent", qa_agent)
    return seen


def _case(expected: Any) -> SpaceQA:
    return SpaceQA(
        id="space_case",
        inputs="Here are your choices: 1) 4.5  2) 8.5",
        familiarization=(),
        question=("Here are your choices: 1) 4.5  2) 8.5",),
        expected=expected,
    )


# -- boundary to the benchmark --------------------------------------------------


def test_space_parse_uses_the_official_parser(monkeypatch: pytest.MonkeyPatch) -> None:
    seen = _install_fake_space(monkeypatch, parsed=3)
    assert space_parse('reasoning... {"answer": 3}') == 3
    assert seen == ['reasoning... {"answer": 3}'], "the raw reply must reach the parser"


def test_space_parse_names_the_setup_when_the_package_is_missing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    for name in [m for m in sys.modules if m == "space" or m.startswith("space.")]:
        monkeypatch.delitem(sys.modules, name)
    monkeypatch.setitem(sys.modules, "space", None)
    with pytest.raises(ImportError, match=r"space\.sh"):
        space_parse('{"answer": 1}')


# -- QA case wiring ----------------------------------------------------------------


@pytest.mark.parametrize(
    ("parsed", "expected", "score"),
    [
        (2, 2, 1.0),
        (4, 2, 0.0),
        (None, 1, 0.0),  # benchmark returns None when it finds no answer; None != 1
    ],
)
def test_qa_case_scores(
    monkeypatch: pytest.MonkeyPatch, parsed: Any, expected: Any, score: float
) -> None:
    _install_fake_space(monkeypatch, parsed=parsed)
    assert _case(expected=expected).evaluate(FakeRig(answer="reply")).score == score


def test_blind_withholds_walkthrough_but_keeps_the_question(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """`--blind` is the paper's ablation: the walkthrough goes, the question stays."""
    _install_fake_space(monkeypatch, parsed=1)

    class BlindRig(FakeRig):
        blind = True

        def ask(self, context: Any, question: str) -> str:
            self.calls.append("ask")
            self.blocks = list(context)
            return self.answer

    case = SpaceQA(
        id="c",
        inputs="q",
        familiarization=("intro", "VIDEO:data/SPACE_data_release/missing.mp4"),
        question=("the question text", "final instruction"),
        expected=1,
    )
    rig = BlindRig(answer="reply")
    assert case.evaluate(rig).score == 1.0
    texts = [b["text"] for b in rig.blocks]
    assert texts == ["the question text"], "walkthrough withheld, question kept"


# -- the walkthrough/question split ------------------------------------------------


def test_split_familiarization_shape() -> None:
    segments = ["intro", "VIDEO:data/x.mp4", "IMAGE:data/y.jpg", "choices", "json format"]
    familiarization, question = split_familiarization(segments)
    assert familiarization == ("intro", "VIDEO:data/x.mp4")
    assert question == ("IMAGE:data/y.jpg", "choices", "json format")
    with pytest.raises(ValueError):
        split_familiarization(["no video here"])


def test_every_dataset_question_splits(monkeypatch: pytest.MonkeyPatch) -> None:
    """The two-stage shape must hold for every row of every wired task."""
    found = False
    for task_base in ("DirectionEstimation", "DistanceEstimation", "MapSketching"):
        for presentation in ("BEVImage", "Ego"):
            rows = load_qas(f"{task_base}{presentation}")
            for row in rows:
                familiarization, question = split_familiarization([str(s) for s in row["question"]])
                assert familiarization[-1].startswith("VIDEO:")
                assert question, row
            found = found or bool(rows)
    if not found:
        pytest.skip("SPACE dataset not present")


# -- prefix-safe ordering -----------------------------------------------------------


def test_suite_prefix_is_a_spread_sample() -> None:
    """`--limit N` must not hand back one corner of the dataset.

    Rows ship in groups of four that rotate one question's correct option
    through positions 1-4. In file order a prefix is a handful of environments
    with the answer key 1,2,3,4,1,2,3,4..., which scores like signal and is not.
    """
    rows = load_qas("DistanceEstimationBEVImage")
    if not rows:
        pytest.skip("SPACE dataset not present")

    out = spread(rows)
    assert len(out) == len(rows) and len({i for i, _ in out}) == len(rows)
    prefix = [row for _, row in out[:20]]
    answers = collections.Counter(row["answer"] for row in prefix)
    assert max(answers.values()) <= 3 * min(answers.values()), answers
    assert len({row["metadata"]["env_dir"] for row in prefix}) >= 10
    questions = {
        (row["metadata"]["env_dir"], str(row["metadata"].get("gt_choice"))) for row in prefix
    }
    assert len(questions) == len(prefix), "a prefix must not repeat one question's rotations"


def test_lead_answers_are_not_a_cycle() -> None:
    """Balanced is not enough: a repeating key scores like signal too."""
    rows = load_qas("DirectionEstimationBEVImage")
    if not rows:
        pytest.skip("SPACE dataset not present")
    answers = [row["answer"] for _, row in spread(rows)[:40]]
    assert answers[:ROTATION] * (len(answers) // ROTATION) != answers, "answer key repeats"


# -- media fidelity -----------------------------------------------------------------


def test_frames_keep_their_colours() -> None:
    """A frame must reach the model as shot.

    `Image.from_numpy` tags arrays `ImageFormat.BGR` and `to_base64` encodes
    them as such, so converting cv2's output to RGB first would silently swap
    red and blue in every frame of every walkthrough.
    """
    import base64

    import cv2
    import numpy as np

    rows = load_qas("DirectionEstimationEgo")  # this task ships landmark stills
    if not rows:
        pytest.skip("SPACE dataset not present")
    segment = next(s for s in rows[0]["question"] if s.startswith("IMAGE:"))

    source = cv2.imread(str(media_path(segment[len("IMAGE:") :])))
    block = segment_blocks([segment])[0]
    payload = block["image_url"]["url"].split(",", 1)[1]
    decoded = cv2.imdecode(np.frombuffer(base64.b64decode(payload), np.uint8), cv2.IMREAD_COLOR)

    # JPEG is lossy, so compare channel means rather than pixels.
    assert np.allclose(decoded.mean(axis=(0, 1)), source.mean(axis=(0, 1)), atol=6)


def test_question_blocks_keep_the_benchmark_order() -> None:
    """Media expands in place; the text around it stays put and gains nothing."""
    rows = load_qas("DistanceEstimationBEVImage")
    if not rows:
        pytest.skip("SPACE dataset not present")
    segments = [str(s) for s in rows[0]["question"]][:-1]
    blocks = segment_blocks(segments)
    assert [b["text"] for b in blocks if b["type"] == "text"] == [
        s for s in segments if not s.startswith(("IMAGE:", "VIDEO:"))
    ]
    assert blocks[0]["type"] == "text", "the framing text leads, as upstream has it"
    assert sum(b["type"] == "image_url" for b in blocks) > 1, "the video must expand to frames"


# -- navigation episodes --------------------------------------------------------------


def test_nav_preflight_reports_whats_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    case = SpaceNav(
        id="nav",
        inputs="nav",
        env_name="NoSuchEnv_00000",
        walkthrough_key="shortestpath",
        presentation="bevimage",
    )
    with pytest.raises(FileNotFoundError, match=SETUP_HINT.split(",")[0].split()[-1]):
        case.preflight(FakeRig())


def test_nav_ego_preflight_names_habitat(monkeypatch: pytest.MonkeyPatch) -> None:
    if config.data_dir.joinpath("3D_scenes").is_dir() is False:
        pytest.skip("SPACE dataset not present")
    try:
        import habitat_sim  # noqa: F401

        pytest.skip("habitat available here; the error path cannot fire")
    except ImportError:
        pass
    scene = sorted(p.name for p in (config.data_dir / "3D_scenes").iterdir() if p.is_dir())[0]
    case = SpaceNav(
        id="nav",
        inputs="nav",
        env_name=scene,
        walkthrough_key="shortestpath",
        presentation="ego",
    )
    with pytest.raises(RuntimeError, match="habitat-sim"):
        case.preflight(FakeRig())


def test_nav_episode_scores_with_the_official_spl() -> None:
    """Drive a real discrete-map episode with a scripted rig.

    Replaying the ground-truth shortest path must score SPL 1.0; stopping on
    the spot must score 0.0 — both judged by the benchmark's own metric.
    """
    scenes = config.data_dir / "2D_scenes"
    if not scenes.is_dir():
        pytest.skip("SPACE dataset not present")
    try:
        space_parse("probe")  # ensures the checkout imports (networkx etc.)
    except ImportError as e:
        pytest.skip(str(e))

    import json as json_module

    scene = sorted(p.name for p in scenes.iterdir() if p.is_dir())[0]
    walkthrough_dir = config.data_dir / "BEV_image_walkthroughs" / scene
    gt = json_module.loads((walkthrough_dir / "shortestpath_info.json").read_text())["positions"]

    def action_between(a: list[int], b: list[int]) -> str:
        if b[0] > a[0]:
            return "right"
        if b[0] < a[0]:
            return "left"
        if b[1] > a[1]:
            return "down"
        return "up"

    perfect = [action_between(a, b) for a, b in itertools.pairwise(gt)] + ["stop"]

    class ScriptedRig(FakeRig):
        def __init__(self, script: list[str]) -> None:
            super().__init__()
            self.script = list(script)

        def ask(self, context: Any, question: str) -> str:
            return f'{{\n  "action": "{self.script.pop(0)}"\n}}'

    case = SpaceNav(
        id="nav",
        inputs="nav",
        env_name=scene,
        walkthrough_key="shortestpath",
        presentation="bevimage",
    )
    assert case.evaluate(ScriptedRig(perfect)).score == pytest.approx(1.0)
    lazy = case.evaluate(ScriptedRig(["stop"] * 2))
    assert lazy.score == 0.0


# -- fidelity: needs the real benchmark package ---------------------------------


def test_scoring_matches_the_official_metric() -> None:
    """`exact(answer, space_parse(reply))` must equal SPACE's own accuracy term.

    The benchmark computes `float(P == qa["answer"]) * 100.0` in
    `space/evaluate_qas.py`, where `P` is `parse_answer_from_response(reply)`.
    """
    try:
        space_parse("probe")  # puts the checkout on sys.path, or explains how to get it
    except ImportError as e:
        pytest.skip(str(e))
    from space.agents.qa_agent import QA_Agent

    samples: list[tuple[str, Any]] = [
        ('{"answer": 1}', 1),
        ('{"answer": 1}', 2),
        ('thinking...\n```\n{"answer": 3}\n```', 3),
        ('{"answer": "2"}', 2),
        ("no json at all", 1),
        ('{"answer": null}', 1),
        ('{"answer": 2} then {"answer": 4}', 4),
    ]
    for reply, answer in samples:
        official = float(QA_Agent.parse_answer_from_response(None, reply) == answer)
        assert exact(answer, space_parse(reply)) == official, reply
