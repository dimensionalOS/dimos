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

"""SPACE benchmark boundary: data, official parser and metrics, case classes.

Nothing of the benchmark is vendored (code: Apple Sample Code License; data:
CC BY-NC-ND 4.0). ``scripts/eval/space.sh`` fetches both; ``dimos[space]`` adds
the packages its parser imports. SPACE ships no packaging metadata, so the
checkout is put on ``sys.path`` here rather than declared as a dependency.

Two case types, mirroring the paper's two-stage protocol (SPACE §3.1: first a
walkthrough familiarizes the model with the environment, then it is tested):

- :class:`SpaceQA` — one multiple-choice question. ``familiarization`` holds the
  walkthrough segments, ``question`` the rest; ``--blind`` withholds exactly the
  walkthrough, which makes it the paper-faithful ablation ("can you answer
  without having seen the environment?").
- :class:`SpaceNav` — one interactive navigation episode (route retracing or
  shortcut discovery), scored with the benchmark's own SPL implementation.

Scoring never deviates from upstream: answers go through SPACE's
``parse_answer_from_response`` (called unbound — it never touches ``self``) and
``exact`` reproduces its ``float(P == qa["answer"])``; episodes are scored by
``space.envs.nav_dm.evaluate_path_efficiency`` / the habitat SPL classes.

One deliberate deviation, disclosed: SPACE's navigation agents keep a
multi-turn dialog, while :meth:`EvalRig.ask` is single-turn. Episodes therefore
resend the transcript flattened into one message per step, with the model's
prior replies quoted as text. The prompt wording itself is the benchmark's,
copied verbatim from ``space/agents/{dmnav,egonav}_agent.py``.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import json
from pathlib import Path
import random
import re
import sys
from typing import Any, cast

from dimos.evals.scorers import exact
from dimos.evals.suites.space._config import config
from dimos.evals.types import EvalCase, EvalResult, EvalRig

SETUP_HINT = "run scripts/eval/space.sh, then pip install 'dimos[space]'"


# -- checkout / dataset access --------------------------------------------------


def _repo_on_path() -> None:
    repo = config.repo_dir
    if repo.is_dir() and str(repo) not in sys.path:
        sys.path.insert(0, str(repo))


def space_parse(text: str) -> Any:
    """Normalize a reply with the benchmark's own parser, called unmodified.

    ``parse_answer_from_response`` never touches ``self`` — it is regex + json
    over the reply text — so it is callable unbound: no ``QA_Agent`` is built
    and no model client is opened. Its sibling ``postprocess_response`` *is*
    skipped on purpose; it reads ``self.model_name`` and only strips a
    Mistral-specific ``[/INST]`` prefix.
    """
    _repo_on_path()
    try:
        from space.agents.qa_agent import QA_Agent
    except ImportError as e:
        raise ImportError(f"SPACE's parser is unavailable ({e}). {SETUP_HINT}") from e
    return QA_Agent.parse_answer_from_response(None, text)


def load_qas(task: str) -> list[dict[str, Any]]:
    """Rows for one SPACE QA task, or ``[]`` when the dataset is not present.

    Returning empty keeps suites importable — and ``dimos evals list`` working —
    on a checkout that never ran the setup script.
    """
    path = config.data_dir / task / "qas.json"
    if not path.exists():
        return []
    return list(json.loads(path.read_text()))


_MEDIA_PREFIX = "data/SPACE_data_release/"


def media_path(reference: str) -> Path:
    """Resolve a ``data/SPACE_data_release/...`` reference from a question."""
    return config.data_dir / reference.removeprefix(_MEDIA_PREFIX)


# -- prompt blocks ----------------------------------------------------------------


def _image_blocks(frame_bgr: Any) -> list[dict[str, Any]]:
    """Encode one BGR frame, the layout both cv2 and ``Image`` use.

    Converting to RGB first would invert the picture: ``Image.from_numpy``
    tags the array ``ImageFormat.BGR`` and ``to_base64`` encodes it as such.
    """
    import numpy as np

    from dimos.msgs.sensor_msgs.Image import Image

    frame_bgr = np.ascontiguousarray(frame_bgr)  # channel-flip views are strided
    # agent_encode returns the block list, though it is annotated as one block
    # (Image.py carries its own ignore on the return).
    return cast("list[dict[str, Any]]", Image.from_numpy(frame_bgr).agent_encode())


def _text_block(text: str) -> dict[str, Any]:
    return {"type": "text", "text": text}


def video_frame_blocks(path: Path) -> list[dict[str, Any]]:
    """Every ``config.frame_stride``-th frame of a video, as image blocks."""
    import cv2

    capture = cv2.VideoCapture(str(path))
    blocks: list[dict[str, Any]] = []
    try:
        index = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if index % config.frame_stride == 0:
                blocks.extend(_image_blocks(frame))
            index += 1
    finally:
        capture.release()
    if not blocks:
        raise FileNotFoundError(f"no frames decoded from: {path}")
    return blocks


def segment_blocks(segments: Sequence[str]) -> list[dict[str, Any]]:
    """Question segments -> prompt blocks, in the benchmark's own order.

    ``IMAGE:``/``VIDEO:`` markers become image blocks and everything else stays
    text, so the interleaving the questions rely on ("See image below")
    survives. Deliberately not routed through a mem2 Store: ``EvalRunner.encode``
    stamps a ``[t=Ns]`` text block before every observation, which the benchmark
    never sends.
    """
    import cv2

    blocks: list[dict[str, Any]] = []
    for segment in segments:
        if segment.startswith("IMAGE:"):
            image = cv2.imread(str(media_path(segment[len("IMAGE:") :])))
            if image is None:
                raise FileNotFoundError(f"unreadable image in question: {segment}")
            blocks.extend(_image_blocks(image))
        elif segment.startswith("VIDEO:"):
            blocks.extend(video_frame_blocks(media_path(segment[len("VIDEO:") :])))
        else:
            blocks.append(_text_block(segment))
    return blocks


def split_familiarization(segments: Sequence[str]) -> tuple[tuple[str, ...], tuple[str, ...]]:
    """Split a question into (walkthrough, question) at the paper's stage boundary.

    Every media-presentation question starts with framing text and the
    walkthrough VIDEO, then asks its actual question (SPACE §3.1's two stages).
    """
    for index, segment in enumerate(segments):
        if segment.startswith("VIDEO:"):
            familiarization = tuple(segments[: index + 1])
            question = tuple(segments[index + 1 :])
            if question and not question[-1].startswith(("IMAGE:", "VIDEO:")):
                return familiarization, question
    raise ValueError(f"question does not follow the walkthrough-then-question shape: {segments!r}")


# -- prefix-safe ordering ---------------------------------------------------------

ROTATION = 4
"""Rows per position-rotation group: one question, correct option in slot 1-4."""


def spread(rows: list[dict[str, Any]]) -> list[tuple[int, dict[str, Any]]]:
    """``(original_index, row)`` reordered so any prefix is a spread sample.

    Rows ship grouped by environment, and within one in blocks of four that
    rotate a single question's correct option through positions 1-4 — the
    benchmark's guard against position bias. Read in file order a small
    ``--limit`` therefore buys a couple of environments, each question asked
    four times, and the answer key 1,2,3,4,1,2,3,4: a run that scores like
    signal and is not.

    So shuffle the groups, and hand out the variant slots from a balanced pool
    that is itself shuffled — both from one seeded generator. Stepping the slot
    instead would put the answer key straight back: a group lists its rotations
    in answer order, so slot ``n % 4`` reproduces 1,2,3,4,1,2,3,4 exactly.
    Drawing it independently at random removes the cycle but lets a short
    prefix drift off balance; dealing from a balanced pool avoids both.
    Leftover rotations follow, so nothing is dropped.
    """
    rng = random.Random(0)
    indexed = list(enumerate(rows))
    groups = [indexed[i : i + ROTATION] for i in range(0, len(indexed), ROTATION)]
    rng.shuffle(groups)
    slots = [i % ROTATION for i in range(len(groups))]
    rng.shuffle(slots)
    slots = [min(slot, len(group) - 1) for slot, group in zip(slots, groups, strict=True)]
    lead = [group[slot] for group, slot in zip(groups, slots, strict=True)]
    rest = [e for g, s in zip(groups, slots, strict=True) for i, e in enumerate(g) if i != s]
    return lead + rest


# -- passive QA case ----------------------------------------------------------------


@dataclass(frozen=True, kw_only=True)
class SpaceQA(EvalCase):
    """One SPACE multiple-choice question, scored with the benchmark's own pipeline.

    ``--blind`` withholds ``familiarization`` (the walkthrough) and keeps the
    question itself — landmark stills, choices, answer-format instruction — which
    is the paper's own ablation shape.
    """

    familiarization: tuple[str, ...] = ()
    question: tuple[str, ...] = ()
    expected: Any = None

    def evaluate(self, rig: EvalRig) -> EvalResult:
        blocks = [] if rig.blind else segment_blocks(self.familiarization)
        blocks += segment_blocks(self.question[:-1])
        outputs = rig.ask(blocks, self.question[-1])
        got = space_parse(outputs)
        return EvalResult(case_id=self.id, outputs=outputs, score=exact(self.expected, got))

    def preflight(self, rig: EvalRig) -> None:
        for segment in (*self.familiarization, *self.question):
            if segment.startswith(("IMAGE:", "VIDEO:")):
                reference = segment.split(":", 1)[1]
                if not media_path(reference).exists():
                    raise FileNotFoundError(f"{self.id}: missing {reference} — {SETUP_HINT}")


def qa_suite(task_base: str, slug: str) -> list[EvalCase]:
    """Both media presentations of one QA task, prefix-safe and tagged.

    The text presentations are deliberately not wired up: a prose walkthrough
    has no visual pathway, which is the part a robot stack cares about.
    """
    cases: list[EvalCase] = []
    for presentation, task in (("bevimage", f"{task_base}BEVImage"), ("ego", f"{task_base}Ego")):
        for index, row in spread(load_qas(task)):
            segments = [str(s) for s in row["question"]]
            familiarization, question = split_familiarization(segments)
            cases.append(
                SpaceQA(
                    id=f"space_{slug}_{presentation}_{index:04d}",
                    inputs=" ".join(s for s in question if not s.startswith(("IMAGE:", "VIDEO:"))),
                    familiarization=familiarization,
                    question=question,
                    expected=row["answer"],  # compared as-is: the benchmark does not coerce
                    tags=frozenset({"space", slug, presentation}),
                )
            )
    return cases


# -- interactive navigation case -----------------------------------------------------

# Wording below is the benchmark's own, copied verbatim from
# space/agents/dmnav_agent.py and space/agents/egonav_agent.py (their prompt
# strings live inside agent methods, so they cannot be imported directly).
_DM_WALKTHROUGH_PROMPT = {
    "shortestpath": "Here is sequence of video frames recorded in the 2D world. This demonstrates the route you need to repeat. Analyze the video to understand the movements and the world structure. Take a note of all the details needed to help you repeat this route when navigating next. Think step by step.",
    "walkthrough": "Here is the sequence of video frames recorded in the 2D world. This demonstrates a suboptimal route from the start to some goal location. Analyze the video to understand the movements and the world structure. Keep track of the start and goal locations, and the current location in the world as you watch the video. Then plan a shortcut route that takes you to the goal while avoiding any unnecessary detours. Think step by step.",
}
_DM_OBSERVATION_PROMPT = (
    "Here is the local view of your surroundings in the 2D world. "
    "You are at the center of this view."
)
_DM_ACTION_QUESTION = (
    "What action do you select next? The available actions are up, down, left, right and stop."
)
_EGO_WALKTHROUGH_PROMPT = {
    "shortestpath": "Here are the sequence of frames from the walkthrough video demonstrating the route you need to take. Analyze the walkthrough to understand the movements and the maze structure. Take a note of all the details needed to help you repeat this route when navigating next. Think step by step.",
    "walkthrough": "Here are the sequence of frames from the walkthrough video demonstrating a suboptimal route from the start to some goal location. Analyze the walkthrough to understand the movements and the environment structure. Keep track of the start and goal locations, and the current location in the environment as you watch the walkthrough. Then plan a shortcut route that takes you to the goal while avoiding unnecessary detours. Think step by step.",
}
_EGO_OBSERVATION_PROMPT = (
    "Here is the current observation. If you are stuck very close to the same wall for "
    "several steps, it means that you are colliding and need to turn around and search elsewhere."
)
_EGO_ACTION_QUESTION = (
    "What action do you select next? The available actions are move_forward, turn_left, "
    "turn_right and stop. Recall that each turn action is only 30 degrees and each forward "
    "step is only 0.25m, so you may have to execute several actions to notice substantial "
    "changes in your viewpoints. Be patient and persist with your actions over a longer "
    "time horizon."
)
# space/evaluate_egonav.py:30-36 — configuration values, not logic. Passed through
# verbatim: the action prompt promises 30-degree turns, and load_sim's default is
# 10 degrees, so dropping a key here silently breaks the sim/prompt contract.
_EGO_IMAGE_DOWNSCALING = 4
_EGO_HABITAT_CONFIG = {
    "resolution": [512 * _EGO_IMAGE_DOWNSCALING, 512 * _EGO_IMAGE_DOWNSCALING],
    "forward_amount": 0.25,
    "turn_amount": 30,
}


@dataclass(frozen=True, kw_only=True)
class SpaceNav(EvalCase):
    """One navigation episode, scored with the benchmark's own SPL.

    ``walkthrough_key`` selects the task, exactly as upstream does: the
    demonstrated route is the shortest path for route retracing
    (``"shortestpath"``) and a detour for shortcut discovery
    (``"walkthrough"``). The transcript is resent flattened each step (see the
    module docstring); wording and fallbacks — unparseable reply becomes the
    same default action upstream uses — match the benchmark's agents.
    """

    env_name: str = ""
    walkthrough_key: str = ""
    presentation: str = ""  # "bevimage" | "ego"
    timeout_s: float = 3600.0

    def evaluate(self, rig: EvalRig) -> EvalResult:
        if self.presentation == "bevimage":
            metrics = self._discrete_map_episode(rig)
        else:
            metrics = self._ego_episode(rig)
        return EvalResult(case_id=self.id, outputs=json.dumps(metrics), score=float(metrics["spl"]))

    def preflight(self, rig: EvalRig) -> None:
        if self.presentation == "bevimage":
            needed = [
                config.data_dir / "2D_scenes" / self.env_name / "info.json",
                config.data_dir
                / "BEV_image_walkthroughs"
                / self.env_name
                / f"{self.walkthrough_key}_obs.mp4",
            ]
        else:
            needed = [config.data_dir / "3D_scenes" / self.env_name / f"{self.walkthrough_key}.mp4"]
            python = config.habitat_python
            if python is None or not python.exists():
                # habitat-sim has Python 3.9 conda builds only, so it can never
                # import into this interpreter; episodes drive it in a sidecar
                # process instead (_ego_env.py).
                raise RuntimeError(
                    f"{self.id}: egocentric navigation renders observations online and "
                    "needs a habitat-sim interpreter (x86_64 Linux: micromamba create "
                    "-n habitat -c conda-forge -c aihabitat python=3.9 habitat-sim=0.3.0 "
                    "headless; then set DIMOS_SPACE_HABITAT_PYTHON to that env's "
                    "python). See docs/usage/evaluation/space.md"
                )
        for path in needed:
            if not path.exists():
                raise FileNotFoundError(f"{self.id}: missing {path.name} — {SETUP_HINT}")

    def _tracer(self, rig: EvalRig) -> Any:
        """Persist each step (observation, reply, action) under the run dir.

        The runner already promotes ``<case_id>.jsonl`` in its run dir to
        ``EvalResult.transcript`` — the same convention ``agent_loop`` uses —
        so writing the file is all it takes. Silently disabled for rigs
        without a run dir (tests' FakeRig).
        """
        run_dir = getattr(rig, "run_dir", None)
        if run_dir is None:
            return None
        import cv2

        log = (Path(run_dir) / f"{self.id}.jsonl").open("w")

        def record(step: int, observation_rgb: Any, action: str, reply: str) -> None:
            image = Path(run_dir) / f"{self.id}_step{step:03d}.jpg"
            cv2.imwrite(str(image), observation_rgb[..., ::-1])
            log.write(
                json.dumps(
                    {"step": step, "action": action, "reply": reply, "image": image.name},
                    ensure_ascii=False,
                )
                + "\n"
            )
            log.flush()

        return record

    # -- episode drivers ----------------------------------------------------------

    def _header(
        self, task_prompt: str, walkthrough: str, frames: list[dict[str, Any]], rig: EvalRig
    ) -> list[dict[str, Any]]:
        """Task prompt + walkthrough. The task prompt rides in the transcript —
        the benchmark's own fallback placement for models without a system
        channel (space/agents/basenav_agent.py:88)."""
        header = [_text_block(task_prompt), _text_block(walkthrough)]
        if not rig.blind:
            header += frames
        return header

    def _act(
        self,
        rig: EvalRig,
        transcript: list[dict[str, Any]],
        question: str,
        valid: tuple[str, ...],
        fallback: str,
    ) -> tuple[str, str]:
        reply = rig.ask(transcript, question)
        _repo_on_path()
        from space.agents.dmnav_agent import Base_DiscreteMap_Nav

        # Upstream's regex first, for fidelity — but it demands whitespace after
        # the brace, so the compact ```json {"action":"turn_right"}``` modern
        # models emit parses to None, and the fallback then walks the episode in
        # circles while the replies reason correctly. Action extraction is
        # agent-side plumbing (upstream keeps it in its agent classes too), so a
        # tolerant pass runs before the fallback; scoring is untouched.
        action = Base_DiscreteMap_Nav.convert_response_to_action(None, reply)
        if action not in valid:
            for candidate in re.findall(r"\{[^{}]*\}", reply):
                try:
                    parsed = json.loads(candidate)
                except json.JSONDecodeError:
                    continue
                if isinstance(parsed, dict) and parsed.get("action") in valid:
                    action = str(parsed["action"])
                    break
            else:
                action = fallback  # upstream's own default for an unusable reply
        return action, reply

    def _discrete_map_episode(self, rig: EvalRig) -> dict[str, Any]:
        _repo_on_path()
        import numpy as np
        from space.agents.dmnav_agent import (
            IMAGE_TASK_PROMPT_NOVEL_SHORTCUTS,
            IMAGE_TASK_PROMPT_ROUTE_FOLLOWING,
        )
        from space.envs.nav_dm import NavDiscreteMapEnv, evaluate_path_efficiency

        scene = config.data_dir / "2D_scenes" / self.env_name
        walkthrough_dir = config.data_dir / "BEV_image_walkthroughs" / self.env_name
        env = NavDiscreteMapEnv(str(scene / "info.json"), config.nav_local_context, "image")
        gt_path = json.loads((walkthrough_dir / "shortestpath_info.json").read_text())["positions"]
        task_prompt = (
            IMAGE_TASK_PROMPT_ROUTE_FOLLOWING
            if self.walkthrough_key == "shortestpath"
            else IMAGE_TASK_PROMPT_NOVEL_SHORTCUTS
        )
        transcript = self._header(
            task_prompt,
            _DM_WALKTHROUGH_PROMPT[self.walkthrough_key],
            video_frame_blocks(walkthrough_dir / f"{self.walkthrough_key}_obs.mp4"),
            rig,
        )
        transcript.append(
            _text_block(
                "Now, you must navigate to the goal based on your knowledge of the 2D "
                f"world you obtained from the video. Here is the goal description: "
                f"landmark {env.goal_name}"
            )
        )

        observation = env.reset()
        pred_path = [list(env.current_location)]
        stop_issued = False
        trace = self._tracer(rig)
        for step in range(config.nav_max_steps):
            transcript.append(_text_block(_DM_OBSERVATION_PROMPT))
            transcript += _image_blocks(observation[..., ::-1])  # env renders RGB
            action, reply = self._act(
                rig,
                transcript,
                _DM_ACTION_QUESTION,
                ("up", "down", "left", "right", "stop"),
                "up",
            )
            if trace is not None:
                trace(step, observation, action, reply)
            transcript.append(_text_block(f"You responded:\n{reply}"))
            if action == "stop":
                stop_issued = True
                break
            observation = env.step(action)
            pred_path.append(list(env.current_location))

        grid = np.array([[0 if col == "0" else 1 for col in row] for row in env.textmap])
        return cast(
            "dict[str, Any]",
            evaluate_path_efficiency(
                [(r, c) for c, r in gt_path],
                [(r, c) for c, r in pred_path],
                grid,
                stop_issued,
                dist_thresh=1.5,
            ),
        )

    def _ego_episode(self, rig: EvalRig) -> dict[str, Any]:
        """Drive the habitat environment in its own interpreter (see _ego_env.py).

        The env and the benchmark's metric classes run under the habitat
        Python; prompts, the model loop and action parsing stay here.
        """
        import base64
        import collections
        import io
        import subprocess
        import threading

        import numpy as np

        _repo_on_path()
        from space.agents.egonav_agent import (
            TASK_PROMPT_NOVEL_SHORTCUTS,
            TASK_PROMPT_ROUTE_FOLLOWING,
        )

        scene = config.data_dir / "3D_scenes" / self.env_name
        server = subprocess.Popen(
            [str(config.habitat_python), str(Path(__file__).parent / "_ego_env.py")],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        # habitat is chatty on stderr (and the sidecar redirects its stdout
        # there); left unread, a full 64 KiB pipe deadlocks both processes.
        # Drain it continuously, keeping a bounded tail for error reports.
        stderr_tail: collections.deque[str] = collections.deque(maxlen=50)

        def _drain() -> None:
            assert server.stderr is not None
            for text_line in server.stderr:
                stderr_tail.append(text_line)

        threading.Thread(target=_drain, daemon=True).start()

        def call(**request: Any) -> dict[str, Any]:
            assert server.stdin is not None and server.stdout is not None
            server.stdin.write(json.dumps(request) + "\n")
            server.stdin.flush()
            line = server.stdout.readline()
            if not line:
                raise RuntimeError(
                    f"{self.id}: habitat env server died on {request['cmd']}: "
                    f"{''.join(stderr_tail)[-800:]}"
                )
            return cast("dict[str, Any]", json.loads(line))

        def decode_obs(payload: str) -> Any:
            return np.load(io.BytesIO(base64.b64decode(payload)), allow_pickle=False)

        try:
            task_info = call(
                cmd="init",
                scene=str(scene),
                repo=str(config.repo_dir),
                habitat=_EGO_HABITAT_CONFIG,
                image_downscaling=_EGO_IMAGE_DOWNSCALING,
            )["task_info"]
            frames = video_frame_blocks(scene / f"{self.walkthrough_key}.mp4")
            task_prompt = (
                TASK_PROMPT_ROUTE_FOLLOWING
                if self.walkthrough_key == "shortestpath"
                else TASK_PROMPT_NOVEL_SHORTCUTS
            )
            transcript = self._header(
                task_prompt, _EGO_WALKTHROUGH_PROMPT[self.walkthrough_key], frames, rig
            )
            transcript.append(
                _text_block(
                    "Now, you must navigate to the goal. Here is the goal description "
                    f"and the image: {task_info['goal_desc']}"
                )
            )
            transcript += frames[-1:]  # upstream shows the last walkthrough frame as the goal

            observation = decode_obs(call(cmd="reset")["obs"])
            stop_issued = False
            trace = self._tracer(rig)
            for step in range(config.nav_ego_max_steps):
                transcript.append(_text_block(_EGO_OBSERVATION_PROMPT))
                transcript += _image_blocks(observation[..., ::-1])  # habitat renders RGB
                action, reply = self._act(
                    rig,
                    transcript,
                    _EGO_ACTION_QUESTION,
                    ("move_forward", "turn_left", "turn_right", "stop"),
                    "turn_left",
                )
                if trace is not None:
                    trace(step, observation, action, reply)
                transcript.append(_text_block(f"You responded:\n{reply}"))
                if action == "stop":
                    stop_issued = True
                    break
                observation = decode_obs(call(cmd="step", action=action)["obs"])

            metrics = call(cmd="finish", stop_issued=stop_issued)["metrics"]
            if server.stdin is not None:  # close has no reply; fire and forget
                server.stdin.write(json.dumps({"cmd": "close"}) + "\n")
                server.stdin.flush()
            return cast("dict[str, Any]", metrics)
        finally:
            if server.stdin is not None:
                server.stdin.close()
            server.wait(timeout=30)


def nav_suite(walkthrough_key: str, slug: str) -> list[EvalCase]:
    """One episode per environment, both presentations, tagged like the QA suites."""
    cases: list[EvalCase] = []
    for presentation, scenes_dir in (("bevimage", "2D_scenes"), ("ego", "3D_scenes")):
        root = config.data_dir / scenes_dir
        if not root.is_dir():
            continue
        scenes = sorted(
            (p.name for p in root.iterdir() if p.is_dir()),
            # Instances of one base env share a layout; interleaving them keeps
            # a --limit prefix layout-diverse instead of N copies of one map.
            key=lambda name: (name.rsplit("_", 1)[-1], name),
        )
        for scene in scenes:
            cases.append(
                SpaceNav(
                    id=f"space_{slug}_{presentation}_{scene}",
                    inputs=f"{slug} in {scene}",
                    env_name=scene,
                    walkthrough_key=walkthrough_key,
                    presentation=presentation,
                    tags=frozenset({"space", slug, presentation}),
                )
            )
    return cases
