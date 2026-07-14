# Copyright 2025-2026 Dimensional Inc.
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

"""Master VQA (visual question answering) e2e test.

Generalizes the single-question pattern in test_dimos_cli_e2e.py's
test_dimos_skills: a natural-language request goes to the in-graph LLM
agent (McpClient), which must pick and call the right registered skill and
return an answer -- the full MCP tool-calling loop over LCM. This test loops
that pattern over a whole bank of questions against one running robot/scene
and writes every {question, answer} pair to a JSON file.

Flow:
  1. Start a scene-serving blueprint (McpServer + McpClient +
     ObjectSceneRegistrationModule + SpatialMemory). Three scene sources are
     exercised (see the three test functions below):
       - test_master_vqa: live mujoco sim, spun in place at spawn. Simple,
         fast, but the sim scene has very little for perception to see.
       - test_master_vqa_china_office: GO2Connection's built-in `--replay`
         backend replaying a real ~137s recorded office walkthrough
         (dataset "go2_china_office", auto-fetched via LFS) -- a large,
         complex, real space with many real objects to catalog.
       - test_master_vqa_china_office_full_rrd: the full 6-level china-office
         recording (~19.6 min), fed live from a raw .rrd + sibling mem2.db
         via dimos/e2e_tests/rrd_feed.py onto `unitree-go2-scene-memory-feed`
         (no robot connection module at all -- neither file is natively
         readable by GO2Connection's own backends). Local-files-only, skips
         if the session directory isn't present.
  2. Give perception something to see: spin in place (mujoco), wait out the
     replay's duration (curated china office), or feed the .rrd live (full
     china office) -- the recorded trajectory already tours the space in the
     latter two, no live driving needed/possible.
  3. Feed each question from fixtures/vqa_questions.txt over /human_input,
     one at a time, and capture the agent's final textual answer from the
     /agent message stream once McpClient reports agent_idle again.
  4. Write the full Q&A transcript to a `*.ignore.json` file next to this
     one (the `.ignore.` infix keeps run output out of git, see
     .gitignore's `**/*.ignore.*`).

Add more questions by editing fixtures/vqa_questions.txt -- see that file's
header for the current wishlist-skill coverage and how to extend it.
"""

from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path
import pickle
import time
from typing import Any

from langchain_core.messages import AIMessage, BaseMessage
import pytest

from dimos.core.transport import LCMTransport
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.e2e_tests.rrd_feed import (
    feed_rrd_live,
    iter_mem2_camera_frames,
    load_gt_session,
    session_keepalive,
)
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.testing.waiting import wait_until

QUESTIONS_FILE = Path(__file__).parent / "fixtures" / "vqa_questions.txt"
# Recorded-session variant: no ego-"right now" questions (nearest obstacle,
# clearance) -- against a replayed tour there is no meaningful current robot
# position, so those only belong in the live/sim set above.
RECORDED_QUESTIONS_FILE = Path(__file__).parent / "fixtures" / "vqa_questions_recorded.txt"
RESULTS_FILE = Path(__file__).parent / "vqa_results.ignore.json"
CHINA_RESULTS_FILE = Path(__file__).parent / "vqa_results_china_office.ignore.json"

AGENT_TOPIC = "/agent"
AGENT_IDLE_TOPIC = "/agent_idle"

DEFAULT_ANSWER_TIMEOUT_S = 90.0
# Max time ask() waits for a previous (timed-out) turn to finish before
# sending the next question — see the settle step at the top of ask().
STALE_TURN_SETTLE_S = 30.0

# Spin in place at spawn rather than driving to scripted waypoints. Waypoint
# navigation (as movement-focused e2e tests do with explore_office()) needs
# knowledge of the scene's open floor space; blindly picking nearby
# coordinates risks wedging the robot against unseen geometry with no
# collision recovery (direct /cmd_vel driving has none). Spinning in place
# never leaves the spawn footprint, so it's collision-safe by construction,
# while still giving perception several distinct viewing angles.
MUJOCO_START_POS = "-10.75 -6.78"
SPIN_ANGULAR_SPEED = 0.6  # rad/s
SPIN_DURATION_S = 12.0  # ~1.1 full rotations at SPIN_ANGULAR_SPEED
SPIN_PUBLISH_RATE_HZ = 10.0

# GO2Connection's built-in replay backend (dimos/robot/unitree/go2/connection.py
# ReplayConnection / dimos/memory2/replay.py): `--replay --replay-db <name>`
# feeds recorded lidar/color/odom through the exact same topics a live robot
# would, at the dataset's native pace (no live speed/seek/loop control wired
# through the CLI). "go2_china_office" is a real ~137s indoor office
# walkthrough (982 lidar frames, 1951 camera frames, auto-fetched via LFS as
# data/go2_china_office.db on first use) -- much richer than the mujoco sim
# scene, with many real objects for perception to catalog.
CHINA_OFFICE_REPLAY_DB = "go2_china_office"
LIDAR_TOPIC = "/lidar#sensor_msgs.PointCloud2"
# The go2_china_office recording has one genuine ~11.2s gap in /lidar
# publishing partway through (a real pause during capture, confirmed by
# inspecting inter-frame deltas in the dataset -- every other gap is <1s),
# so this needs real margin above that or quiet-detection false-triggers on
# it mid-replay, well before the recording is actually done.
REPLAY_QUIET_S = 20.0  # no new /lidar message for this long -> replay has ended
REPLAY_MAX_WAIT_S = 400.0  # safety net well above the ~137s recording
REPLAY_SETTLE_S = 5.0  # slack for the last frame's detection/embedding to finish

# Full 6-level china-office recording (~19.6 min, 558 camera / 964 lidar
# frames) -- local files, not fetched via LFS, so this scenario skips
# gracefully if they're not present on the machine running the test. See
# dimos/e2e_tests/rrd_feed.py for why this needs its own feeder (neither the
# .rrd nor this session's mem2.db is directly readable by GO2Connection's
# own replay/live backends) and the "odom" == "world" approximation it makes.
RRD_SESSION_DIR = Path("/home/dimos/Desktop/2026-06-12_03-26am-PST__china_office1")
RRD_PATH = RRD_SESSION_DIR / "2026-06-12_0326am-PST.rrd"
MEM2_DB_PATH = RRD_SESSION_DIR / "mem2.db"
RRD_RESULTS_FILE = Path(__file__).parent / "vqa_results_china_office_full_rrd.ignore.json"
RRD_FEED_SETTLE_S = 20.0  # slack for the tail of the feed to finish processing
# Camera frames come straight from mem2.db's color_image stream (same clock as
# the gt_pointlio pose/scan the feeder already uses), decimated to this rate --
# see rrd_feed.iter_mem2_camera_frames. The .rrd is no longer needed for the
# feed; only mem2.db must be present.
MEM2_CAMERA_HZ = 1.0


def spin_in_place(
    duration_s: float = SPIN_DURATION_S,
    angular_speed: float = SPIN_ANGULAR_SPEED,
) -> None:
    """Rotate the robot in place so perception sees the scene from several
    angles, without the collision risk of driving to unverified coordinates.
    """
    cmd_vel: LCMTransport[Twist] = LCMTransport("/cmd_vel", Twist)
    cmd_vel.start()
    spin = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, angular_speed))
    stop = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
    try:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            cmd_vel.broadcast(None, spin)
            time.sleep(1.0 / SPIN_PUBLISH_RATE_HZ)
    finally:
        cmd_vel.broadcast(None, stop)
        cmd_vel.stop()


def wait_for_replay_to_finish(
    lcm_spy: LcmSpy,
    quiet_s: float = REPLAY_QUIET_S,
    max_wait_s: float = REPLAY_MAX_WAIT_S,
) -> None:
    """Wait until the china-office replay stops producing new /lidar
    messages, instead of sleeping a fixed duration guessed from the
    recording's nominal length.

    The replay's playback clock (dimos/memory2/replay.py: Replay._resolve_anchor)
    pins (wall_t0, replay_t0) at the *first* stream subscribe, which happens
    early in GO2Connection.start() -- well before on_system_modules fires,
    which only completes once every other module (YOLO-E, CLIP, ChromaDB,
    McpServer, McpClient's own tool-fetch + agent creation) has also finished
    starting. That startup tail is not fixed: cold model caches, GPU
    contention, etc. all change how long it takes. A sleep timed from
    on_system_modules -- rather than from the replay's own anchor -- can
    therefore massively under- or over-shoot the real ~137s window (in the
    worst case, most of the recording plays out *during* startup, before
    anything is listening for it, leaving only a short, sparse tail to
    actually catalog). Watching for the topic to go quiet sidesteps the
    guesswork entirely: it's done exactly when the replay is done, no matter
    how long startup took.
    """
    lcm_spy.save_topic(LIDAR_TOPIC)

    print(f"[master_vqa] waiting for {LIDAR_TOPIC} to go quiet for {quiet_s}s (replay finished)")
    deadline = time.monotonic() + max_wait_s
    last_count = 0
    stable_since: float | None = None
    while time.monotonic() < deadline:
        count = len(lcm_spy.messages.get(LIDAR_TOPIC, []))
        now = time.monotonic()
        if count != last_count:
            last_count = count
            stable_since = now if count > 0 else None
        elif count > 0 and stable_since is not None and now - stable_since >= quiet_s:
            print(f"[master_vqa] replay finished: {count} lidar frames observed")
            time.sleep(REPLAY_SETTLE_S)
            return
        time.sleep(1.0)

    raise TimeoutError(
        f"{LIDAR_TOPIC} never went quiet for {quiet_s}s within {max_wait_s}s "
        f"(saw {last_count} message(s) total) -- replay may be stuck or never started"
    )


def load_questions(path: Path) -> list[str]:
    """Parse the question template: one question per line.

    Blank lines and lines starting with '#' (comment headers) are ignored.
    """
    questions = []
    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        questions.append(line)
    return questions


def decode_agent_messages(raw_messages: list[bytes]) -> list[BaseMessage]:
    """Unpickle a slice of raw /agent LCM payloads (McpClient uses a pickled
    pLCMTransport for this topic) back into LangChain messages."""
    return [pickle.loads(m) for m in raw_messages]


def final_answer(messages: list[BaseMessage]) -> str:
    """The turn's answer: the last AIMessage carrying real text content.

    A single turn's /agent stream includes intermediate AIMessages that only
    carry tool_calls (no content) and ToolMessages with raw tool output; the
    actual answer to show a human is the final AIMessage after any tool
    calls have resolved, so scan from the end for the first one with content.
    """
    for msg in reversed(messages):
        if isinstance(msg, AIMessage) and isinstance(msg.content, str) and msg.content.strip():
            return msg.content
    return ""


def tool_calls_used(messages: list[BaseMessage]) -> list[str]:
    """Names of every tool the agent invoked while answering one question."""
    names = []
    for msg in messages:
        if isinstance(msg, AIMessage):
            for call in msg.tool_calls or []:
                names.append(call["name"])
    return names


# Approximate OpenAI list prices in USD per 1M tokens (input, output), for a
# rough credit-cost estimate only -- update when pricing or the model changes.
# Matched by longest model-name prefix so e.g. "gpt-4o-2024-08-06" -> gpt-4o
# but "gpt-4o-mini-..." -> the mini rate. Priced as of 2025.
_OPENAI_PRICES_PER_1M = {
    "gpt-4o-mini": (0.15, 0.60),
    "gpt-4o": (2.50, 10.00),
    "gpt-4.1-mini": (0.40, 1.60),
    "gpt-4.1": (2.00, 8.00),
}


def _price_for(model_name: str) -> tuple[float, float] | None:
    best, best_len = None, -1
    for prefix, price in _OPENAI_PRICES_PER_1M.items():
        if model_name.startswith(prefix) and len(prefix) > best_len:
            best, best_len = price, len(prefix)
    return best


def token_usage(messages: list[BaseMessage]) -> dict[str, Any]:
    """Total OpenAI token usage (and est. USD cost) across one turn's messages.

    ChatOpenAI attaches ``usage_metadata`` (input/output token counts) and
    ``response_metadata['model_name']`` to each AIMessage it returns; a single
    question can span several model calls (one per tool-calling round), so we
    sum them. Cost is a static-price estimate, None when the model is unknown.
    """
    inp = out = 0
    model_name = ""
    for msg in messages:
        um = getattr(msg, "usage_metadata", None)
        if um:
            inp += int(um.get("input_tokens", 0) or 0)
            out += int(um.get("output_tokens", 0) or 0)
        mn = (getattr(msg, "response_metadata", {}) or {}).get("model_name")
        if mn:
            model_name = mn
    price = _price_for(model_name) if model_name else None
    cost = (inp / 1e6 * price[0] + out / 1e6 * price[1]) if price else None
    return {
        "input_tokens": inp,
        "output_tokens": out,
        "total_tokens": inp + out,
        "model": model_name,
        "cost_usd": round(cost, 4) if cost is not None else None,
    }


def aggregate_usage(results: list[dict[str, Any]]) -> dict[str, Any]:
    """Grand-total token usage + est. cost across a whole Q&A pass."""
    inp = sum(r.get("tokens", {}).get("input_tokens", 0) for r in results)
    out = sum(r.get("tokens", {}).get("output_tokens", 0) for r in results)
    model_name = next(
        (r["tokens"]["model"] for r in results if r.get("tokens", {}).get("model")), ""
    )
    price = _price_for(model_name) if model_name else None
    cost = (inp / 1e6 * price[0] + out / 1e6 * price[1]) if price else None
    return {
        "input_tokens": inp,
        "output_tokens": out,
        "total_tokens": inp + out,
        "model": model_name,
        "cost_usd": round(cost, 4) if cost is not None else None,
        "questions": len(results),
    }


def ask(
    lcm_spy: LcmSpy,
    human_input: Callable[[str], None],
    question: str,
    timeout: float = DEFAULT_ANSWER_TIMEOUT_S,
) -> dict[str, Any]:
    """Send one question over /human_input and capture the agent's answer.

    Waits for McpClient's `agent_idle` output to publish `True` again (see
    McpClient._process_message: it publishes False when it starts working a
    queued message and True once the queue drains), then reads back every
    /agent message published during that turn to extract the final answer.
    """
    # Settle any still-running turn before sending. After a timed-out
    # question the agent may still be working it; asking the next question
    # immediately would let the PREVIOUS turn's late idle=True satisfy this
    # turn's completion check (mispairing answers) and would fold its tail
    # messages into this turn's tool-call/token accounting.
    idle_msgs = lcm_spy.messages.get(AGENT_IDLE_TOPIC, [])
    if idle_msgs and pickle.loads(idle_msgs[-1]) is not True:
        try:
            wait_until(
                lambda: (m := lcm_spy.messages.get(AGENT_IDLE_TOPIC, []))
                and pickle.loads(m[-1]) is True,
                timeout=STALE_TURN_SETTLE_S,
            )
        except TimeoutError:
            pass  # best effort: proceed, accounting may include the stale tail

    start_agent = len(lcm_spy.messages.get(AGENT_TOPIC, []))
    start_idle = len(lcm_spy.messages.get(AGENT_IDLE_TOPIC, []))

    t0 = time.monotonic()
    human_input(question)

    def turn_complete() -> bool:
        idle_msgs = lcm_spy.messages.get(AGENT_IDLE_TOPIC, [])[start_idle:]
        return any(pickle.loads(m) is True for m in idle_msgs)

    wait_until(
        turn_complete,
        timeout=timeout,
        message=f"Timed out waiting for an answer to: {question!r}",
    )
    elapsed_s = time.monotonic() - t0

    new_messages = decode_agent_messages(lcm_spy.messages.get(AGENT_TOPIC, [])[start_agent:])

    return {
        "question": question,
        "answer": final_answer(new_messages),
        "tool_calls": tool_calls_used(new_messages),
        "duration_s": round(elapsed_s, 2),
        "tokens": token_usage(new_messages),
    }


def run_qa_pass(
    lcm_spy: LcmSpy,
    human_input: Callable[[str], None],
    results_file: Path,
    questions_file: Path = QUESTIONS_FILE,
    answer_timeout_s: float = DEFAULT_ANSWER_TIMEOUT_S,
) -> list[dict[str, Any]]:
    """Ask every question in ``questions_file``, writing the transcript
    incrementally after each one (not once at the end -- if a single
    question times out, ask() raises and a batched write would never
    happen at all, silently leaving a *previous* run's stale file on disk
    with no indication anything went wrong), and assert every question got
    a non-empty answer. Shared by the scene variants below -- what differs
    is how the scene gets populated, and which question set fits it (live
    vs recorded, see RECORDED_QUESTIONS_FILE).
    """
    questions = load_questions(questions_file)
    assert questions, f"No questions found in {questions_file}"

    print(f"[master_vqa] writing results incrementally to {results_file}")

    results: list[dict[str, Any]] = []
    for question in questions:
        try:
            result = ask(lcm_spy, human_input, question, timeout=answer_timeout_s)
        except Exception as e:
            result = {
                "question": question,
                "answer": "",
                "tool_calls": [],
                "duration_s": None,
                "error": f"{type(e).__name__}: {e}",
            }
        results.append(result)
        results_file.write_text(json.dumps(results, indent=2))
        print(f"[master_vqa] Q: {question}\n[master_vqa] A: {result['answer'] or result.get('error')}")

    print(f"[master_vqa] wrote {len(results)} result(s) to {results_file}")

    # Grand-total token/credit usage for the whole pass -- printed and written
    # to a sibling *.usage.ignore.json so the dashboard can show what the run
    # cost. (Kept out of results_file so its shape stays a flat Q&A list.)
    usage = aggregate_usage(results)
    usage_file = results_file.with_name(
        results_file.name.replace(".ignore.json", ".usage.ignore.json")
    )
    usage_file.write_text(json.dumps(usage, indent=2))
    cost_str = f"${usage['cost_usd']:.4f}" if usage["cost_usd"] is not None else "n/a"
    print(
        f"[master_vqa] token usage: {usage['total_tokens']:,} total "
        f"({usage['input_tokens']:,} in / {usage['output_tokens']:,} out) "
        f"on {usage['model'] or 'unknown'} ~= {cost_str} over {usage['questions']} question(s); "
        f"wrote {usage_file}"
    )

    unanswered = [r["question"] for r in results if not r["answer"]]
    assert not unanswered, (
        f"{len(unanswered)}/{len(results)} question(s) got no answer: {unanswered}. "
        f"Full transcript: {results_file}"
    )
    return results


@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.mujoco
@pytest.mark.timeout(900)
def test_master_vqa(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[..., DimosCliCall],
    human_input: Callable[[str], None],
) -> None:
    lcm_spy.save_topic(AGENT_TOPIC)
    lcm_spy.save_topic(AGENT_IDLE_TOPIC)
    lcm_spy.save_topic("/rpc/McpClient/on_system_modules/res")

    start_blueprint(
        "--mujoco-start-pos",
        MUJOCO_START_POS,
        "run",
        "unitree-go2-scene-memory-agentic",
    )

    lcm_spy.wait_for_saved_topic("/rpc/McpClient/on_system_modules/res", timeout=120.0)
    time.sleep(5)

    # Spin in place so perception gets a few distinct viewpoints before the
    # Q&A pass (see spin_in_place's docstring for why not scripted waypoints).
    spin_in_place()

    run_qa_pass(lcm_spy, human_input, RESULTS_FILE)


@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.self_hosted
@pytest.mark.timeout(900)
def test_master_vqa_china_office(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[..., DimosCliCall],
    human_input: Callable[[str], None],
) -> None:
    """Same harness as test_master_vqa, but the scene is a real ~137s
    recorded office walkthrough instead of an empty mujoco room -- exercises
    perception (detect/list/locate/catalog_scene/refine) against a large,
    complex space with many real objects. Marked self_hosted (not mujoco):
    it needs no simulator, just downloads a real LFS dataset + a GPU for
    inference, matching this repo's convention for that kind of heavy test.
    """
    lcm_spy.save_topic(AGENT_TOPIC)
    lcm_spy.save_topic(AGENT_IDLE_TOPIC)
    lcm_spy.save_topic("/rpc/McpClient/on_system_modules/res")
    # Registered before start_blueprint so no early /lidar messages are
    # dropped -- the replay's clock starts ticking the moment GO2Connection
    # subscribes, which can be well before this fixture is otherwise ready.
    lcm_spy.save_topic(LIDAR_TOPIC)

    start_blueprint(
        "--replay",
        "--replay-db",
        CHINA_OFFICE_REPLAY_DB,
        "run",
        "unitree-go2-scene-memory-agentic",
    )

    lcm_spy.wait_for_saved_topic("/rpc/McpClient/on_system_modules/res", timeout=120.0)

    # No live driving here -- the recorded trajectory already tours the
    # space. Wait for the replay to actually finish (not a fixed sleep --
    # see wait_for_replay_to_finish) so perception has seen the whole
    # walkthrough before asking.
    wait_for_replay_to_finish(lcm_spy)

    run_qa_pass(lcm_spy, human_input, CHINA_RESULTS_FILE, questions_file=RECORDED_QUESTIONS_FILE)


@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.self_hosted
# ~7 min lossless feed + Q&A including two floorplan questions that each run
# a 60s live-collection window plus the drawing pipeline.
@pytest.mark.timeout(1800)
def test_master_vqa_china_office_full_rrd(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[..., DimosCliCall],
    human_input: Callable[[str], None],
) -> None:
    """Same harness again, but against the full 6-level china-office
    recording (2026-06-12_0326am-PST.rrd + its sibling mem2.db) instead of
    the curated 137s go2_china_office slice -- a much larger, richer real
    space (558 camera / 964 lidar frames spanning ~19.6 minutes of touring).

    Uses `unitree-go2-scene-memory-feed` (no robot connection at all) plus
    `rrd_feed.feed_rrd_live`, which publishes camera/lidar/tf onto the same
    LCM topics a live GO2Connection would, decoded straight from the .rrd
    with pose pulled from mem2.db (see rrd_feed.py's module docstring for
    why two files, and the "odom" == "world" approximation this implies --
    positions from this scenario are approximate, not precise).

    Skips if the session directory isn't present -- these are large local
    files (~1.9 GB together), not fetched via LFS like go2_china_office.db.
    """
    if not MEM2_DB_PATH.exists():
        pytest.skip(f"china-office mem2.db not found under {RRD_SESSION_DIR}")

    lcm_spy.save_topic(AGENT_TOPIC)
    lcm_spy.save_topic(AGENT_IDLE_TOPIC)
    lcm_spy.save_topic("/rpc/McpClient/on_system_modules/res")
    # Processed-frame counter: the module publishes one detections_2d message
    # per camera frame it actually runs YOLO-E on, so this count vs the
    # feeder's pairable_camera_frames stat shows whether frames were dropped.
    lcm_spy.save_topic("/detections_2d#vision_msgs.Detection2DArray")

    start_blueprint(
        "run",
        "unitree-go2-scene-memory-feed",
    )

    lcm_spy.wait_for_saved_topic("/rpc/McpClient/on_system_modules/res", timeout=120.0)

    print(f"[master_vqa] feeding {MEM2_DB_PATH.name} color_image @ ~{MEM2_CAMERA_HZ}Hz (+ gt_pointlio pose/scan)")
    session = load_gt_session(MEM2_DB_PATH)
    stats = feed_rrd_live(
        None,
        MEM2_DB_PATH,
        session=session,
        cam_frames=iter_mem2_camera_frames(MEM2_DB_PATH, target_hz=MEM2_CAMERA_HZ),
    )
    processed = len(lcm_spy.messages.get("/detections_2d#vision_msgs.Detection2DArray", []))
    print(f"[master_vqa] feed done: {stats}")
    print(
        f"[master_vqa] frames processed by YOLO-E: {processed}"
        f"/{stats.camera_frames_published} published"
    )
    time.sleep(RRD_FEED_SETTLE_S)

    # Keep /global_map + /odom alive during Q&A so live-collecting skills
    # (generate_floorplan, LidarSignalSkills) have data to answer from --
    # the recorded questions include floorplan/levels/size queries.
    with session_keepalive(session):
        run_qa_pass(
            lcm_spy,
            human_input,
            RRD_RESULTS_FILE,
            questions_file=RECORDED_QUESTIONS_FILE,
            # The floorplan questions legitimately take minutes: a 60s live
            # lidar-collection window plus the drawing pipeline, and the
            # agent often follows a long tool result with a second turn.
            answer_timeout_s=300.0,
        )
