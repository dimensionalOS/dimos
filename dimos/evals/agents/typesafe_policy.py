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

"""TypeSafe (Jev) as a closed-loop navigation policy.

The model calls no skill. Each tick: code assembles the world state, Jev picks
one body-frame command (forward, backward, turn left, turn right, stop), code
scales it into a Twist and republishes it for the tick. The scene (obstacles,
walls, goal) is a static ground-truth snapshot of the simulated house (a DimSim
detections export, or a Habitat scene file in the same layout); the robot pose
comes live off the simulator's odometry topic.

Two contracts are meant to be edited: :class:`WorldState` (what Jev sees) and
:data:`STEP_CRITERIA` / :func:`build_questions` (what Jev answers). Everything
else is plumbing.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
import re
import threading
import time
from typing import TYPE_CHECKING, Any, Literal

from dimos.evals.agents.base import Agent, AgentConfig
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.types import EndedBy, Metrics, RunningEnvironment, Trajectory
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist

if TYPE_CHECKING:
    from dimos.evals.environments.base import Environment


# --- input contract: what Jev sees each tick ---------------------------------
# One global world frame: DimSim's scene origin, ROS Z-up, meters, the frame
# odometry is published in. Obstacles are DimSim ground-truth boxes with real
# extents, walls included; there is no object/obstacle distinction, every box
# is something to avoid.
#
# Note: Jev is documented as unreliable at numeric comparison
# (docs.typesafe.ai/model-jaggedness/jev-1.13). Raw coordinates are a
# deliberate choice; the scripted baselines are what tell us whether it works.


@dataclass(frozen=True, kw_only=True)
class Obstacle:
    label: str
    min_xy: tuple[float, float]
    max_xy: tuple[float, float]


@dataclass(frozen=True, kw_only=True)
class WorldState:
    frame_id: str = "world"  # the global origin everything is expressed in
    robot_xy: tuple[float, float] = (0.0, 0.0)
    robot_yaw_deg: float = 0.0
    goal_label: str = ""
    goal_xy: tuple[float, float] = (0.0, 0.0)
    # minx, miny, maxx, maxy. The centre is inside the object; arrival means
    # touching this box.
    goal_box: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    room_bounds: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    obstacles: list[Obstacle] = field(default_factory=list)
    ticks_elapsed: int = 0

    def encode(self) -> dict[str, Any]:
        """The ``state`` payload sent to system_one."""
        return asdict(self)


# --- output contract: what Jev answers ---------------------------------------
# Options are body-frame (linear.x, angular.z) pairs keyed by their literal
# values: the pick IS the Twist, scaled by speed / turn_rate. There is no
# controller in between, so Jev reasons about its own heading (robot_yaw_deg)
# itself. DimSim's ground model integrates exactly these two axes.

STEPS: dict[str, tuple[float, float]] = {  # (linear.x, angular.z)
    "0,0": (0.0, 0.0),
    "1,0": (1.0, 0.0),
    "-1,0": (-1.0, 0.0),
    "0,1": (0.0, 1.0),
    "0,-1": (0.0, -1.0),
}

# ROS convention, which DimSim's physics follows: +angular.z turns left
# (counter-clockwise), yaw increases.
STEP_CRITERIA: dict[str, str] = {
    "0,0": "Stop.",
    "1,0": "Drive forward along the current heading.",
    "-1,0": "Drive backward.",
    "0,1": "Turn left in place; yaw increases.",
    "0,-1": "Turn right in place; yaw decreases.",
}


def build_questions() -> dict[str, Any]:
    """One fan-out call per tick: the step plus the termination flag."""
    from typesafe_sdk import Choice, Noul

    return {
        "step": Choice(
            instructions=(
                "Which command moves the robot toward the goal without entering an obstacle "
                "box. The robot faces robot_yaw_deg (0 = +x, 90 = +y); forward is along "
                "that heading."
            ),
            criteria=STEP_CRITERIA,
        ),
        "reached": Noul(instructions="The robot has reached the goal box and should stop"),
    }


# --- scene file --------------------------------------------------------------
# DimSim ground truth: the snapshot ``SceneClient.get_object_detections()``
# writes through ``detection3d_array_to_dict`` (PR #4208), already in the ROS
# ``world`` frame odometry uses. Static for now; the live call returns the
# same schema, so switching is a one-line change in ``run()``.

Box2D = tuple[float, float, float, float]  # minx, miny, maxx, maxy

# DimSim's agent is a capsule (halfHeight 0.25 + radius 0.12) resting on the
# floor. Anything whose box bottom is above its top passes overhead: door
# headers, wall cabinets, the TV.
ROBOT_TOP_M = 0.74
# A footprint inside another kept footprint (books on a shelf, a plate on a
# cart) adds nothing for navigation, and Jev's accuracy falls with irrelevant
# state. The tolerance absorbs a throw blanket overhanging its bed, and a chair
# tucked under its table (the table's footprint already blocks that spot).
CONTAINMENT_TOL_M = 0.1
WALL_PREFIXES = ("wall", "yard")


@dataclass(frozen=True, kw_only=True)
class Scene:
    frame_id: str
    goal_label: str
    goal_xy: tuple[float, float]
    goal_box: Box2D
    room_bounds: Box2D
    obstacles: list[Obstacle]


def load_scene(path: Path, goal_label: str) -> Scene:
    """Detections JSON -> the 2D scene Jev sees. ``goal_label`` is a substring."""
    if not goal_label:
        raise ValueError("goal_label must name the goal; an empty string matches everything")
    raw = json.loads(Path(path).expanduser().read_text())
    goal = next(
        (d for d in raw["detections"] if goal_label.lower() in str(d["label"]).lower()), None
    )
    if goal is None:
        raise LookupError(f"no detection labelled like {goal_label!r} in {path}")
    grounded = [d for d in raw["detections"] if _box_bottom(d) < ROBOT_TOP_M]
    boxes = {str(d["id"]): _footprint(d) for d in grounded}
    kept = [d for d in grounded if not _contained(str(d["id"]), boxes)]
    walls = [boxes[str(d["id"])] for d in kept if str(d["label"]).startswith(WALL_PREFIXES)]
    bounds = walls or list(boxes.values())
    return Scene(
        frame_id=str(raw.get("frame_id", "world")),
        goal_label=str(goal["label"]),
        goal_xy=(float(goal["center_xyz"][0]), float(goal["center_xyz"][1])),
        goal_box=_footprint(goal),
        room_bounds=(
            min(b[0] for b in bounds),
            min(b[1] for b in bounds),
            max(b[2] for b in bounds),
            max(b[3] for b in bounds),
        ),
        obstacles=[
            Obstacle(
                label=str(d["label"]),
                min_xy=(boxes[str(d["id"])][0], boxes[str(d["id"])][1]),
                max_xy=(boxes[str(d["id"])][2], boxes[str(d["id"])][3]),
            )
            for d in kept
        ],
    )


def scene_labels(path: Path) -> list[str]:
    raw = json.loads(Path(path).expanduser().read_text())
    return [str(d["label"]) for d in raw["detections"]]


# Instruction words that also occur inside object labels ("with chrome",
# "wall-kitchen-front") but never name the goal.
_STOP_WORDS = frozenset(
    {
        "back",
        "drive",
        "find",
        "front",
        "from",
        "goal",
        "head",
        "into",
        "left",
        "navigate",
        "near",
        "next",
        "over",
        "please",
        "reach",
        "right",
        "robot",
        "side",
        "then",
        "there",
        "toward",
        "towards",
        "walk",
        "with",
    }
)


def derive_goal_label(instruction: str, labels: Sequence[str]) -> str:
    """The longest word of the instruction that occurs in some scene label.

    Jev does no language grounding here: the instruction has to call the goal
    by a word from its label ("bathtub", "sectional"), and code does the lookup
    so the goal box in the state is always a real object. Letting the model
    pick the goal would be a separate Choice question and a separate eval.
    """
    lowered = [label.lower() for label in labels]
    words = re.findall(r"[a-z][a-z-]{3,}", instruction.lower())
    hits = [w for w in words if w not in _STOP_WORDS and any(w in label for label in lowered)]
    if not hits:
        raise LookupError(
            f"no word of {instruction!r} names a scene object; use a word from its label"
        )
    return max(hits, key=len)


def _footprint(d: dict[str, Any]) -> Box2D:
    (cx, cy, _), (sx, sy, _) = d["center_xyz"], d["size_xyz"]
    return (
        round(cx - sx / 2, 3),
        round(cy - sy / 2, 3),
        round(cx + sx / 2, 3),
        round(cy + sy / 2, 3),
    )


def _box_bottom(d: dict[str, Any]) -> float:
    return float(d["center_xyz"][2] - d["size_xyz"][2] / 2)


def _inside(a: Box2D, b: Box2D, tol: float) -> bool:
    return a[0] >= b[0] - tol and a[1] >= b[1] - tol and a[2] <= b[2] + tol and a[3] <= b[3] + tol


def _contained(own_id: str, boxes: dict[str, Box2D]) -> bool:
    """Whether the box lies inside another footprint that does not lie inside it."""
    box = boxes[own_id]
    return any(
        other_id != own_id
        and _inside(box, other, CONTAINMENT_TOL_M)
        and not _inside(other, box, CONTAINMENT_TOL_M)
        for other_id, other in boxes.items()
    )


# --- the agent ---------------------------------------------------------------


class TypeSafePolicyConfig(AgentConfig):
    model: str = "jev-latest"  # typesafe_sdk.constants.DEFAULT_MODEL
    scene_json: Path = Path()  # required; a DimSim detections snapshot, see load_scene
    goal_label: str = ""  # a word of the goal's label; empty derives it from the instruction
    max_ticks: int = 60
    tick_s: float = 1.0
    # The SDK default is 10s; a tick that blocks longer than this is dead time.
    request_timeout_s: float = 10.0
    # DimSim scales linear and angular commands by 3x (DEFAULT_SPEED_SCALE /
    # DEFAULT_TURN_SCALE in misc/DimSim/cli/bridge/physics.ts).
    speed: float = 0.2  # m/s for forward/backward; ~0.6 m/s in-sim, ~0.6 m per tick
    turn_rate: float = 0.5  # rad/s for turns; ~86 deg/s in-sim, ~a quarter turn per tick
    # Republish interval inside a tick; must beat the sim's 500 ms cmd_vel deadman.
    control_dt: float = 0.1
    reached_noul: float = 0.8
    # The environment should already have waited for the sim; this is a backstop.
    pose_wait_s: float = 60.0
    # Where the pose comes from and where commands go. Unset, preflight takes the
    # simulator's conventions (see ``Wiring``): DimSim's go2 stack publishes a
    # PoseStamped on /odom and MovementManager owns /cmd_vel, so commands go
    # upstream of it on /nav_cmd_vel; Habitat publishes nav_msgs Odometry on
    # /odometry and takes Twist straight on /cmd_vel.
    odom_topic: str | None = None
    odom_msg: Literal["PoseStamped", "Odometry"] | None = None
    cmd_topic: str | None = None
    # DimSim scales commands 3x in-sim (see ``speed``); Habitat integrates them
    # as given, so it gets the same factor here for the same motion per tick.
    speed_scale: float | None = None


@dataclass(frozen=True)
class Wiring:
    """How the policy plugs into one simulator's stack."""

    odom_topic: str
    odom_msg: Literal["PoseStamped", "Odometry"]
    cmd_topic: str
    speed_scale: float


DIMSIM_WIRING = Wiring("/odom", "PoseStamped", "/nav_cmd_vel", 1.0)
HABITAT_WIRING = Wiring("/odometry", "Odometry", "/cmd_vel", 3.0)


def wiring_for(environment: Environment, config: TypeSafePolicyConfig) -> Wiring:
    """The simulator's conventions, with any explicit config field on top."""
    from dimos.evals.environments.habitat import HabitatEnvironment

    base = HABITAT_WIRING if isinstance(environment, HabitatEnvironment) else DIMSIM_WIRING
    return Wiring(
        odom_topic=config.odom_topic or base.odom_topic,
        odom_msg=config.odom_msg or base.odom_msg,
        cmd_topic=config.cmd_topic or base.cmd_topic,
        speed_scale=base.speed_scale if config.speed_scale is None else config.speed_scale,
    )


class TypeSafePolicy(Agent):
    """Drive the robot with Jev decisions. Publishes Twist, calls no skill."""

    config: TypeSafePolicyConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._pose: PoseStamped | None = None
        self._pose_seen = threading.Event()
        self._wiring = DIMSIM_WIRING

    def preflight(self, environment: Environment) -> None:
        if self.config.modules:
            raise ValueError("TypeSafePolicy calls no tools; leave modules empty")
        self._wiring = wiring_for(environment, self.config)
        if not self.config.scene_json.is_file():
            raise FileNotFoundError(f"scene_json not found: {self.config.scene_json}")
        scene_labels(self.config.scene_json)  # fail before anything starts
        if self.config.goal_label:
            load_scene(self.config.scene_json, self.config.goal_label)

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        from typesafe_sdk import TypeSafeClient

        from dimos.core.transport_factory import make_transport

        raw = run_dir / "raw"
        raw.mkdir(parents=True, exist_ok=True)
        trajectory = TrajectoryBuilder(inputs, name=type(self).__name__, model=self.config.model)

        goal_label = self.config.goal_label or derive_goal_label(
            inputs, scene_labels(self.config.scene_json)
        )
        scene = load_scene(self.config.scene_json, goal_label)
        # api_key comes from TYPESAFE_API_KEY (typesafe_sdk.constants.API_KEY_ENV).
        client = TypeSafeClient(model=self.config.model, timeout=self.config.request_timeout_s)
        questions = build_questions()

        wiring = self._wiring
        odom: Any
        if wiring.odom_msg == "Odometry":
            from dimos.msgs.nav_msgs.Odometry import Odometry

            odom = make_transport(wiring.odom_topic, Odometry)
        else:
            odom = make_transport(wiring.odom_topic, PoseStamped)
        cmd = make_transport(wiring.cmd_topic, Twist)
        for transport in (odom, cmd):
            transport.start()
        odom.subscribe(self._on_odometry if wiring.odom_msg == "Odometry" else self._on_odom)

        deadline = time.monotonic() + timeout_s
        ended: EndedBy = "max_steps"
        try:
            if not self._pose_seen.wait(min(self.config.pose_wait_s, timeout_s)):
                raise TimeoutError(f"no pose on {wiring.odom_topic}")
            for tick in range(self.config.max_ticks):
                if time.monotonic() >= deadline:
                    ended = "timeout"
                    break

                _, state = self.observe(scene, tick)
                started = time.time()
                response = client.system_one(state=state.encode(), questions=questions)
                self._trace(trajectory, raw, tick, state, response, started)

                if response.answers["reached"].noul > self.config.reached_noul:
                    ended = "answer"
                    break

                # Hold Jev's command for the whole tick, republishing every
                # control_dt: the sim zeroes velocity 500 ms after the last cmd_vel.
                twist = self.twist(response.answers["step"])
                hold_until = min(deadline, time.monotonic() + self.config.tick_s)
                while time.monotonic() < hold_until:
                    cmd.publish(twist)
                    time.sleep(self.config.control_dt)
        finally:
            cmd.publish(Twist.zero())
            for transport in (odom, cmd):
                transport.stop()
        return trajectory.build(ended)

    def _on_odom(self, pose: PoseStamped) -> None:
        self._pose = pose
        self._pose_seen.set()

    def _on_odometry(self, odom: Any) -> None:
        """nav_msgs Odometry carries the same pose; keep its frame and stamp."""
        self._on_odom(
            PoseStamped(
                ts=odom.ts,
                frame_id=odom.frame_id,
                position=odom.position,
                orientation=odom.orientation,
            )
        )

    def observe(self, scene: Scene, tick: int) -> tuple[PoseStamped, WorldState]:
        """Static scene + live pose -> the state Jev sees."""
        pose = self._pose
        if pose is None:
            raise LookupError("no pose yet")
        return pose, WorldState(
            frame_id=scene.frame_id,
            robot_xy=(pose.position.x, pose.position.y),
            robot_yaw_deg=math.degrees(_yaw(pose)),
            goal_label=scene.goal_label,
            goal_xy=scene.goal_xy,
            goal_box=scene.goal_box,
            room_bounds=scene.room_bounds,
            obstacles=scene.obstacles,
            ticks_elapsed=tick,
        )

    def twist(self, step: Any) -> Twist:
        """Jev's pick is the body-frame Twist: (linear.x, angular.z), scaled.

        DimSim's ground model integrates only ``linear.x`` (along the heading)
        and ``angular.z``; ``linear.y`` is ignored, so these are the only two
        axes there are. Nothing about heading is computed here, and there is
        no confidence threshold: ``choice`` is the argmax and it is applied as
        is. Only a "0,0" pick (or the episode ending) stops the robot.
        """
        x, w = STEPS[str(step.choice)]
        scale = self._wiring.speed_scale
        return Twist(
            linear=(scale * self.config.speed * x, 0.0, 0.0),
            angular=(0.0, 0.0, scale * self.config.turn_rate * w),
        )

    def _trace(
        self,
        trajectory: TrajectoryBuilder,
        raw: Path,
        tick: int,
        state: WorldState,
        response: Any,
        started: float,
    ) -> None:
        """Persist the call and record one ATIF step.

        The runner counts ``raw/NNN-request.json`` for request_attempts, so the
        naming matters.
        """
        request_path = raw / f"{tick:03d}-request.json"
        response_path = raw / f"{tick:03d}-response.json"
        request_path.write_text(
            json.dumps({"body": {"state": state.encode()}, "started_at": started}, indent=2)
        )
        answers = {name: _answer_json(a) for name, a in response.answers.items()}
        response_path.write_text(
            json.dumps({"model": response.model, "answers": answers}, indent=2, default=str)
        )
        usage = response.usage
        trajectory.step(
            message=json.dumps(answers),
            request=request_path,
            response=response_path,
            # What actually ran, as reported by the provider.
            model_name=str(response.model),
            metrics=Metrics(
                prompt_tokens=usage.input_tokens,
                completion_tokens=usage.output_tokens,
            ),
            at=started,
            latency_s=time.time() - started,
        )


def _answer_json(answer: Any) -> dict[str, Any]:
    """Flatten a Jev answer for the trace; graders read these back."""
    return {
        name: getattr(answer, name)
        for name in ("choice", "score", "noul", "confidence", "probabilities")
        if getattr(answer, name, None) is not None
    }


def _yaw(pose: PoseStamped) -> float:
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
