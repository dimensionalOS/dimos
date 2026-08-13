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

"""Run and score closed-loop episodes.

python -m dimos.navigation.motion.control --ls                 # scenario names
python -m dimos.navigation.motion.control --view -s corridor   # watch one
python -m dimos.navigation.motion.control --view --gen 8 -s gen003   # a generated one
python -m dimos.navigation.motion.control --score              # curated battery
python -m dimos.navigation.motion.control --score --gen 8      # + generated
"""

import argparse
import json
import sys

import numpy as np

from dimos.navigation.motion.control.controller import load as load_controller
from dimos.navigation.motion.control.referee.battery import group_summaries, ood_worlds, run_battery
from dimos.navigation.motion.control.referee.episode import (
    DomainRandomization,
    EpisodeConfig,
    run_episode,
)
from dimos.navigation.motion.control.referee.judge import print_row, score_episode, summarize
from dimos.navigation.motion.control.tracks import TRACKS
from dimos.navigation.motion.scenarios import (
    SCENARIOS,
    generated,
    recorded,
)
from dimos.navigation.motion.simulation.policy import FreePolicy
from dimos.utils.data import get_data

DEFAULT_POLICY = "ml-trajectory-research/freewalk_mcf.bin"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-s", "--scenario", help="scenario name, glob ('gen*'), or group (curated/gen)")
    ap.add_argument("--ls", action="store_true", help="list scenario names and exit")
    ap.add_argument("--gen", type=int, default=0, help="add N generated worlds")
    ap.add_argument(
        "--recorded",
        action="append",
        default=[],
        metavar="NPZ",
        help="add a world recorded on the robot (simulation.recorded_world npz)",
    )
    ap.add_argument("--view", action="store_true", help="live MuJoCo viewer")
    ap.add_argument("--speed", type=float, default=1.0, help="viewer speed factor")
    ap.add_argument("--score", action="store_true", help="judge each episode + summary")
    ap.add_argument("--json", action="store_true", help="summary as JSON only")
    ap.add_argument("--replan-hz", type=float, default=5.0, help="0 = plan once (lenient)")
    ap.add_argument("--policy", default=DEFAULT_POLICY, help="FREE policy blob")
    ap.add_argument("--planner", default="target", help="referee planner registry name")
    ap.add_argument(
        "--controller",
        default=None,
        help="controller registry name or module:factory (default: the track's law)",
    )
    ap.add_argument("--blind", action="store_true", help="the blind track: no clearance annotation")
    ap.add_argument("--dr", action="store_true", help="randomize mechanisms per episode")
    ap.add_argument("--seed", type=int, default=0, help="rng seed for --dr")
    ap.add_argument("--draws", type=int, default=1, help="DR draws per world (implies --dr)")
    ap.add_argument("--ood", type=int, default=0, help="add N held-out-rules worlds")
    ap.add_argument("--jobs", type=int, default=1, help="parallel episode workers")
    args = ap.parse_args()

    tagged = [(sc, "curated") for sc in SCENARIOS]
    if args.gen:
        tagged += [(sc, "gen") for sc in generated(args.gen)]
    if args.ood:
        tagged += [(sc, "ood") for sc in ood_worlds(args.ood)]
    tagged += [(recorded(p), "recorded") for p in args.recorded]
    if args.scenario:
        from fnmatch import fnmatch

        # a glob selects a family: -s 'gen*' runs only the generated worlds
        tagged = [(s, g) for s, g in tagged if fnmatch(s.name, args.scenario) or g == args.scenario]
        if not tagged:
            ap.error(f"no scenario matching {args.scenario!r}")
    if args.ls:
        for sc, group in tagged:
            emb = f" [{sc.emb.tag}]" if sc.emb.tag != "go2" else ""
            print(f"{sc.name:<20s} {group:<8s} {sc.expect:<7s}{emb} {sc.note}")
        return
    if args.jobs > 1 and args.view:
        ap.error("--jobs > 1 cannot render; --view runs serially")

    # The track fixes what the follower is handed AND which law it runs; an
    # explicit --controller still overrides, so cross-track A/Bs stay possible.
    track = TRACKS["blind" if args.blind else "hinted"]
    controller_name = args.controller or track.controller
    cfg = EpisodeConfig(
        replan_hz=args.replan_hz,
        planner=args.planner,
        annotate_clearance=track.annotate_clearance,
    )
    dr = DomainRandomization() if (args.dr or args.draws > 1) else None

    if args.view:
        policy = FreePolicy.load(get_data(args.policy))
        make_controller = load_controller(controller_name)
        rng = np.random.default_rng(args.seed)
        rows = []
        for sc, group in tagged:
            result = run_episode(
                sc, make_controller(), policy, cfg, view=True, speed=args.speed, dr=dr, rng=rng
            )
            row = score_episode(result)
            row["group"] = group
            rows.append(row)
            if not args.json:
                print_row(row, sc)
    else:
        by_name = {sc.name: sc for sc, _ in tagged}
        rows = run_battery(
            tagged,
            policy_path=args.policy,
            controller_name=controller_name,
            cfg=cfg,
            dr=dr,
            seed=args.seed,
            draws=args.draws,
            jobs=args.jobs,
        )
        if not args.json:
            for row in rows:
                print_row(row, by_name[row["name"]])
    summary = summarize(rows)
    summary["groups"] = group_summaries(rows)
    print(json.dumps({"summary": summary} if args.json else summary))
    # Categorical gate: a truth-labeled-clear world that did not reach goal
    # fails the suite outright — the mean is not allowed to absorb it.
    if summary["failed"]:
        print(
            f"FAILED: {len(summary['failed'])} clear world(s) did not reach goal: "
            + ", ".join(summary["failed"]),
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
