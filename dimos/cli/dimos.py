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

"""Assembly of the `dimos` CLI: the root typer app and every command registration.

The commands themselves live in dimos.cli.commands; this module only wires them
up, in the order that defines `dimos --help`.
"""

from __future__ import annotations

import os
import sys

# Native thread-pool hygiene. Every OpenMP/BLAS runtime sizes its pool to all
# cores and spin-waits at barriers (libgomp: ~300k pause iterations per region;
# libomp: KMP_BLOCKTIME of active spinning after each region). With ~10 worker
# processes each owning core-sized pools this oversubscribes ~10x: measured
# 12% of all cycles spinning in gomp_barrier_wait_end and load avg 30-50 at
# ~50% CPU. PASSIVE wait + small pools cut a 10 Hz open3d workload 3.7x
# (1.10 -> 0.30 cores) and ~50 threads/worker -> ~10. Must be set before any
# library loads its OpenMP runtime (libgomp reads env at load, OpenBLAS spawns
# its pool at import); workers inherit via the forkserver. User env wins.
os.environ.setdefault("KMP_BLOCKTIME", "0")
os.environ.setdefault("OMP_WAIT_POLICY", "PASSIVE")
os.environ.setdefault("GOMP_SPINCOUNT", "0")
os.environ.setdefault("OMP_NUM_THREADS", "4")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "2")
os.environ.setdefault("MKL_NUM_THREADS", "2")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "2")
# cv2 reads this at import for its parallel_for pool; replaces the eager
# `import cv2; cv2.setNumThreads(2)` the worker entrypoint used to do.
os.environ.setdefault("OPENCV_FOR_THREADS_NUM", "2")

from dotenv import load_dotenv
import typer

from dimos.cli.cache import app as cache_app
from dimos.cli.can import app as can_app
from dimos.cli.cloud import login as cloud_login, logout as cloud_logout, whoami as cloud_whoami
from dimos.cli.commands.apriltag import apriltag
from dimos.cli.commands.bake import bake
from dimos.cli.commands.cameracalibrate import cameracalibrate
from dimos.cli.commands.collect import collect
from dimos.cli.commands.data import data_app
from dimos.cli.commands.dataprep import dataprep_app
from dimos.cli.commands.docs import docs
from dimos.cli.commands.global_options import create_dynamic_callback
from dimos.cli.commands.info import list_blueprints, show_config
from dimos.cli.commands.lifecycle import log_cmd, restart, run, status, stop
from dimos.cli.commands.map import map_app
from dimos.cli.commands.mcp import agent_send_cmd, mcp_app
from dimos.cli.commands.rerun_bridge import rerun_bridge_cmd
from dimos.cli.commands.topic import topic_app
from dimos.cli.commands.tuis import agentspy, humancli, lcmspy, spy, top
from dimos.cli.hardware_cli import app as hardware_app
from dimos.cli.shell import shell
from dimos.cli.vqa import app as vqa_app
from dimos.robot.unitree.go2.cli.go2tool import app as go2tool_app

main = typer.Typer(
    help="Dimensional CLI",
    no_args_is_help=True,
)

load_dotenv()

SIMULATORS = ("mujoco", "dimsim")
RECORDERS = ("sqlite",)

# Flags with an optional value; bare `--flag` means the first choice.
OPTIONAL_VALUE_FLAGS = {
    "--simulation": SIMULATORS,
    "--record": RECORDERS,
}


def normalize_argv(argv: list[str]) -> list[str]:
    out: list[str] = []
    for arg, nxt in zip(argv, [*argv[1:], None], strict=False):
        out.append(arg)
        choices = OPTIONAL_VALUE_FLAGS.get(arg)
        if choices and nxt not in choices:
            out.append(choices[0])
    return out


def cli_main() -> None:
    sys.argv = normalize_argv(sys.argv)
    main()


main.callback()(create_dynamic_callback())  # type: ignore[no-untyped-call]
hardware_app.add_typer(can_app, name="can")
main.add_typer(hardware_app, name="hardware")
main.add_typer(data_app, name="data")
main.add_typer(go2tool_app, name="go2tool")
main.command()(shell)
main.add_typer(cache_app, name="cache")
main.command("login")(cloud_login)
main.command("logout")(cloud_logout)
main.command("whoami")(cloud_whoami)
main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})(run)
main.command()(status)
main.command()(stop)
main.command("log")(log_cmd)
main.add_typer(mcp_app, name="mcp")
main.command("agent-send")(agent_send_cmd)
main.command()(restart)
main.command()(show_config)
main.command(
    context_settings={
        "allow_extra_args": True,
        "ignore_unknown_options": True,
        # let --help through to the bake command itself
        "help_option_names": [],
    }
)(bake)
main.command(name="list")(list_blueprints)
main.command()(docs)
main.command()(collect)
main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})(spy)
main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})(lcmspy)
main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})(agentspy)
main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})(humancli)
main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})(top)
main.add_typer(topic_app, name="topic")
main.add_typer(map_app, name="map")

from dimos.navigation.nav_3d.evaluator.cli import app as nav_eval_app

main.add_typer(nav_eval_app, name="nav-eval")
main.add_typer(dataprep_app, name="dataprep")

from dimos.memory.cli.app import mem_app

main.add_typer(mem_app, name="mem")

from dimos.evals.cli import app as evals_app

evals_app.add_typer(vqa_app, name="vqa")
main.add_typer(evals_app, name="evals")

main.command()(cameracalibrate)
main.command()(apriltag)
main.command(name="rerun-bridge")(rerun_bridge_cmd)


if __name__ == "__main__":
    cli_main()
