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

"""Attached operator controls and offline dataset preparation."""

import hashlib
import json
from pathlib import Path
import subprocess

from rich.console import Console
import typer

from dimos.cli.imitation_inspect import print_inspection
from dimos.constants import STATE_DIR
from dimos.experimental.isolated_python.module import (
    isolated_python_environment,
    isolated_python_run_command,
    prepare_isolated_python,
)
from dimos.imitation.collection.prompts import CollectionSpeech
from dimos.imitation.collection.recording import RecordingSchema
from dimos.imitation.dataprep.build import inspect_dataset, inspect_recording
from dimos.imitation.dataprep.core import OutputConfig
from dimos.imitation.dataprep.lerobot import lerobot_project, run_lerobot_dataprep
from dimos.imitation.tui import CollectionApp, CollectionSession, RolloutApp, RolloutSession
from dimos.porcelain.dimos import Dimos
from dimos.stream.audio.tts.kokoro import KokoroTTSConfig
from dimos.utils.cache import cache_usage_guard

imitation_app = typer.Typer(help="Operate running collection/policy modules and prepare datasets")


def _default_dataset(recording: Path) -> Path:
    return STATE_DIR / "datasets" / recording.name


def _require_new_path(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    if resolved.exists():
        raise typer.BadParameter(f"Dataset already exists: {resolved}")
    return resolved


@imitation_app.command()
def collect(
    tts: bool = typer.Option(False, "--tts", help="Speak recording feedback on this computer"),
) -> None:
    """Attach episode controls to a blueprint started with dimos run."""
    driver = None
    app = None
    try:
        driver = Dimos.connect()
        session = CollectionSession(driver)
        speech = None
        if tts:
            typer.echo("Preparing recording speech...")
            speech = CollectionSpeech(KokoroTTSConfig(enabled=True))
            speech.prepare()
        app = CollectionApp(session, speech=speech)
        app.run()
    except Exception as exc:
        typer.echo(f"Collection controls failed: {exc}", err=True)
        raise typer.Exit(1) from exc
    finally:
        if app is not None:
            app.stop_audio()
        if driver is not None:
            driver.stop()


@imitation_app.command()
def rollout() -> None:
    """Attach policy start/stop controls; quitting only disconnects."""
    driver = None
    try:
        driver = Dimos.connect()
        RolloutApp(RolloutSession(driver)).run()
    except Exception as exc:
        typer.echo(f"Rollout controls failed: {exc}", err=True)
        raise typer.Exit(1) from exc
    finally:
        if driver is not None:
            driver.stop()


@imitation_app.command()
def prepare(
    recording: Path = typer.Argument(..., help="Collection directory containing schema.json"),
    output: Path | None = typer.Option(None, "--output", help="New LeRobot dataset directory"),
) -> None:
    """Prepare a dataset using the schema saved with its recording."""
    source = recording.expanduser().resolve()
    target = _require_new_path(output or _default_dataset(source))
    try:
        schema = RecordingSchema.read(source)
        config = schema.dataprep_config(source, OutputConfig(format="lerobot", path=target))
        result = run_lerobot_dataprep(config)
    except Exception as exc:
        typer.echo(f"Preparation failed: {exc}", err=True)
        raise typer.Exit(1) from exc
    typer.echo(f"Wrote dataset: {result}")


@imitation_app.command()
def inspect(
    artifact: Path,
    json_output: bool = typer.Option(False, "--json", help="Print the complete result as JSON"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Show every assessed episode"),
) -> None:
    """Inspect a collection directory or a prepared dataset."""
    path = artifact.expanduser().resolve()
    try:
        if (path / "schema.json").is_file():
            schema = RecordingSchema.read(path)
            config = schema.dataprep_config(path, OutputConfig(path=_default_dataset(path)))
            info = inspect_recording(path / schema.payload, config=config)
        else:
            info = inspect_dataset(path)
    except Exception as exc:
        typer.echo(f"Inspection failed: {exc}", err=True)
        raise typer.Exit(1) from exc
    if json_output:
        typer.echo(json.dumps(info, indent=2, default=str))
    else:
        print_inspection(info, console=Console(highlight=False), verbose=verbose)


@imitation_app.command()
def visualize(
    path: Path = typer.Argument(..., help="Local prepared LeRobot dataset directory"),
    episode: int = typer.Option(0, "--episode", min=0, help="Zero-based episode index"),
) -> None:
    """View camera images, joint states, and actions in the local Rerun viewer."""
    dataset = path.expanduser().resolve()
    if not dataset.is_dir():
        raise typer.BadParameter(f"Dataset directory does not exist: {dataset}")
    if (dataset / "schema.json").is_file():
        raise typer.BadParameter("This is a recording; run dimos imitation prepare first")
    if not (dataset / "meta" / "info.json").is_file():
        raise typer.BadParameter(
            f"Not a prepared LeRobot dataset: missing {dataset / 'meta/info.json'}"
        )

    project = lerobot_project()
    # LeRobot uses repo_id as the Rerun application identity for saved layouts.
    dataset_id = hashlib.sha256(str(dataset).encode("utf-8")).hexdigest()[:16]
    command = isolated_python_run_command(
        project,
        "--project",
        str(project),
        "lerobot-dataset-viz",
        "--root",
        str(dataset),
        "--repo-id",
        f"local/dataset-{dataset_id}",
        "--episode-index",
        str(episode),
        "--num-workers",
        "0",
        "--mode",
        "local",
    )
    env = isolated_python_environment(project)
    env["HF_HUB_OFFLINE"] = "1"
    try:
        with cache_usage_guard():
            prepare_isolated_python(project, env)
            result = subprocess.run(command, env=env, check=False)
    except OSError as exc:
        typer.echo(f"Visualization failed to launch uv: {exc}", err=True)
        raise typer.Exit(1) from exc
    if result.returncode:
        raise typer.Exit(result.returncode)


@imitation_app.command(
    context_settings={
        "allow_extra_args": True,
        "ignore_unknown_options": True,
        "help_option_names": [],
    }
)
def train(ctx: typer.Context) -> None:
    """Pass all arguments directly to ``lerobot-train``."""
    project = lerobot_project()
    command = isolated_python_run_command(
        project, "--project", str(project), "lerobot-train", *ctx.args
    )
    env = isolated_python_environment(project)
    with cache_usage_guard():
        prepare_isolated_python(project, env)
        result = subprocess.run(command, env=env, check=False)
    if result.returncode:
        raise typer.Exit(result.returncode)
