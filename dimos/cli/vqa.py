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

"""CLI commands for generating and evaluating standalone VQA datasets."""

from __future__ import annotations

from pathlib import Path

import typer

app = typer.Typer(help="Generate and evaluate standalone visual question-answering datasets.")


@app.command("generate")
def generate(
    dataset: str = typer.Argument(
        help="Memory dataset with color_image, camera_info, tf, and pointlio_lidar/lidar streams"
    ),
    image_index: int | None = typer.Option(None, min=0, help="Process one color_image index"),
    start: int | None = typer.Option(None, min=0, help="First color_image index in range mode"),
    stop: int | None = typer.Option(None, min=1, help="Exclusive color_image stop index"),
    stride: int | None = typer.Option(None, min=1, help="Frame stride in range mode"),
    output: Path | None = typer.Option(
        None,
        help="Destination directory; must be absent or empty",
    ),
    family_max_share: list[str] | None = typer.Option(
        None,
        help="Cap accepted share for a family as name=share (repeatable), e.g. presence=0.25",
    ),
    family_balance: str = typer.Option(
        "none",
        help="Family menu policy: none, or uniform (cap every capable family at 1/N)",
    ),
) -> None:
    """Generate questions for one image or an indexed image range."""
    # Keep generation's optional model stack out of global CLI startup.
    from dimos.evals.vqa.generate import (
        GenerationRequest,
        VqaGenerationConfig,
        generate_dataset,
    )

    try:
        if family_balance not in {"none", "uniform"}:
            raise ValueError("family-balance must be 'none' or 'uniform'")
        request = GenerationRequest(
            dataset=dataset,
            output=output,
            image_index=image_index,
            start=start,
            stop=stop,
            stride=stride,
        )
        config = VqaGenerationConfig(
            family_max_share=_parse_family_max_share(family_max_share),
            family_balance=family_balance,  # type: ignore[arg-type]
        )
        result = generate_dataset(request, config=config)
    except (ValueError, IndexError, OSError) as exc:
        raise typer.BadParameter(str(exc)) from exc
    typer.echo(f"Generated {len(result.cases)} VQA case(s) in {result.output}")


def _parse_family_max_share(entries: list[str] | None) -> dict[str, float] | None:
    if not entries:
        return None
    shares: dict[str, float] = {}
    for entry in entries:
        if "=" not in entry:
            raise ValueError(f"family-max-share must be name=share, got {entry!r}")
        name, raw_share = entry.split("=", 1)
        name = name.strip()
        if not name:
            raise ValueError(f"family-max-share name is empty in {entry!r}")
        try:
            share = float(raw_share.strip())
        except ValueError as exc:
            raise ValueError(f"family-max-share share must be a float in {entry!r}") from exc
        shares[name] = share
    return shares


@app.command("run")
def run(
    dataset: Path = typer.Argument(help="Generated standalone VQA dataset"),
    agent: str = typer.Option("dimos.evals.agents.question_answer", help="Dotted agent module"),
    model: str = typer.Option("", help="Override the agent's model"),
) -> None:
    """Evaluate a generated standalone VQA dataset."""
    # Keep evaluation implementation imports out of global CLI startup.
    from dimos.evals.cli import agent_kwargs, load_agent, run_provenance
    from dimos.evals.runner import EvalRunner, summarize
    from dimos.evals.vqa.suite import load_suite, source_record

    runner = EvalRunner()
    overrides = [f"model={model}"] if model else []
    try:
        results = runner.run(
            load_suite(dataset),
            load_agent(agent, overrides),
            provenance=run_provenance(source_record(dataset), agent, agent_kwargs(overrides)),
        )
    except (ValueError, OSError) as exc:
        raise typer.BadParameter(str(exc)) from exc
    for result in results:
        status = "ERROR" if result.error else ("PASS" if result.passed else "fail")
        detail = result.error or f"answer={result.final_answer[:60]!r}"
        typer.echo(f"{status:5} {result.case_id:30} {detail}")
    summary = summarize(results)
    typer.echo(
        f"\n{summary.n} cases | mean {summary.mean_score:.2f} | "
        f"pass {summary.pass_rate:.0%} | errors {summary.errors} | {runner.run_dir}"
    )
