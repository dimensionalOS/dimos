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

"""MCP surface for evals — lets a coding agent spin runs and grep the run dir."""

from __future__ import annotations

import importlib
import pkgutil

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.module import Module


def list_suites() -> list[str]:
    """Dotted module paths under dimos.evals.suites exporting ``SUITE``."""
    from dimos.evals import suites

    return [
        name for _, name, _ in pkgutil.iter_modules(suites.__path__, prefix=f"{suites.__name__}.")
    ]


class EvalModule(Module):
    """Expose eval runs as skills so agents can iterate: run, read the summary,
    grep transcripts in the returned run_dir, edit code/prompts, run again."""

    @skill
    def run_evals(self, suite: str, tags: str = "") -> SkillResult:
        """Run an eval suite by dotted module path (see list_eval_suites).

        Args:
            suite: e.g. "dimos.evals.suites.go2_smoke" (must export SUITE).
            tags: optional comma-separated tag filter.
        """
        from dimos.evals.runner import EvalRunner, summarize

        cases = importlib.import_module(suite).SUITE
        runner = EvalRunner(attach=True)
        results = runner.run(
            cases, tags=frozenset(t for t in tags.split(",") if t) if tags else frozenset()
        )
        s = summarize(results)
        return SkillResult.ok(
            f"{s.n} cases: mean={s.mean_score:.2f} pass={s.pass_rate:.0%} errors={s.errors}",
            run_dir=str(runner.run_dir),
        )

    @skill
    def list_eval_suites(self) -> SkillResult:
        """List available eval suite module paths."""
        return SkillResult.ok(", ".join(list_suites()))
