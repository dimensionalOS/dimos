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

"""Every test module under dimos/e2e_tests carries an e2e routing marker.

The default suite (`uv run pytest`, the CI `tests` job) excludes `web_browser`
and `self_hosted_large`, and the daily `e2e` workflow selects them. An unmarked
file there would run on every PR instead.
"""

import re

import yaml

from dimos.constants import DIMOS_PROJECT_ROOT

E2E_TESTS_DIR = DIMOS_PROJECT_ROOT / "dimos" / "e2e_tests"
E2E_MARKER = re.compile(r"pytest\.mark\.(web_browser|self_hosted_large)\b")


def test_e2e_test_modules_are_marked() -> None:
    unmarked = [
        str(path.relative_to(DIMOS_PROJECT_ROOT))
        for path in sorted(E2E_TESTS_DIR.glob("test_*.py"))
        if not E2E_MARKER.search(path.read_text(encoding="utf-8"))
    ]
    assert not unmarked, (
        f"e2e test modules without a web_browser or self_hosted_large marker: {unmarked}. "
        "Tests under dimos/e2e_tests run in the daily e2e workflow only."
    )


def test_large_e2e_job_collects_all_and_only_its_test_modules() -> None:
    workflow = yaml.safe_load((DIMOS_PROJECT_ROOT / ".github/workflows/e2e.yml").read_text())
    job = workflow["jobs"]["self-hosted-large-tests"]
    marker = re.compile(r"pytest\.mark\.self_hosted_large\b")
    expected = {
        str(path.relative_to(DIMOS_PROJECT_ROOT))
        for path in (DIMOS_PROJECT_ROOT / "dimos").rglob("test_*.py")
        if marker.search(path.read_text(encoding="utf-8"))
    }

    assert set(job["env"]["LARGE_TEST_PATHS"].split()) == expected, (
        "Update LARGE_TEST_PATHS when adding or removing self_hosted_large test modules. "
        "Collecting unrelated modules causes import skips before pytest applies -m."
    )
    test_commands = [step["run"] for step in job["steps"] if "uv run pytest" in step.get("run", "")]
    assert test_commands
    for command in test_commands:
        assert "$LARGE_TEST_PATHS" in command
        assert "--error-for-skips" in command
