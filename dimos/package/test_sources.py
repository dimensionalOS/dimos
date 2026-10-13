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

from pathlib import Path

import pytest

from dimos.package.sources import resolve_source, workspace_member


def test_workspace_member_uses_declared_distribution_name(tmp_path):
    (tmp_path / "pyproject.toml").write_text(
        '[tool.uv.workspace]\nmembers=["packages/*"]\nexclude=["packages/skip"]\n'
    )
    for folder, name in (("one", "acme_probe"), ("two", "acme-camera"), ("skip", "acme-probe")):
        member = tmp_path / "packages" / folder
        member.mkdir(parents=True)
        (member / "pyproject.toml").write_text(f'[project]\nname="{name}"\n')
    assert workspace_member(tmp_path, "Acme.Probe") == Path("packages/one")
    with pytest.raises(ValueError, match="found 0"):
        workspace_member(tmp_path, "probe-blueprint")


def test_workspace_rejects_ambiguous_names_and_escaping_members(tmp_path):
    (tmp_path / "pyproject.toml").write_text(
        '[project]\nname="same"\n[tool.uv.workspace]\nmembers=["member"]\n'
    )
    member = tmp_path / "member"
    member.mkdir()
    (member / "pyproject.toml").write_text('[project]\nname="same"\n')
    with pytest.raises(ValueError, match="found 2"):
        workspace_member(tmp_path, "same")
    (tmp_path / "pyproject.toml").write_text('[tool.uv.workspace]\nmembers=["../outside"]\n')
    with pytest.raises(ValueError, match="inside"):
        workspace_member(tmp_path, "same")


def test_explicit_sources_and_safe_subdirectory():
    assert (
        resolve_source("git+https://example.com/repo", subdirectory="packages/a")
        == "git+https://example.com/repo#subdirectory=packages/a"
    )
    assert resolve_source("../local") == str((Path.cwd() / "../local").resolve())
    for source in ("github:owner/repo", "git+https://example.com/repo#subdirectory=../outside"):
        with pytest.raises(ValueError):
            resolve_source(source)


def test_github_member_is_inspected_at_the_selected_revision(mocker):
    def git_run(args, **kwargs):
        if args[1] == "clone":
            root = Path(args[-1])
            root.mkdir()
            (root / "pyproject.toml").write_text('[tool.uv.workspace]\nmembers=["packages/*"]\n')
            member = root / "packages" / "sensor"
            member.mkdir(parents=True)
            (member / "pyproject.toml").write_text('[project]\nname="acme-probe"\n')

    run = mocker.patch("dimos.package.sources.subprocess.run", side_effect=git_run)
    lookup = mocker.patch("dimos.package.sources.subprocess.check_output", return_value="abc123\n")
    assert (
        resolve_source("github:owner/robot#acme-probe", branch="feature")
        == "git+https://github.com/owner/robot.git#subdirectory=packages/sensor"
    )
    assert lookup.call_args.args[0][-1] == "refs/remotes/origin/feature^{commit}"
    assert run.call_args.args[0][-2:] == ["--detach", "abc123"]
