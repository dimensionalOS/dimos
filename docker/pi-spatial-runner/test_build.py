import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import zipfile

import pytest

SPEC = importlib.util.spec_from_file_location("spatial_build", Path(__file__).with_name("build.py"))
assert SPEC and SPEC.loader
build = importlib.util.module_from_spec(SPEC)
sys.modules["spatial_build"] = build
SPEC.loader.exec_module(build)

VERIFY_SPEC = importlib.util.spec_from_file_location(
    "builder_verify", Path(__file__).with_name("builder_verify.py")
)
assert VERIFY_SPEC and VERIFY_SPEC.loader
builder_verify = importlib.util.module_from_spec(VERIFY_SPEC)
sys.modules["builder_verify"] = builder_verify
VERIFY_SPEC.loader.exec_module(builder_verify)


def test_base_image_must_be_immutable() -> None:
    build.validate_base_image("python:3.12@sha256:" + "a" * 64)
    with pytest.raises(build.BuildError):
        build.validate_base_image("python:3.12")


def test_dirty_source_identity_is_head_tree_not_working_tree(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    outputs = iter(
        [
            " M tracked.py\n?? local-secret.txt\n",
            "deadbeef\n",
            "cafebabe\n",
            "tracked diff\n",
        ]
    )
    monkeypatch.setattr(
        build, "run", lambda *args, **kwargs: type("R", (), {"stdout": next(outputs)})()
    )
    revision, dirty, tree, working, publishable = build.source_identity(True)
    assert (revision, dirty, tree, publishable) == ("deadbeef", True, "cafebabe", False)
    assert working and working != "unavailable-dirty-working-tree"


def test_auto_builder_cleanup_never_targets_external_builder(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[list[str]] = []
    monkeypatch.setattr(
        build,
        "run",
        lambda args, **kwargs: calls.append(list(args)) or type("R", (), {"stdout": ""})(),
    )
    build.remove_auto_builder("dimos-spatial-builder:123", "podman")
    assert calls == [
        ["podman", "rmi", "--force", "dimos-spatial-builder:123"],
        ["podman", "image", "exists", "dimos-spatial-builder:123"],
    ]


def test_builder_cleanup_failure_is_fatal() -> None:
    with pytest.raises(build.BuildError, match="remains"):
        build.require_builder_cleanup(False)


def test_clean_build_rejects_uv_drift(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr(build.shutil, "which", lambda name: "/usr/bin/uv")
    monkeypatch.setattr(
        build,
        "run",
        lambda *args, **kwargs: type("R", (), {"stdout": "uv 0.9.17"})(),
    )
    with pytest.raises(build.BuildError, match=build.UV_VERSION):
        build.verify_uv(tmp_path, allow_dirty=False)


def test_dirty_build_may_use_pinned_uvx_fallback(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(
        build.shutil, "which", lambda name: "/usr/bin/uvx" if name == "uvx" else "/usr/bin/uv"
    )
    monkeypatch.setattr(
        build,
        "run",
        lambda args, **kwargs: type(
            "R",
            (),
            {"stdout": f"uv {build.UV_VERSION}" if "uv-pinned" in str(args[0]) else "uv 0.9.17"},
        )(),
    )
    assert build.verify_uv(tmp_path, allow_dirty=True).endswith("uv-pinned")


def test_pinned_uv_parser_accepts_current_lock_owner(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(build.shutil, "which", lambda name: "/usr/bin/uv")
    monkeypatch.setattr(
        build,
        "run",
        lambda *args, **kwargs: type("R", (), {"stdout": f"uv {build.UV_VERSION}\n"})(),
    )
    assert build.verify_uv(tmp_path) == "/usr/bin/uv"


def test_export_is_exactly_bare_locked_project_export() -> None:
    assert build.BARE_EXPORT_ARGS == (
        "export",
        "--format",
        "requirements.txt",
        "--no-default-groups",
        "--no-emit-project",
        "--locked",
    )
    assert not any(arg in build.BARE_EXPORT_ARGS for arg in ("--group", "--all-groups"))


def make_sdist(path: Path, *, malicious: str | None = None, link: bool = False) -> None:
    with tarfile.open(path, "w:gz") as archive:
        for name in (
            "dimos-0.0.13",
            "dimos-0.0.13/PKG-INFO",
            "dimos-0.0.13/pyproject.toml",
            "dimos-0.0.13/setup.py",
            "dimos-0.0.13/dimos/__init__.py",
            "dimos-0.0.13/dimos/navigation/replanning_a_star/min_cost_astar_cpp.cpp",
            "dimos-0.0.13/packages/dimos-runtime-protocol/src/dimos_runtime_protocol/__init__.py",
            "dimos-0.0.13/packages/dimos-gpd-grasp-demo/src/dimos_gpd_grasp_demo/__init__.py",
        ):
            info = tarfile.TarInfo(name)
            if name == "dimos-0.0.13" or name.endswith("/"):
                info.type = tarfile.DIRTYPE
            else:
                data = b"Name: dimos\nVersion: 0.0.13\n" if name.endswith("PKG-INFO") else b"x"
                info.size = len(data)
                archive.addfile(info, io.BytesIO(data))
        if malicious:
            info = tarfile.TarInfo(malicious)
            if link:
                info.type = tarfile.SYMTYPE
                info.linkname = "/etc/passwd"
                archive.addfile(info)
            else:
                info.size = 1
                archive.addfile(info, io.BytesIO(b"x"))


def test_sdist_inspection_and_canonical_normalization(tmp_path: Path) -> None:
    source = tmp_path / "dimos-0.0.13.tar.gz"
    make_sdist(source)
    build.normalize_sdist(source, 123)
    version, digest, inventory, _ = build.inspect_sdist(source)
    assert version == "0.0.13" and len(digest) == 64 and len(inventory) == 64


@pytest.mark.parametrize("name", ["../secret", "dimos-0.0.13/.env", "dimos-0.0.13/model.pt"])
def test_sdist_rejects_unsafe_members(tmp_path: Path, name: str) -> None:
    source = tmp_path / "dimos-0.0.13.tar.gz"
    make_sdist(source, malicious=name)
    with pytest.raises(build.BuildError):
        build.inspect_sdist(source)


def test_sdist_rejects_links(tmp_path: Path) -> None:
    source = tmp_path / "dimos-0.0.13.tar.gz"
    make_sdist(source, malicious="dimos-0.0.13/dimos/link", link=True)
    with pytest.raises(build.BuildError):
        build.inspect_sdist(source)


def test_sdist_rejects_unknown_nested_packaged_root(tmp_path: Path) -> None:
    source = tmp_path / "dimos-0.0.13.tar.gz"
    make_sdist(source, malicious="dimos-0.0.13/packages/unknown/file.py")
    with pytest.raises(build.BuildError, match="packaged root"):
        build.inspect_sdist(source)


def test_publishable_sdist_rejects_unadmitted_package_data(tmp_path: Path) -> None:
    source = tmp_path / "dimos-0.0.13.tar.gz"
    make_sdist(source, malicious="dimos-0.0.13/dimos/harmless.json")
    with pytest.raises(build.BuildError, match="package data"):
        build.inspect_sdist(source, require_git_members=True)


def test_builder_labels_are_fail_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    expected = {
        "io.dimos.builder": "true",
        "io.dimos.builder-base-digest": "sha256:" + "a" * 64,
        "io.dimos.builder-requirements-sha256": "r",
        "io.dimos.builder-verifier-sha256": "v",
        "io.dimos.builder-platform": "linux/amd64",
    }
    monkeypatch.setattr(
        build,
        "run",
        lambda *args, **kwargs: type("R", (), {"stdout": json.dumps(expected)})(),
    )
    build.inspect_builder_labels(
        "builder", "base@sha256:" + "a" * 64, "r", "v", "linux/amd64", "podman"
    )
    for key in expected:
        bad = dict(expected)
        bad[key] = "wrong"
        monkeypatch.setattr(
            build,
            "run",
            lambda *args, bad=bad, **kwargs: type("R", (), {"stdout": json.dumps(bad)})(),
        )
        with pytest.raises(build.BuildError):
            build.inspect_builder_labels(
                "builder", "base@sha256:" + "a" * 64, "r", "v", "linux/amd64", "podman"
            )


def test_builder_verifier_rejects_expected_sdist_hash(tmp_path: Path) -> None:
    source = tmp_path / "source.tar.gz"
    source.write_bytes(b"not the expected sdist")
    result = subprocess.run(
        [
            "python",
            str(Path(__file__).with_name("builder_verify.py")),
            "--sdist",
            str(source),
            "--expected-arch",
            "linux/amd64",
            "--source-date-epoch",
            "0",
        ],
        env={**os.environ, "EXPECTED_SDIST_SHA256": "0" * 64},
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0 and "sdist bytes do not match" in result.stderr + result.stdout


def test_abi_report_identity_and_runtime_ceilings_are_rejected() -> None:
    report: dict[str, object] = {
        "wheel_filename": "dimos.whl",
        "wheel_sha256": "w",
        "architecture": "linux/amd64",
        "max_glibc": "2.34",
        "glibcxx": [],
        "cxxabi": ["1.3.13"],
        "max_glibcxx": "3.4.29",
        "max_cxxabi": "1.3.13",
        "auditwheel_report_sha256": "a",
        "auditwheel_report": "ok",
    }
    with pytest.raises(build.BuildError):
        build.validate_abi_report(
            report | {"wheel_sha256": "wrong"}, "dimos.whl", "w", "linux/amd64"
        )
    with pytest.raises(build.BuildError):
        build.validate_abi_report(
            report | {"architecture": "linux/arm64"}, "dimos.whl", "w", "linux/amd64"
        )
    for key, value in (("max_glibcxx", "3.4.31"), ("max_cxxabi", "1.3.13")):
        metadata = {
            "runtime_glibc": "2.36",
            "runtime_glibcxx": "3.4.30",
            "runtime_cxxabi": "1.3.12",
        }
        if key == "max_cxxabi":
            report = report | {key: value}
        else:
            report = report | {key: value}
        with pytest.raises(build.BuildError):
            build.validate_runtime_abi(report, metadata)


def test_wheel_tags_are_runtime_compatible(tmp_path: Path) -> None:
    wheel = tmp_path / "dimos-0.0.13-cp312-cp312-manylinux_2_34_x86_64.manylinux_2_36_x86_64.whl"
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr("dimos/__init__.py", "")
        archive.writestr("dimos-0.0.13.dist-info/METADATA", "Name: dimos\nVersion: 0.0.13\n")
    assert build.inspect_wheel(wheel, "linux/amd64")[0] == "0.0.13"
    with pytest.raises(build.BuildError):
        build.inspect_wheel(tmp_path / "dimos-0.0.13-py3-none-any.whl", "linux/amd64")


def test_abi_gate_rejects_glibc_238() -> None:
    with pytest.raises(RuntimeError, match="GLIBC_2.36"):
        builder_verify.validate_glibc(["2.34", "2.38"])


def test_builder_contract_has_no_host_wheel_build() -> None:
    text = (Path(__file__).with_name("build.py")).read_text()
    assert '"--no-isolation"' in text and "build_sdist_with_pinned_frontend" in text
    assert "verify_uv" in text and "PACKAGING_LOCK" in text
    assert "--network=none" in text
    assert "--cap-drop=ALL" in text
    assert "--userns=keep-id" in text
    assert "--read-only" in text
    assert "--no-isolation" in text


def test_open3d_widget_runtime_is_bound_to_root_lock() -> None:
    root = Path(__file__).parents[2]
    project = (root / "pyproject.toml").read_text()
    lock = (root / "uv.lock").read_text()
    assert '"ipywidgets>=8.0.4"' in project
    assert 'name = "ipywidgets"' in lock
    assert 'name = "jupyterlab-widgets"' in lock
    assert 'name = "widgetsnbextension"' in lock


def test_runner_does_not_promote_optional_model_dependencies() -> None:
    project = (Path(__file__).parents[2] / "pyproject.toml").read_text()
    dependencies = project.split("[project]\n", 1)[1].split("[project.scripts]", 1)[0]
    assert all(f'"{name}' not in dependencies for name in ("torch", "transformers", "onnxruntime"))


def test_command_failure_diagnostic_is_bounded_and_redacted() -> None:
    import subprocess

    error = subprocess.CalledProcessError(
        2,
        ["podman", "build"],
        output="password=hunter2 " + "x" * 5000,
        stderr="token=abc123 " + "y" * 5000,
    )
    text = build.command_failure(error, "podman build")
    assert len(text) < 2 * build.DIAGNOSTIC_LIMIT + 200
    assert "hunter2" not in text and "abc123" not in text
