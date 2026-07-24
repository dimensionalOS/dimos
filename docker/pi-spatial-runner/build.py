"""Build the artifact-only Dimos spatial runner image."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import dataclass
from email.parser import Parser
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
from typing import Any
import zipfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
# uv 0.11.24 owns the repository's lock schema ([options] and
# [options.exclude-newer-package]); uv 0.9.17 cannot parse it.
UV_VERSION = "0.11.24"
BARE_EXPORT_ARGS = (
    "export",
    "--format",
    "requirements.txt",
    "--no-default-groups",
    "--no-emit-project",
    "--locked",
)
BASE_RE = re.compile(r"^[^@\s]+@sha256:[0-9a-f]{64}$")
WHEEL_RE = re.compile(
    r"^dimos-(?P<version>[^-]+)(?:-[^-]+)?-(?P<python>[^-]+)-(?P<abi>[^-]+)-(?P<platform>[^-]+)\.whl$"
)
SDIST_RE = re.compile(r"^dimos-(?P<version>[^/]+)\.tar\.gz$")
SUSPICIOUS = re.compile(
    r"(?:^|[/_.-])(credentials?|secrets?|tokens?|passwords?|private[-_]?key|id_rsa)(?:[/_.-]|$)",
    re.I,
)
DIAGNOSTIC_LIMIT = 4000
MAX_MEMBER_SIZE = 64 * 1024 * 1024
MAX_ARCHIVE_SIZE = 512 * 1024 * 1024
PACKAGING_LOCK = HERE / "builder-requirements.lock"
TEMP_PATH_RE = re.compile(r"/tmp/dimos-spatial-build-[^/\s]+")
SECRET_RE = re.compile(r"(?i)(password|token|secret|credential|api[_-]?key)(\s*[:=]\s*)[^\s,]+")


class BuildError(RuntimeError):
    """A safe-build validation error."""


@dataclass(frozen=True)
class BuildResult:
    image_id: str
    manifest: dict[str, object]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_bounded_zip_member(archive: zipfile.ZipFile, name: str) -> bytes:
    info = archive.getinfo(name)
    if info.file_size > MAX_MEMBER_SIZE:
        raise BuildError(f"oversized archive member: {name}")
    with archive.open(info) as stream:
        data = stream.read(MAX_MEMBER_SIZE + 1)
    if len(data) > MAX_MEMBER_SIZE:
        raise BuildError(f"archive member expands beyond bound: {name}")
    return data


def run(
    args: Sequence[str],
    *,
    cwd: Path = ROOT,
    capture: bool = False,
    env: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(args), cwd=cwd, check=True, text=True, capture_output=capture, shell=False, env=env
    )


def _bounded_diagnostic(value: str | None) -> str:
    text = SECRET_RE.sub(r"\1\2<redacted>", value or "")
    text = TEMP_PATH_RE.sub("<build-context>", text)
    if len(text) > DIAGNOSTIC_LIMIT:
        text = text[-DIAGNOSTIC_LIMIT:]
        text = "...[truncated]...\n" + text
    return text


def command_failure(error: subprocess.CalledProcessError, operation: str) -> str:
    return (
        f"{operation} failed (return code {error.returncode})\n"
        f"stdout:\n{_bounded_diagnostic(error.stdout)}\n"
        f"stderr:\n{_bounded_diagnostic(error.stderr)}"
    )


def validate_base_image(value: str) -> None:
    if not BASE_RE.fullmatch(value):
        raise BuildError(
            "BASE_IMAGE must be an immutable image reference ending in @sha256:<64hex>"
        )


def native_platform() -> str:
    machine = platform.machine().lower()
    try:
        return {"x86_64": "linux/amd64", "aarch64": "linux/arm64", "arm64": "linux/arm64"}[machine]
    except KeyError as error:
        raise BuildError(f"unsupported native architecture: {machine}") from error


def _frame(digest: Any, *parts: bytes) -> None:
    for part in parts:
        digest.update(len(part).to_bytes(8, "big"))
        digest.update(part)


def source_identity(allow_dirty: bool) -> tuple[str, bool, str, str | None, bool]:
    dirty = bool(
        run(["git", "status", "--porcelain", "--untracked-files=all"], capture=True).stdout
    )
    if dirty and not allow_dirty:
        raise BuildError(
            "source checkout is dirty; use --allow-dirty only for a non-publishable smoke build"
        )
    revision = run(["git", "rev-parse", "HEAD"], capture=True).stdout.strip()
    tree = run(["git", "rev-parse", "HEAD^{tree}"], capture=True).stdout.strip()
    # Deliberately hash only the tracked diff.  This is a diagnostic for a dirty
    # checkout, never a source identity and never a packaged input; untracked
    # files may contain credentials or unrelated local data.
    working_tree = None
    if dirty:
        diff = run(["git", "diff", "--binary", "HEAD", "--"], capture=True).stdout
        working_tree = hashlib.sha256(diff.encode()).hexdigest()
    return revision, dirty, tree, working_tree, not dirty


def inspect_wheel(
    path: Path, target_platform: str | None = None, sdist_inventory: set[str] | None = None
) -> tuple[str, str]:
    match = WHEEL_RE.fullmatch(path.name)
    if not match:
        raise BuildError(f"unexpected wheel filename: {path.name}")
    if match.group("python") != "cp312" or match.group("abi") != "cp312":
        raise BuildError("wheel must target cp312-cp312")
    if target_platform:
        expected_arch = "x86_64" if target_platform == "linux/amd64" else "aarch64"
        platform_tag = match.group("platform")
        if not platform_tag.startswith("manylinux_2_") or not platform_tag.endswith(expected_arch):
            raise BuildError(f"wheel has unsupported platform tag: {match.group('platform')}")
    wheel_hash = sha256(path)
    if path.stat().st_size > MAX_ARCHIVE_SIZE:
        raise BuildError("wheel archive is too large")
    with zipfile.ZipFile(path) as wheel:
        names = wheel.namelist()
        if len(names) > 100_000:
            raise BuildError("wheel contains too many members")
        tops: set[str] = set()
        aggregate = 0
        seen: set[str] = set()
        seen_casefold: set[str] = set()
        for name in names:
            p = Path(name)
            if p.is_absolute() or ".." in p.parts or "\\" in name:
                raise BuildError(f"unsafe wheel path: {name}")
            if SUSPICIOUS.search(name):
                raise BuildError(f"suspicious credential-like wheel path: {name}")
            if re.search(r"(?:^|/)(?:evidence|corpus|output|weights?)(?:/|$)", name, re.I):
                raise BuildError(f"forbidden wheel member: {name}")
            if name in seen or name.casefold() in seen_casefold:
                raise BuildError(f"duplicate wheel member: {name}")
            seen.add(name)
            seen_casefold.add(name.casefold())
            info = wheel.getinfo(name)
            if info.file_size > MAX_MEMBER_SIZE:
                raise BuildError(f"oversized wheel member: {name}")
            aggregate += info.file_size
            if aggregate > MAX_ARCHIVE_SIZE:
                raise BuildError("wheel aggregate payload is too large")
            if (info.external_attr >> 16) & 0o170000 == 0o120000:
                raise BuildError(f"symlink wheel path: {name}")
            tops.add(p.parts[0])
        version = match.group("version")
        dist_info = f"dimos-{version}.dist-info"
        allowed = {
            "dimos",
            "dimos_runtime_protocol",
            "dimos_gpd_grasp_demo",
            "dimos.libs",
            dist_info,
        }
        allowed.add(f"dimos-{version}.data")
        if "dimos" not in tops or tops - allowed:
            raise BuildError(f"unexpected wheel top-level entries: {sorted(tops - allowed)}")
        metadata_name = f"{dist_info}/METADATA"
        if metadata_name not in names:
            raise BuildError("wheel must contain matching dimos dist-info metadata")
        metadata = Parser().parsestr(read_bounded_zip_member(wheel, metadata_name).decode("utf-8"))
        if (
            metadata.get("Name", "").lower() != "dimos"
            or metadata.get("Version") != project_version()
        ):
            raise BuildError("wheel metadata does not match the project")
        if sdist_inventory is not None:
            for name in names:
                if name.endswith((".py", ".json", ".yaml", ".yml")) and ".dist-info/" not in name:
                    if not any(
                        name == item or item.endswith("/" + name) for item in sdist_inventory
                    ):
                        raise BuildError(f"wheel member is absent from inspected sdist: {name}")
    return metadata["Version"], wheel_hash


def source_date_epoch() -> str:
    return run(["git", "show", "-s", "--format=%ct", "HEAD"], capture=True).stdout.strip()


def inspect_sdist(
    path: Path, *, require_git_members: bool = False
) -> tuple[str, str, str, list[dict[str, object]]]:
    match = SDIST_RE.fullmatch(path.name)
    if not match:
        raise BuildError(f"unexpected source distribution filename: {path.name}")
    if path.stat().st_size > 512 * 1024 * 1024:
        raise BuildError("source distribution is too large")
    root = f"dimos-{match.group('version')}"
    required = {"PKG-INFO", "pyproject.toml", "setup.py"}
    package_roots = {"dimos", "dimos_runtime_protocol", "dimos_gpd_grasp_demo"}
    allowed_top_level_files = {
        "PKG-INFO",
        "pyproject.toml",
        "setup.py",
        "setup.cfg",
        "MANIFEST.in",
        "LICENSE",
        "LICENSE.txt",
        "README.md",
        "dimos.egg-info",
        "packages",
    }
    inventory: list[dict[str, object]] = []
    seen: set[str] = set()
    seen_casefold: set[str] = set()
    aggregate = 0
    git_members = {
        line for line in run(["git", "ls-files", "--cached"], capture=True).stdout.splitlines()
    }
    try:
        archive = tarfile.open(path, "r:gz")
    except (tarfile.TarError, OSError) as error:
        raise BuildError(f"invalid source distribution: {error}") from error
    with archive:
        for member in archive:
            name = member.name
            normalized = name.replace("\\", "/")
            if (
                normalized != name
                or name.startswith("/")
                or any(part == ".." for part in name.split("/"))
            ):
                raise BuildError(f"unsafe sdist member: {name}")
            if not name == root and not name.startswith(root + "/"):
                raise BuildError(f"sdist must contain exactly one {root} root")
            relative = "" if name == root else name[len(root) + 1 :]
            components = relative.split("/") if relative else []
            top = components[0] if components else ""
            if top == "packages":
                if len(components) > 1 and components[1] not in {
                    "dimos-runtime-protocol",
                    "dimos-gpd-grasp-demo",
                }:
                    raise BuildError(f"unexpected packaged root: {name}")
            elif top and top not in package_roots and top not in allowed_top_level_files:
                raise BuildError(f"unexpected sdist top-level entry: {name}")
            if relative in seen or relative.casefold() in seen_casefold:
                raise BuildError(f"duplicate sdist member: {name}")
            seen.add(relative)
            seen_casefold.add(relative.casefold())
            if SUSPICIOUS.search(name) or re.search(
                r"(?:^|/)(?:\.env(?:\.|$)|.*(?:\.db|\.sqlite|\.sqlite3|\.onnx|\.pt|\.pth|\.safetensors|\.gguf|\.zip|\.tar|\.gz|\.7z|\.rar)$|output|evidence|weights?|corpus)(?:/|$)",
                name,
                re.I,
            ):
                raise BuildError(f"forbidden sdist member: {name}")
            if top in package_roots or top == "packages":
                if (
                    relative.lower().endswith((".json", ".yaml", ".yml"))
                    and relative not in git_members
                ):
                    raise BuildError(f"unadmitted package data member: {name}")
            if ".git" in name.lower() or member.size > MAX_MEMBER_SIZE:
                raise BuildError(f"forbidden or oversized sdist member: {name}")
            if not (member.isdir() or member.isreg()) or member.issym() or member.islnk():
                raise BuildError(f"non-regular sdist member: {name}")
            if member.size:
                aggregate += member.size
                if aggregate > MAX_ARCHIVE_SIZE:
                    raise BuildError("sdist aggregate payload is too large")
            content_hash = ""
            if member.isreg():
                stream = archive.extractfile(member)
                if stream is None:
                    raise BuildError(f"cannot read sdist member: {name}")
                digest = hashlib.sha256()
                while block := stream.read(1024 * 1024):
                    digest.update(block)
                content_hash = digest.hexdigest()
            if (
                require_git_members
                and member.isreg()
                and relative
                not in {
                    "PKG-INFO",
                    "pyproject.toml",
                    "setup.py",
                }
                and not relative.startswith("dimos.egg-info/")
            ):
                mapped = relative
                if mapped.startswith("packages/"):
                    mapped = mapped
                if mapped not in git_members:
                    raise BuildError(f"sdist member is not a tracked source file: {name}")
            inventory.append(
                {
                    "path": relative,
                    "mode": member.mode & 0o7777,
                    "size": member.size,
                    "sha256": content_hash,
                }
            )
    if not required.issubset(seen) or not any(
        str(item["path"]).endswith("min_cost_astar_cpp.cpp") for item in inventory
    ):
        raise BuildError("sdist is missing required project metadata or native A* source")
    paths = {str(item["path"]) for item in inventory}
    if not all(
        any(
            path == package_root
            or path.startswith(f"{package_root}/")
            or f"/src/{package_root}/" in path
            for path in paths
        )
        for package_root in package_roots
    ):
        raise BuildError("sdist is missing an expected package root")
    inventory.sort(key=lambda item: str(item["path"]))
    inventory_hash = hashlib.sha256(
        json.dumps(inventory, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    return match.group("version"), sha256(path), inventory_hash, inventory


def normalize_sdist(path: Path, epoch: int) -> None:
    """Normalize sdist metadata while enforcing streaming member bounds."""
    members: list[tuple[tarfile.TarInfo, bytes | None]] = []
    aggregate = 0
    with tarfile.open(path, "r:gz") as source:
        for member in source:
            if member.size > MAX_MEMBER_SIZE:
                raise BuildError(f"oversized sdist member: {member.name}")
            aggregate += member.size
            if aggregate > MAX_ARCHIVE_SIZE:
                raise BuildError("sdist aggregate payload is too large")
            data: bytes | None = None
            if member.isreg():
                stream = source.extractfile(member)
                if stream is None:
                    raise BuildError(f"cannot read sdist member: {member.name}")
                chunks: list[bytes] = []
                remaining = member.size
                while remaining:
                    chunk = stream.read(min(1024 * 1024, remaining))
                    if not chunk:
                        raise BuildError(f"truncated sdist member: {member.name}")
                    chunks.append(chunk)
                    remaining -= len(chunk)
                data = b"".join(chunks)
            members.append((member, data))
    temporary = path.with_suffix(".normalized.tar.gz")
    with temporary.open("wb") as raw:
        with gzip.GzipFile(fileobj=raw, mode="wb", mtime=epoch) as compressed:
            with tarfile.open(fileobj=compressed, mode="w") as target:
                for member, data in members:
                    member.mtime = epoch
                    member.uid = member.gid = 0
                    member.uname = member.gname = ""
                    member.pax_headers = {}
                    target.addfile(member, io.BytesIO(data) if data is not None else None)
    temporary.replace(path)


def validate_builder_image(
    ref: str, runtime_digest: str, *, publishable: bool, engine: str = "podman"
) -> None:
    if publishable and not BASE_RE.fullmatch(ref):
        raise BuildError("publishable builds require a digest-pinned builder image")
    if not ref:
        raise BuildError("builder image reference is required")
    result = run(
        [
            engine,
            "inspect",
            "--format",
            '{{index .Config.Labels "io.dimos.builder-base-digest"}}',
            ref,
        ],
        capture=True,
    )
    if result.stdout.strip() != runtime_digest.split("@", 1)[1]:
        raise BuildError("builder base digest does not equal runtime base digest")


def inspect_image_id(ref: str, engine: str) -> str:
    result = run([engine, "inspect", "--format", "{{.Id}}", ref], capture=True)
    value = result.stdout.strip()
    if not re.fullmatch(r"sha256:[0-9a-f]{64}", value):
        raise BuildError("builder inspect did not contain exactly sha256:<64hex>")
    return value


def validate_json_object(value: object, required: set[str], label: str) -> dict[str, object]:
    if not isinstance(value, dict) or not required.issubset(value):
        raise BuildError(f"{label} has an invalid schema")
    return value


def validate_abi_report(
    report: object, wheel_name: str, wheel_hash: str, target: str
) -> dict[str, object]:
    result = validate_json_object(
        report,
        {
            "wheel_filename",
            "wheel_sha256",
            "architecture",
            "max_glibc",
            "glibcxx",
            "cxxabi",
            "max_glibcxx",
            "max_cxxabi",
            "auditwheel_report_sha256",
            "auditwheel_report",
        },
        "ABI report",
    )
    if result["wheel_filename"] != wheel_name or result["wheel_sha256"] != wheel_hash:
        raise BuildError("ABI report wheel identity does not match independent inspection")
    max_glibc = result["max_glibc"]
    if not isinstance(max_glibc, str) or tuple(map(int, max_glibc.split("."))) > (2, 36):
        raise BuildError("ABI report exceeds GLIBC 2.36")
    if result["architecture"] != target:
        raise BuildError("ABI report architecture mismatch")
    for key in ("glibcxx", "cxxabi"):
        values = result[key]
        if not isinstance(values, list) or not all(isinstance(item, str) for item in values):
            raise BuildError(f"ABI report {key} must be a string list")
    return result


def version_tuple(value: str) -> tuple[int, ...]:
    return tuple(int(part) for part in value.split("."))


def validate_runtime_abi(report: dict[str, object], metadata: dict[str, object]) -> None:
    if version_tuple(str(report["max_glibc"])) > version_tuple(str(metadata["runtime_glibc"])):
        raise BuildError("wheel GLIBC requirement exceeds measured builder runtime")
    for report_key, runtime_key in (
        ("max_glibcxx", "runtime_glibcxx"),
        ("max_cxxabi", "runtime_cxxabi"),
    ):
        if version_tuple(str(report[report_key])) > version_tuple(str(metadata[runtime_key])):
            raise BuildError(f"wheel {report_key} exceeds measured builder runtime")


def inspect_builder_labels(
    ref: str,
    runtime_digest: str,
    requirements_hash: str,
    verifier_hash: str,
    platform_name: str,
    engine: str,
) -> dict[str, str]:
    result = run([engine, "inspect", "--format", "{{json .Config.Labels}}", ref], capture=True)
    try:
        labels = json.loads(result.stdout.strip())
    except json.JSONDecodeError as error:
        raise BuildError("builder labels are not valid JSON") from error
    expected = {
        "io.dimos.builder": "true",
        "io.dimos.builder-base-digest": runtime_digest.split("@", 1)[1],
        "io.dimos.builder-requirements-sha256": requirements_hash,
        "io.dimos.builder-verifier-sha256": verifier_hash,
        "io.dimos.builder-platform": platform_name,
    }
    for key, value in expected.items():
        if labels.get(key) != value:
            raise BuildError(f"builder label {key} does not match measured expectation")
    return {key: str(labels[key]) for key in expected}


def remove_auto_builder(ref: str, engine: str) -> bool:
    """Remove only a builder tag created by this invocation."""
    try:
        run([engine, "rmi", "--force", ref], capture=True)
    except subprocess.CalledProcessError:
        # Cleanup must not hide the build error (or make a successful build
        # fail because the engine already removed the temporary tag).
        return False
    try:
        run([engine, "image", "exists", ref], capture=True)
    except subprocess.CalledProcessError:
        return True
    return False


def require_builder_cleanup(clean: bool) -> None:
    if not clean:
        raise BuildError("automatic builder image/tag remains after wheel build")


def project_version() -> str:
    for line in (ROOT / "pyproject.toml").read_text().splitlines():
        if line.startswith("version = "):
            return line.split('"')[1]
    raise BuildError("project version not found")


def validate_requirements(path: Path) -> None:
    text = path.read_text()
    pending = False
    pending_has_hash = False
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or stripped.startswith("--"):
            if stripped.startswith("--hash="):
                pending_has_hash = True
            continue
        if not line[:1].isspace():
            if pending and not pending_has_hash:
                raise BuildError("unhashed requirements entry")
            pending = True
            pending_has_hash = "--hash=" in stripped
        lowered = stripped.lower()
        if re.search(
            r"(?:-e\s|--editable\s|file:|git\+|hg\+|svn\+|bzr\+|(?:^|\s)\.{0,2}/|(?:^|\s)/)",
            lowered,
        ):
            raise BuildError("requirements export contains a local, editable, or VCS reference")
        if re.search(r"extra\s*==|\[default\]", lowered):
            raise BuildError("requirements export contains extras or default groups")
        if re.match(r"dimos(?:[<=>!~\[]|\s|$)", lowered):
            raise BuildError("requirements export must not contain the root Dimos project")
        if "--hash=" not in stripped:
            continue
        pending_has_hash = True
    if pending and not pending_has_hash:
        raise BuildError("unhashed requirements entry")


def verify_uv(tmp: Path | None = None, *, allow_dirty: bool = False) -> str:
    executable = shutil.which("uv")
    if executable is None:
        raise BuildError("pinned uv executable is unavailable")
    output = run([executable, "--version"], capture=True).stdout.strip()
    match = re.search(r"\b(\d+\.\d+\.\d+)\b", output)
    if not match or match.group(1) != UV_VERSION:
        if not allow_dirty or tmp is None or shutil.which("uvx") is None:
            raise BuildError(f"uv version must be {UV_VERSION}, got {output!r}")
        pinned = tmp / "uv-pinned"
        pinned.write_text(f'#!/bin/sh\nexec uvx --from uv=={UV_VERSION} uv "$@"\n')
        pinned.chmod(0o755)
        pinned_output = run([str(pinned), "--version"], capture=True).stdout.strip()
        if not pinned_output.startswith(f"uv {UV_VERSION}"):
            raise BuildError(f"pinned uv bootstrap did not produce {UV_VERSION}: {pinned_output!r}")
        return str(pinned)
    return executable


def build_sdist_with_pinned_frontend(
    sdist_dir: Path, epoch: str, tmp: Path, *, allow_dirty: bool
) -> dict[str, str]:
    """Build the host sdist in a temporary, hash-locked packaging environment."""
    uv = verify_uv(tmp, allow_dirty=allow_dirty)
    packaging_venv = tmp / "packaging-venv"
    run([uv, "venv", "--python", sys.executable, str(packaging_venv)], cwd=tmp, capture=True)
    python = packaging_venv / "bin" / "python"
    if not python.exists():
        raise BuildError("packaging venv did not produce its Python executable")
    run(
        [
            uv,
            "pip",
            "install",
            "--python",
            str(python),
            "--require-hashes",
            "--requirement",
            str(PACKAGING_LOCK),
        ],
        cwd=tmp,
        capture=True,
    )
    sanitized = {
        "PATH": f"{packaging_venv / 'bin'}:/usr/bin:/bin",
        "HOME": str(tmp / "home"),
        "TMPDIR": str(tmp / "host-tmp"),
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
        "SOURCE_DATE_EPOCH": epoch,
        "PYTHONHASHSEED": "0",
        "PYTHONNOUSERSITE": "1",
    }
    for directory in (Path(sanitized["HOME"]), Path(sanitized["TMPDIR"])):
        directory.mkdir()
    run(
        [str(python), "-m", "build", "--sdist", "--no-isolation", "--outdir", str(sdist_dir)],
        cwd=ROOT,
        capture=True,
        env=sanitized,
    )
    frontend = run(
        [str(python), "-c", "import importlib.metadata as m; print(m.version('build'))"],
        capture=True,
    ).stdout.strip()
    backend = run(
        [str(python), "-c", "import importlib.metadata as m; print(m.version('setuptools'))"],
        capture=True,
    ).stdout.strip()
    return {
        "packaging_uv": UV_VERSION,
        "packaging_frontend": frontend,
        "packaging_backend": backend,
        "packaging_lock_sha256": sha256(PACKAGING_LOCK),
    }


def bootstrap_builder(
    base_image: str, tmp: Path, engine: str, allow_dirty: bool
) -> tuple[str, str]:
    if not allow_dirty:
        raise BuildError("local builder bootstrap is only permitted for --allow-dirty builds")
    context = tmp / "builder-context"
    context.mkdir()
    requirements = HERE / "builder-requirements.lock"
    shutil.copy2(HERE / "Builder.Containerfile", context / "Builder.Containerfile")
    shutil.copy2(requirements, context / requirements.name)
    shutil.copy2(HERE / "builder_verify.py", context / "builder_verify.py")
    if {item.name for item in context.iterdir()} != {
        "Builder.Containerfile",
        "builder-requirements.lock",
        "builder_verify.py",
    }:
        raise BuildError("builder context contains an unallowlisted artifact")
    iidfile = tmp / "builder.iid"
    builder_tag = f"dimos-spatial-builder:{os.getpid()}"
    args = [
        engine,
        "build",
        "--platform",
        native_platform(),
        "--iidfile",
        str(iidfile),
        "--file",
        "Builder.Containerfile",
        "--tag",
        builder_tag,
        "--http-proxy=false",
        "--build-arg",
        f"BASE_IMAGE={base_image}",
        "--build-arg",
        f"BUILDER_BASE_DIGEST={base_image.split('@', 1)[1]}",
        "--build-arg",
        f"BUILDER_REQUIREMENTS_SHA256={sha256(requirements)}",
        "--build-arg",
        f"BUILDER_VERIFIER_SHA256={sha256(HERE / 'builder_verify.py')}",
        "--build-arg",
        f"BUILDER_PLATFORM={native_platform()}",
        str(context),
    ]
    try:
        run(args, capture=True)
    except BaseException:
        remove_auto_builder(builder_tag, engine)
        raise
    builder_id = iidfile.read_text().strip() if iidfile.exists() else ""
    if not re.fullmatch(r"sha256:[0-9a-f]{64}", builder_id):
        remove_auto_builder(builder_tag, engine)
        raise BuildError("builder iidfile did not contain exactly sha256:<64hex>")
    return builder_tag, builder_id


def build(
    *,
    base_image: str,
    tag: str,
    engine: str = "podman",
    allow_dirty: bool = False,
    builder_image: str | None = None,
) -> BuildResult:
    validate_base_image(base_image)
    if not tag or tag.startswith("-"):
        raise BuildError("an output image tag is required")
    revision, dirty, tree, working_tree, publishable = source_identity(allow_dirty)
    target_platform = native_platform()
    with tempfile.TemporaryDirectory(prefix="dimos-spatial-build-") as tmp_name:
        tmp = Path(tmp_name)
        sdist_dir = tmp / "sdist"
        sdist_dir.mkdir()
        epoch = source_date_epoch()
        packaging_metadata = build_sdist_with_pinned_frontend(
            sdist_dir, epoch, tmp, allow_dirty=allow_dirty
        )
        sdists = sorted(sdist_dir.glob("dimos-*.tar.gz"))
        if len(sdists) != 1:
            raise BuildError("uv build must produce exactly one sdist")
        inspect_sdist(sdists[0], require_git_members=publishable)
        normalize_sdist(sdists[0], int(epoch))
        version, sdist_hash, sdist_inventory_hash, sdist_inventory = inspect_sdist(
            sdists[0], require_git_members=publishable
        )
        builder_ref, builder_id = (
            (builder_image, "")
            if builder_image
            else bootstrap_builder(base_image, tmp, engine, allow_dirty)
        )
        auto_builder = builder_image is None
        cleanup_ok = True
        try:
            builder_labels = inspect_builder_labels(
                builder_ref or "",
                base_image,
                sha256(PACKAGING_LOCK),
                sha256(HERE / "builder_verify.py"),
                target_platform,
                engine,
            )
            validate_builder_image(
                builder_ref or "", base_image, publishable=publishable, engine=engine
            )
            if not builder_id:
                builder_id = inspect_image_id(builder_ref or "", engine)
            wheel_dir = tmp / "wheel"
            wheel_dir.mkdir()
            builder_args = [
                engine,
                "run",
                "--rm",
                "--network=none",
                "--read-only",
                "--userns=keep-id",
                "--cap-drop=ALL",
                "--security-opt=no-new-privileges",
                "--mount",
                f"type=bind,src={sdists[0]},dst=/input/source.tar.gz,ro",
                "--mount",
                f"type=bind,src={wheel_dir},dst=/out,rw",
                "--mount",
                "type=tmpfs,destination=/build,tmpfs-mode=0777,tmpfs-size=4294967296",
                "--pids-limit=512",
                "--memory=8g",
                "--cpus=4",
                "--env",
                f"SOURCE_DATE_EPOCH={epoch}",
                "--env",
                f"EXPECTED_SDIST_SHA256={sdist_hash}",
                builder_ref or "",
                "--sdist",
                "/input/source.tar.gz",
                "--out",
                "/out",
                "--expected-arch",
                target_platform,
                "--source-date-epoch",
                epoch,
            ]
            run(builder_args, capture=True)
            wheels = sorted(wheel_dir.glob("dimos-*.whl"))
            if len(wheels) != 1:
                raise BuildError("offline builder must produce exactly one wheel")
            version, wheel_hash = inspect_wheel(
                wheels[0], target_platform, {str(item["path"]) for item in sdist_inventory}
            )
            abi_report_path = wheel_dir / "abi-report.json"
            if not abi_report_path.exists():
                raise BuildError("builder did not produce ABI report")
            abi_report = json.loads(abi_report_path.read_text())
            builder_metadata_path = wheel_dir / "builder-metadata.json"
            if not builder_metadata_path.exists():
                raise BuildError("builder did not produce measured metadata")
            builder_metadata = json.loads(builder_metadata_path.read_text())
            validate_json_object(
                builder_metadata,
                {
                    "python",
                    "glibc",
                    "compiler",
                    "python_inventory_sha256",
                    "runtime_glibc",
                    "runtime_glibcxx",
                    "runtime_cxxabi",
                },
                "builder metadata",
            )
            abi_report = validate_abi_report(
                abi_report, wheels[0].name, wheel_hash, target_platform
            )
            validate_runtime_abi(abi_report, builder_metadata)
        finally:
            if auto_builder:
                cleanup_ok = remove_auto_builder(builder_ref or "", engine)
                if not cleanup_ok:
                    print(
                        "warning: automatic builder cleanup could not be verified", file=sys.stderr
                    )
        if auto_builder:
            require_builder_cleanup(cleanup_ok)
        lock = tmp / "requirements.lock"
        pinned_uv = verify_uv(tmp, allow_dirty=allow_dirty)
        run(
            [
                pinned_uv,
                *BARE_EXPORT_ARGS,
                "--output-file",
                str(lock),
            ],
            cwd=ROOT,
            capture=True,
        )
        validate_requirements(lock)
        manifest: dict[str, object] = {
            "schema": "build-manifest.v1",
            "dirty": dirty,
            "publishable": publishable,
            "source_revision": revision,
            "source_tree_identity": tree,
            "head_tree_identity": tree,
            "working_tree_identity": working_tree,
            "dirty_policy": "rejected" if not dirty else "allowed-local-only",
            "dimos_version": version,
            "runtime_base": base_image,
            "runtime_base_digest": base_image.split("@", 1)[1],
            "builder_image": builder_ref,
            "builder_id": builder_id,
            "builder_ephemeral": auto_builder,
            "builder_base_digest": base_image.split("@", 1)[1],
            "builder_requirements_sha256": builder_labels["io.dimos.builder-requirements-sha256"],
            "builder_verifier_sha256": builder_labels["io.dimos.builder-verifier-sha256"],
            "builder_platform": builder_labels["io.dimos.builder-platform"],
            "builder_python": builder_metadata["python"],
            "builder_glibc": builder_metadata["glibc"],
            "builder_compiler": builder_metadata["compiler"],
            "builder_python_inventory_sha256": builder_metadata["python_inventory_sha256"],
            "runtime_glibc": builder_metadata["runtime_glibc"],
            "runtime_glibcxx": builder_metadata["runtime_glibcxx"],
            "runtime_cxxabi": builder_metadata["runtime_cxxabi"],
            "sdist_filename": sdists[0].name,
            "sdist_sha256": sdist_hash,
            "sdist_inventory_sha256": sdist_inventory_hash,
            "wheel_filename": wheels[0].name,
            "wheel_sha256": wheel_hash,
            "requirements_sha256": sha256(lock),
            "lock_sha256": sha256(ROOT / "uv.lock"),
            "python": "3.12",
            "architecture": target_platform,
            "uv_version": UV_VERSION,
            "max_glibc": abi_report["max_glibc"],
            "glibcxx": abi_report["glibcxx"],
            "cxxabi": abi_report["cxxabi"],
            "max_glibcxx": abi_report["max_glibcxx"],
            "max_cxxabi": abi_report["max_cxxabi"],
            "source_payload_identity": sdist_inventory_hash,
            "auditwheel_report_sha256": abi_report["auditwheel_report_sha256"],
            "auditwheel_report": abi_report["auditwheel_report"],
            **packaging_metadata,
        }
        context = tmp / "context"
        context.mkdir()
        shutil.copy2(HERE / "Containerfile", context / "Containerfile")
        shutil.copy2(wheels[0], context / wheels[0].name)
        shutil.copy2(lock, context / "requirements.lock")
        (context / "build-manifest.v1.json").write_text(
            json.dumps(manifest, sort_keys=True, separators=(",", ":")) + "\n"
        )
        if {p.name for p in context.iterdir()} != {
            "Containerfile",
            wheels[0].name,
            "requirements.lock",
            "build-manifest.v1.json",
        }:
            raise BuildError("build context contains an unallowlisted artifact")
        iidfile = tmp / "image.iid"
        build_args = [
            engine,
            "build",
            "--platform",
            target_platform,
            "--iidfile",
            str(iidfile),
            "--file",
            "Containerfile",
            "--tag",
            tag,
            "--build-arg",
            f"BASE_IMAGE={base_image}",
            "--build-arg",
            f"SOURCE_REVISION={revision}",
            "--build-arg",
            f"DIMOS_VERSION={version}",
            "--build-arg",
            f"WHEEL_FILENAME={wheels[0].name}",
            "--build-arg",
            f"SDIST_SHA256={sdist_hash}",
            "--build-arg",
            f"BUILDER_IMAGE={builder_ref}",
            "--build-arg",
            f"BUILDER_ID={builder_id}",
            "--build-arg",
            f"BUILDER_EPHEMERAL={str(auto_builder).lower()}",
            "--build-arg",
            f"BUILDER_VERIFIER_SHA256={sha256(HERE / 'builder_verify.py')}",
            "--build-arg",
            f"WHEEL_SHA256={wheel_hash}",
            "--build-arg",
            f"REQUIREMENTS_SHA256={manifest['requirements_sha256']}",
            "--build-arg",
            f"LOCK_SHA256={manifest['lock_sha256']}",
            "--build-arg",
            f"TARGET_ARCH={target_platform}",
            "--build-arg",
            f"SOURCE_TREE_IDENTITY={tree}",
            "--build-arg",
            f"WORKING_TREE_IDENTITY={working_tree or ''}",
            "--build-arg",
            f"SOURCE_PAYLOAD_IDENTITY={sdist_inventory_hash}",
            "--build-arg",
            f"PUBLISHABLE={str(publishable).lower()}",
            "--build-arg",
            f"DIRTY={str(dirty).lower()}",
            str(context),
        ]
        run(build_args, capture=True)
        image_id = iidfile.read_text().strip() if iidfile.exists() else ""
        if not re.fullmatch(r"sha256:[0-9a-f]{64}", image_id):
            raise BuildError("engine iidfile did not contain exactly sha256:<64hex>")
        print(f"built {tag} ({image_id})")
        return BuildResult(image_id=image_id, manifest=manifest)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-image", required=True)
    parser.add_argument("--tag", required=True)
    parser.add_argument("--engine", default="podman")
    parser.add_argument("--allow-dirty", action="store_true")
    parser.add_argument("--builder-image")
    args = parser.parse_args(argv)
    try:
        build(
            base_image=args.base_image,
            tag=args.tag,
            engine=args.engine,
            allow_dirty=args.allow_dirty,
            builder_image=args.builder_image,
        )
    except subprocess.CalledProcessError as error:
        operation = "podman build" if error.cmd and error.cmd[0] == args.engine else "build command"
        print(command_failure(error, operation), file=sys.stderr)
        return 2
    except BuildError as error:
        print(f"build failed: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
