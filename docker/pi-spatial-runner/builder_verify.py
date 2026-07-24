"""Build and ABI-check one inspected source distribution without network access."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import re
import subprocess
import tarfile
import tempfile
import zipfile

ELF = re.compile(rb"^\x7fELF")
GLIBC = re.compile(r"GLIBC_(\d+)\.(\d+)")
GLIBCXX = re.compile(r"GLIBCXX_(\d+)\.(\d+)(?:\.(\d+))?")
CXXABI = re.compile(r"CXXABI_(\d+)\.(\d+)(?:\.(\d+))?")


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def run(args: list[str], *, cwd: Path | None = None, env: dict[str, str] | None = None) -> str:
    result = subprocess.run(args, cwd=cwd, check=True, text=True, capture_output=True, env=env)
    return result.stdout + result.stderr


def versions(text: str, pattern: re.Pattern[str]) -> list[str]:
    values = []
    for match in pattern.finditer(text):
        values.append(".".join(part for part in match.groups() if part is not None))
    return sorted(set(values), key=lambda value: tuple(int(part) for part in value.split(".")))


def validate_glibc(values: list[str]) -> str:
    ceiling = max(
        values, key=lambda value: tuple(int(part) for part in value.split(".")), default="0.0"
    )
    if tuple(int(part) for part in ceiling.split(".")) > (2, 36):
        raise RuntimeError(f"wheel requires unsupported {ceiling}; runtime ceiling is GLIBC_2.36")
    return ceiling


def ceiling(values: list[str]) -> str:
    return max(
        values, key=lambda value: tuple(int(part) for part in value.split(".")), default="0.0"
    )


def builder_metadata() -> dict[str, str]:
    inventory = "\n".join(
        f"{dist.metadata['Name']}=={dist.version}"
        for dist in sorted(
            importlib.metadata.distributions(), key=lambda item: item.metadata["Name"].lower()
        )
    )
    libc = run(["ldd", "--version"]).splitlines()[0]
    compiler = run(["gcc", "--version"]).splitlines()[0]
    libstdcxx_path = next(Path("/usr/lib").glob("**/libstdc++.so.6"))
    libstdcxx = run(["readelf", "--version-info", str(libstdcxx_path)])
    libc_match = re.search(r"(\d+\.\d+)", libc)
    return {
        "python": __import__("sys").version,
        "glibc": libc,
        "compiler": compiler,
        "python_inventory_sha256": hashlib.sha256(inventory.encode()).hexdigest(),
        "runtime_glibc": libc_match.group(1) if libc_match else "0.0",
        "runtime_glibcxx": ceiling(versions(libstdcxx, GLIBCXX)),
        "runtime_cxxabi": ceiling(versions(libstdcxx, CXXABI)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdist", default="/input/source.tar.gz")
    parser.add_argument("--out", default="/out")
    parser.add_argument("--expected-arch", required=True)
    parser.add_argument("--source-date-epoch", required=True)
    args = parser.parse_args()
    source = Path(args.sdist)
    expected_source_hash = os.environ.get("EXPECTED_SDIST_SHA256")
    if not expected_source_hash or digest(source) != expected_source_hash:
        raise RuntimeError("sdist bytes do not match expected host hash")
    output = Path(args.out)
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir="/build") as directory:
        root = Path(directory)
        with tarfile.open(source, "r:gz") as archive:
            total = 0
            for member in archive:
                if member.size > 64 * 1024 * 1024:
                    raise RuntimeError(f"oversized sdist member: {member.name}")
                total += member.size
                if total > 512 * 1024 * 1024:
                    raise RuntimeError("sdist aggregate payload is too large")
        with tarfile.open(source, "r:gz") as archive:
            archive.extractall(root, filter="data")
        roots = [path for path in root.iterdir() if path.is_dir()]
        if len(roots) != 1:
            raise RuntimeError("sdist extraction did not produce one root")
        env = os.environ.copy()
        env.update(
            {
                "CIBUILDWHEEL": "1",
                "PYTHONHASHSEED": "0",
                "SOURCE_DATE_EPOCH": args.source_date_epoch,
            }
        )
        run(
            ["python", "-m", "build", "--wheel", "--no-isolation", "--outdir", str(output)],
            cwd=roots[0],
            env=env,
        )
    wheels = sorted(output.glob("dimos-*.whl"))
    if len(wheels) != 1:
        raise RuntimeError("builder must produce exactly one dimos wheel")
    platform_tag = (
        "manylinux_2_36_x86_64" if args.expected_arch == "linux/amd64" else "manylinux_2_36_aarch64"
    )
    repaired = output / "repaired"
    repaired.mkdir()
    run(
        [
            "auditwheel",
            "repair",
            "--plat",
            platform_tag,
            "--wheel-dir",
            str(repaired),
            str(wheels[0]),
        ]
    )
    wheels[0].unlink()
    repaired_wheels = sorted(repaired.glob("dimos-*.whl"))
    if len(repaired_wheels) != 1:
        raise RuntimeError("auditwheel must produce exactly one repaired wheel")
    repaired_wheels[0].replace(output / repaired_wheels[0].name)
    repaired.rmdir()
    wheels = sorted(output.glob("dimos-*.whl"))
    if len(wheels) != 1:
        raise RuntimeError("repaired output must contain exactly one wheel")
    audit = run(["auditwheel", "show", str(wheels[0])])
    glibc: list[str] = []
    glibcxx: list[str] = []
    cxxabi: list[str] = []
    with (
        zipfile.ZipFile(wheels[0]) as wheel,
        tempfile.TemporaryDirectory(dir="/build") as directory,
    ):
        for info in wheel.infolist():
            if info.filename.endswith("/"):
                continue
            data = wheel.read(info)
            if not ELF.match(data):
                continue
            target = Path(directory) / Path(info.filename).name
            target.write_bytes(data)
            header = run(["readelf", "-h", str(target)])
            if (
                args.expected_arch == "linux/amd64"
                and "Advanced Micro Devices X86-64" not in header
            ):
                raise RuntimeError(f"wrong ELF architecture: {info.filename}")
            if args.expected_arch == "linux/arm64" and "AArch64" not in header:
                raise RuntimeError(f"wrong ELF architecture: {info.filename}")
            version_report = run(["readelf", "--version-info", str(target)])
            glibc.extend(versions(version_report, GLIBC))
            glibcxx.extend(versions(version_report, GLIBCXX))
            cxxabi.extend(versions(version_report, CXXABI))
    max_glibc = validate_glibc(glibc)
    abi_report = {
        "architecture": args.expected_arch,
        "wheel_filename": wheels[0].name,
        "wheel_sha256": digest(wheels[0]),
        "auditwheel_report_sha256": hashlib.sha256(audit.encode()).hexdigest(),
        "auditwheel_report": audit[-4000:],
        "max_glibc": max_glibc,
        "glibcxx": sorted(set(glibcxx)),
        "cxxabi": sorted(set(cxxabi)),
        "max_glibcxx": ceiling(sorted(set(glibcxx))),
        "max_cxxabi": ceiling(sorted(set(cxxabi))),
    }
    (output / "abi-report.json").write_text(json.dumps(abi_report, sort_keys=True) + "\n")
    (output / "builder-metadata.json").write_text(
        json.dumps(builder_metadata(), sort_keys=True) + "\n"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
