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

"""Writable native build sources belonging to the installed SDK."""

from hashlib import sha256
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tarfile
import tempfile

from filelock import FileLock

from dimos.constants import CACHE_DIR, DIMOS_PROJECT_ROOT


def unpack_sources(bundle: Path, cache: Path) -> Path:
    digest = sha256(bundle.read_bytes()).hexdigest()
    root = cache / f"{digest}-{platform.system()}-{platform.machine()}"
    cache.mkdir(parents=True, exist_ok=True)
    with FileLock(str(root) + ".lock"):
        if (root / ".git").is_dir():
            return root
        temporary = Path(tempfile.mkdtemp(prefix="native-", dir=cache))
        try:
            with tarfile.open(bundle) as archive:
                archive.extractall(temporary, filter="data")
            # Existing nested flakes use repository-relative inputs. A local
            # snapshot lets Nix include their sibling sources without fetching
            # another DimOS checkout or writing into site-packages.
            (temporary / ".gitignore").write_text("target/\nresult\nresult-*\n.build.lock\n")
            git_env = {
                key: value for key, value in os.environ.items() if not key.startswith("GIT_")
            }
            git_env.update(GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM="1")
            for args in (
                ["init", "--quiet"],
                ["add", "."],
                [
                    "-c",
                    "user.name=DimOS",
                    "-c",
                    "user.email=build@dimos.invalid",
                    "-c",
                    "core.hooksPath=/dev/null",
                    "commit",
                    "--quiet",
                    "--no-gpg-sign",
                    "-m",
                    "Native SDK sources",
                ],
            ):
                subprocess.run(
                    ["git", *args], cwd=temporary, env=git_env, check=True, capture_output=True
                )
            temporary.rename(root)
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)
    return root


def native_source_root() -> Path:
    bundle = Path(__file__).resolve().parents[1] / "_native_sources.tar"
    if bundle.is_file():
        return unpack_sources(bundle, CACHE_DIR / "native")
    if (DIMOS_PROJECT_ROOT / "Cargo.toml").is_file():
        return DIMOS_PROJECT_ROOT
    raise FileNotFoundError("The DimOS SDK is missing its native sources. Reinstall the SDK.")
