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

"""Robot-asset cache inspection policy."""

import os
from pathlib import Path

from dimos.robot.assets.git_cache import git_checkout_protection_reason


def protected_robot_checkouts(robot_cache_root: Path) -> dict[Path, str]:
    """Return cached robot checkouts that contain or may contain local work."""
    sources_root = (robot_cache_root / "sources").absolute()
    if not os.path.lexists(sources_root) or sources_root.is_symlink() or not sources_root.is_dir():
        return {}

    protected: dict[Path, str] = {}

    def protect_unreadable(error: OSError) -> None:
        protected[sources_root] = f"could not inspect robot asset checkouts: {error}"

    for directory, dirnames, filenames in os.walk(
        sources_root,
        followlinks=False,
        onerror=protect_unreadable,
    ):
        if ".git" not in dirnames and ".git" not in filenames:
            continue

        checkout = Path(directory).absolute()
        reason = git_checkout_protection_reason(checkout)
        if reason is not None:
            protected[checkout] = reason

        # Nested repositories are part of the outer source checkout.
        dirnames.clear()

    return protected
