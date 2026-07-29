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

"""Zenoh's shared-memory transport needs more locked memory than systemd grants."""

from __future__ import annotations

import os
from pathlib import Path
import pwd
import resource
import subprocess
import tempfile

from dimos.protocol.service.system_configurator.base import SystemConfigurator
from dimos.utils import prompt
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Zenoh allocates a 16 MiB SHM pool per session (transport/shared_memory/
# transport_optimization/pool_size) and locks it. systemd's compiled-in
# DefaultLimitMEMLOCK is 8 MiB, so the mlock fails with ENOMEM and the session
# silently drops to the non-SHM path. 64 MiB leaves room for several sessions.
REQUIRED_MEMLOCK_BYTES = 64 * 1024 * 1024

# pam_limits reads this at login, so it never helps the run that writes it --
# fix() raises the live process for that. Scoped to the invoking user rather
# than "*": rlimits have no per-application granularity, so the user is the
# narrowest domain pam_limits can express.
LIMITS_FILE = Path("/etc/security/limits.d/99-dimos-memlock.conf")


class MemlockConfiguratorLinux(SystemConfigurator):
    """Raise RLIMIT_MEMLOCK so zenoh can lock its shared-memory pool."""

    # Zenoh falls back to its normal transport when the pool can't be locked, so
    # a declined fix costs throughput on same-host traffic, never correctness.
    critical = False

    def __init__(self, required_bytes: int = REQUIRED_MEMLOCK_BYTES) -> None:
        self.required_bytes = required_bytes
        self.soft, self.hard = resource.getrlimit(resource.RLIMIT_MEMLOCK)

    @property
    def _prlimit_cmd(self) -> list[str]:
        return [
            "prlimit",
            f"--pid={os.getpid()}",
            f"--memlock={self.required_bytes}:{self.required_bytes}",
        ]

    def _sufficient(self, value: int) -> bool:
        return value == resource.RLIM_INFINITY or value >= self.required_bytes

    @property
    def _user(self) -> str:
        return str(pwd.getpwuid(os.getuid()).pw_name)

    def _persisted_bytes(self) -> int:
        """Largest memlock value in the drop-in that applies to this user, in bytes."""
        best = 0
        try:
            for line in LIMITS_FILE.read_text().splitlines():
                fields = line.split()
                if len(fields) == 4 and fields[2] == "memlock" and fields[0] in (self._user, "*"):
                    best = max(best, int(fields[3]) * 1024)
        except (OSError, ValueError):
            return 0
        return best

    def check(self) -> bool:
        self.soft, self.hard = resource.getrlimit(resource.RLIMIT_MEMLOCK)
        if self._sufficient(self.soft):
            return True
        # Raising the soft limit up to the hard limit needs no privileges, so a
        # generous hard limit means there is nothing to ask about.
        if self._sufficient(self.hard):
            resource.setrlimit(resource.RLIMIT_MEMLOCK, (self.required_bytes, self.hard))
            self.soft, self.hard = resource.getrlimit(resource.RLIMIT_MEMLOCK)
            logger.info("Raised RLIMIT_MEMLOCK soft limit to %d bytes", self.soft)
            return True
        # The drop-in is written but pam_limits only reads it at login, so this
        # session is stuck low. Asking again every run would never help -- say
        # what actually clears it instead.
        if self._sufficient(self._persisted_bytes()):
            logger.warning(
                "%s is configured but this login session still has a %dMB locked-memory "
                "limit; log out and back in to pick it up. Zenoh runs without shared "
                "memory until then.",
                LIMITS_FILE,
                self.soft // 1024 // 1024,
            )
            return True
        return False

    def explanation(self) -> str | None:
        if self.check():
            return None
        need = self.required_bytes // 1024 // 1024
        have = self.soft // 1024 // 1024
        return (
            f"Zenoh shared memory: locked-memory limit is {have}MB, below the {need}MB "
            "its SHM pool needs. Without it zenoh logs 'Unable to create POSIX shm "
            "segment: OS error 12' and falls back to a slower same-host transport.\n"
            f"- This run: sudo {' '.join(self._prlimit_cmd)}\n"
            f"- Future logins: write {LIMITS_FILE} for user {self._user} "
            "(needs a re-login to take effect)"
        )

    def fix(self) -> None:
        # Raise the running process first: children inherit it, and unlike the
        # limits.d file this takes effect without logging out.
        prompt.sudo_run(*self._prlimit_cmd, check=True, text=True, capture_output=True)
        self.soft, self.hard = resource.getrlimit(resource.RLIMIT_MEMLOCK)
        logger.info("Raised RLIMIT_MEMLOCK to %d bytes for this run", self.soft)
        self._persist()

    def _persist(self) -> None:
        """Write the pam_limits drop-in so this user's next login starts configured."""
        kb = self.required_bytes // 1024
        content = (
            f"# Written by dimos: zenoh's SHM pool must be lockable.\n"
            f"{self._user}\t-\tmemlock\t{kb}\n"
        )
        try:
            with tempfile.NamedTemporaryFile("w", suffix=".conf", delete=False) as handle:
                handle.write(content)
                staged = handle.name
            prompt.sudo_run("install", "-m", "644", staged, str(LIMITS_FILE), check=True, text=True)
            os.unlink(staged)
            logger.info("Wrote %s for user %s (applies at next login)", LIMITS_FILE, self._user)
        except subprocess.CalledProcessError as error:
            # The live process is already fixed; persistence is a bonus.
            logger.warning("Could not write %s: rc=%d", LIMITS_FILE, error.returncode)
