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

from __future__ import annotations

from abc import ABC, abstractmethod
import logging
import os
import subprocess

from dimos.utils import prompt

logger = logging.getLogger(__name__)


def _read_sysctl_int(name: str) -> int | None:
    try:
        result = subprocess.run(["sysctl", name], capture_output=True, text=True)
        if result.returncode != 0:
            logger.error(
                "[sysctl] `sysctl %s` rc=%d stderr=%r", name, result.returncode, result.stderr
            )
            return None

        text = result.stdout.strip().replace(":", "=")
        if "=" not in text:
            logger.error("[sysctl] unexpected output for %s: %r", name, text)
            return None

        return int(text.split("=", 1)[1].strip())
    except Exception as error:
        logger.error("[sysctl] reading %s: %s", name, error)
        return None


def _write_sysctl_int(name: str, value: int) -> None:
    prompt.sudo_run("sysctl", "-w", f"{name}={value}", check=True, text=True, capture_output=True)


# base class for system config checks/requirements


class SystemConfigurator(ABC):
    critical: bool = False

    @abstractmethod
    def check(self) -> bool:
        """Return True if configured. Log errors and return False on uncertainty."""
        raise NotImplementedError

    @abstractmethod
    def explanation(self) -> str | None:
        """
        Return a human-readable summary of what would be done (sudo commands) if not configured.
        Return None when no changes are needed.
        """
        raise NotImplementedError

    @abstractmethod
    def fix(self) -> None:
        """Apply fixes (may attempt sudo, catch, and apply fallback measures if needed)."""
        raise NotImplementedError


# generic enforcement of system configs


def configure_system(checks: list[SystemConfigurator], check_only: bool = False) -> None:
    if os.environ.get("CI"):
        logger.info("CI environment detected: skipping system configuration.")
        return

    # run checks
    failing = [check for check in checks if not check.check()]
    if not failing:
        return

    # ask for permission to modify system
    explanations: list[str] = [msg for check in failing if (msg := check.explanation()) is not None]

    if explanations:
        logger.warning("System configuration changes are recommended/required:\n")
        logger.warning("\n\n".join(explanations))

    if check_only:
        return

    if not prompt.confirm("\nApply these changes now?"):
        if any(check.critical for check in failing):
            raise SystemExit(1)
        return

    for check in failing:
        try:
            check.fix()
        except subprocess.CalledProcessError as error:
            if check.critical:
                logger.error("Critical fix failed rc=%d", error.returncode)
                logger.error("stdout: %s", error.stdout)
                logger.error("stderr: %s", error.stderr)
                raise
            logger.warning("Optional improvement failed: rc=%d", error.returncode)
            logger.warning("stdout: %s", error.stdout)
            logger.warning("stderr: %s", error.stderr)

    logger.info("System configuration completed.")
