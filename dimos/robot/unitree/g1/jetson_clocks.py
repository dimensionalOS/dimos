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

"""Unprivileged Jetson clock checks before SONIC starts sending motor targets."""

from pathlib import Path

_GPU_NAMES = {"gpu", "ga10b", "gv11b", "gp10b"}


def _lock_issue(path: Path, min_file: str, max_file: str, units_per_mhz: int) -> str | None:
    try:
        minimum = int((path / min_file).read_text().strip())
        maximum = int((path / max_file).read_text().strip())
    except (OSError, ValueError) as exc:
        return f"cannot read clock limits for {path.name}: {exc}"
    if minimum > 0 and minimum == maximum:
        return None
    return (
        f"{path.name} min={minimum / units_per_mhz:g} MHz, "
        f"max={maximum / units_per_mhz:g} MHz (expected equal, positive limits)"
    )


def require_jetson_clocks(
    *,
    tegra_release: Path = Path("/etc/nv_tegra_release"),
    sys_root: Path = Path("/sys"),
) -> None:
    """Reject unlocked or unreadable CPU/GPU limits on Jetson; skip other hosts.

    Check configured limits, not instantaneous frequency, which can vary during
    thermal throttling. This does not validate the power mode, EMC clock, cooling,
    or inference deadlines. A root-owned boot service runs NVIDIA's jetson_clocks;
    the policy only reads sysfs and never requests elevated privileges.
    """
    if not tegra_release.is_file():
        return

    cpu_policies = sorted((sys_root / "devices/system/cpu/cpufreq").glob("policy[0-9]*"))
    gpu_devices = sorted(
        path
        for path in (sys_root / "class/devfreq").glob("*")
        if path.name.rsplit(".", 1)[-1] in _GPU_NAMES
    )
    issues: list[str] = []
    if not cpu_policies:
        issues.append("no CPU clock policies found")
    if not gpu_devices:
        issues.append("no Jetson GPU clock device found")
    for path in cpu_policies:
        issue = _lock_issue(path, "scaling_min_freq", "scaling_max_freq", 1000)
        if issue is not None:
            issues.append(issue)
    for path in gpu_devices:
        issue = _lock_issue(path, "min_freq", "max_freq", 1_000_000)
        if issue is not None:
            issues.append(issue)
    if issues:
        raise RuntimeError(
            "SONIC requires locked Jetson CPU/GPU clocks: "
            + "; ".join(issues)
            + ". Run 'sudo jetson_clocks' for this boot, or install the boot service with "
            "'sudo bash bin/hardware/g1/install-jetson-clocks' from the DimOS checkout."
        )
