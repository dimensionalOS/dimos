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

import shutil

import pytest

from dimos.robot.unitree.g1.jetson_clocks import require_jetson_clocks


@pytest.fixture
def jetson(tmp_path):
    release = tmp_path / "nv_tegra_release"
    release.write_text("# R36 (release)\n")
    sys_root = tmp_path / "sys"
    for policy in ("policy0", "policy4"):
        cpu = sys_root / "devices/system/cpu/cpufreq" / policy
        cpu.mkdir(parents=True)
        (cpu / "scaling_min_freq").write_text("1984000\n")
        (cpu / "scaling_max_freq").write_text("1984000\n")
        (cpu / "scaling_governor").write_text("schedutil\n")
        (cpu / "scaling_cur_freq").write_text("729600\n")
    gpu = sys_root / "class/devfreq/17000000.gpu"
    gpu.mkdir(parents=True)
    (gpu / "min_freq").write_text("918000000\n")
    (gpu / "max_freq").write_text("918000000\n")
    (gpu / "governor").write_text("nvhost_podgov\n")
    (gpu / "cur_freq").write_text("306000000\n")
    return {"tegra_release": release, "sys_root": sys_root}


def test_locked_limits_pass_despite_dynamic_governors_and_instantaneous_clocks(jetson):
    assert require_jetson_clocks(**jetson) is None


def test_checks_every_cpu_policy(jetson):
    minimum = jetson["sys_root"] / "devices/system/cpu/cpufreq/policy4/scaling_min_freq"
    minimum.write_text("729600\n")

    with pytest.raises(RuntimeError, match="policy4 min=729.6 MHz, max=1984 MHz"):
        require_jetson_clocks(**jetson)


def test_unlocked_gpu_error_explains_how_to_fix_the_clock_limits(jetson):
    (jetson["sys_root"] / "class/devfreq/17000000.gpu/min_freq").write_text("306000000\n")

    with pytest.raises(RuntimeError) as error:
        require_jetson_clocks(**jetson)

    assert "17000000.gpu min=306 MHz, max=918 MHz" in str(error.value)
    assert "sudo jetson_clocks" in str(error.value)
    assert "sudo bash bin/hardware/g1/install-jetson-clocks" in str(error.value)


@pytest.mark.parametrize("bad_value", ["not a number", "0", "-1"])
def test_invalid_limits_fail_closed(jetson, bad_value):
    gpu = jetson["sys_root"] / "class/devfreq/17000000.gpu"
    (gpu / "min_freq").write_text(bad_value)
    (gpu / "max_freq").write_text(bad_value)

    with pytest.raises(RuntimeError, match="SONIC requires locked Jetson"):
        require_jetson_clocks(**jetson)


def test_missing_frequency_file_fails_closed(jetson):
    (jetson["sys_root"] / "class/devfreq/17000000.gpu/max_freq").unlink()

    with pytest.raises(RuntimeError, match="cannot read clock limits for 17000000.gpu"):
        require_jetson_clocks(**jetson)


@pytest.mark.parametrize(
    "path, error",
    [
        ("devices/system/cpu/cpufreq", "no CPU clock policies"),
        ("class/devfreq", "no Jetson GPU clock device"),
    ],
)
def test_missing_clock_devices_fail_closed(jetson, path, error):
    shutil.rmtree(jetson["sys_root"] / path)

    with pytest.raises(RuntimeError, match=error):
        require_jetson_clocks(**jetson)


@pytest.mark.parametrize("name", ["17000000.ga10b", "17000000.gv11b", "57000000.gp10b"])
def test_recognizes_jetson_gpu_device_names(jetson, name):
    devfreq = jetson["sys_root"] / "class/devfreq"
    (devfreq / "17000000.gpu").rename(devfreq / name)
    # Other accelerators also use devfreq and must not be mistaken for the GPU.
    (devfreq / "15380000.nvjpg").mkdir()

    assert require_jetson_clocks(**jetson) is None


def test_non_jetson_hosts_need_no_jetson_clock_files(tmp_path):
    assert (
        require_jetson_clocks(
            tegra_release=tmp_path / "missing_release", sys_root=tmp_path / "missing_sys"
        )
        is None
    )
