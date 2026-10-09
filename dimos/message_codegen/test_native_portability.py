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

"""Target selection/cache checks; real macOS builds run in the hosted CI matrix."""

from pathlib import Path
import subprocess

import pytest

from dimos.message_codegen import native_build


@pytest.fixture
def darwin_tools(monkeypatch):
    monkeypatch.setattr(native_build.sys, "platform", "darwin")
    for name in [
        "SDKROOT",
        "MACOSX_DEPLOYMENT_TARGET",
        "CMAKE_OSX_SYSROOT",
        "CMAKE_OSX_DEPLOYMENT_TARGET",
        "CMAKE_OSX_ARCHITECTURES",
        "CMAKE_TOOLCHAIN_FILE",
    ]:
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr(native_build.shutil, "which", lambda name: "/tools/" + name)
    monkeypatch.setattr(native_build, "find_spec", lambda name: True)
    monkeypatch.setattr(native_build.metadata, "version", lambda name: "1.0")

    def output(command, **kwargs):
        return "/SDKs/MacOSX.sdk\n" if command[0] == "xcrun" else "test compiler 1"

    monkeypatch.setattr(native_build.subprocess, "check_output", output)


def test_macos_sdk_and_target_are_forwarded_and_invalidate_cache(darwin_tools, monkeypatch):
    first = native_build._toolchain_key()
    assert native_build._cmake_toolchain_args() == ["-DCMAKE_OSX_SYSROOT=/SDKs/MacOSX.sdk"]
    monkeypatch.setenv("MACOSX_DEPLOYMENT_TARGET", "13.0")
    second = native_build._toolchain_key()
    assert second != first
    monkeypatch.setenv("CMAKE_OSX_ARCHITECTURES", "arm64;x86_64")
    third = native_build._toolchain_key()
    assert third != second
    monkeypatch.setenv("SDKROOT", "/SDKs/Other.sdk")
    assert native_build._toolchain_key() != third
    assert native_build._cmake_toolchain_args() == [
        "-DCMAKE_OSX_ARCHITECTURES=arm64;x86_64",
        "-DCMAKE_OSX_DEPLOYMENT_TARGET=13.0",
        "-DCMAKE_OSX_SYSROOT=/SDKs/Other.sdk",
    ]


def test_missing_macos_sdk_tools_fail_without_installing(darwin_tools, monkeypatch):
    monkeypatch.setattr(
        native_build.shutil, "which", lambda name: None if name == "xcrun" else "/tools/" + name
    )
    with pytest.raises(RuntimeError, match="Command Line Tools"):
        native_build._cmake_toolchain_args()


@pytest.mark.parametrize("apple,origin", [(True, "@loader_path"), (False, "$ORIGIN")])
def test_nested_cmake_projects_receive_target_and_platform_rpath(tmp_path, apple, origin):
    template = Path(native_build.__file__).parent / "templates/native-toolchain.cmake"
    script = tmp_path / "probe.cmake"
    script.write_text(
        f"set(APPLE {'TRUE' if apple else 'FALSE'})\n"
        'set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64")\n'
        'set(CMAKE_OSX_DEPLOYMENT_TARGET "13.0")\n'
        f'include("{template}")\n'
        'message("origin=${_dimos_origin}")\n'
        'message("args=${_dimos_toolchain_args}")\n'
    )
    result = subprocess.run(
        ["cmake", "-P", str(script)], capture_output=True, text=True, check=True
    )
    assert f"origin={origin}" in result.stderr
    assert "-DCMAKE_OSX_ARCHITECTURES=arm64|x86_64" in result.stderr
    assert "-DCMAKE_OSX_DEPLOYMENT_TARGET=13.0" in result.stderr
