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

"""Write source projects for unmodified upstream ROSIDL C++/FastRTPS generation."""

from __future__ import annotations

import json
from pathlib import Path
import shutil

from .definitions import Message


def write_project(output: Path, messages: tuple[Message, ...], module: str, version: str) -> None:
    packages: dict[str, list[Message]] = {}
    for message in messages:
        packages.setdefault(message.package, []).append(message)
    dependencies = {
        package: sorted(
            {name.split("/")[0] for msg in members for name in msg.dependencies} - {package}
        )
        for package, members in packages.items()
    }
    order: list[str] = []
    active: set[str] = set()

    def visit(package: str) -> None:
        if package in order or package not in packages:
            return
        if package in active:
            raise ValueError(f"Cyclic ROS package dependency: {package}")
        active.add(package)
        for dependency in dependencies[package]:
            visit(dependency)
        active.remove(package)
        order.append(package)

    for package in sorted(packages):
        visit(package)
    output.mkdir(parents=True, exist_ok=True)
    for package in order:
        root = output / package
        root.mkdir(exist_ok=True)
        for message in packages[package]:
            target = root / "msg" / (message.short_name + ".msg")
            target.parent.mkdir(exist_ok=True)
            target.write_text(message.text)
        deps = dependencies[package]
        (root / "package.xml").write_text(
            f'<package format="3"><name>{package}</name><version>{version}</version>'
            "<description>DimOS message sources</description>"
            '<maintainer email="build@dimensionalos.com">DimOS</maintainer><license>Apache-2.0</license>'
            + "".join(f"<depend>{dep}</depend>" for dep in deps)
            + "<member_of_group>rosidl_interface_packages</member_of_group>"
            "<export><build_type>ament_cmake</build_type></export></package>\n"
        )
        (root / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.20)\n"
            f"project({package} VERSION {version})\n"
            + "".join(
                f"find_package({dep} REQUIRED)\n"
                for dep in [
                    "ament_cmake",
                    "rosidl_cmake",
                    "rosidl_adapter",
                    "rosidl_generator_type_description",
                    "rosidl_generator_c",
                    "rosidl_generator_cpp",
                    "rosidl_typesupport_fastrtps_cpp",
                    *deps,
                ]
            )
            + 'file(GLOB interfaces RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")\n'
            + "rosidl_generate_interfaces(${PROJECT_NAME} ${interfaces}"
            + (" DEPENDENCIES " + " ".join(deps) if deps else "")
            + ")\nament_package()\n"
        )
    shutil.copyfile(
        Path(__file__).with_name("templates") / "native-toolchain.cmake",
        output / "native-toolchain.cmake",
    )
    # The ordinary source superbuild configures each upstream message project
    # after its dependencies have installed their real CMake exports.
    cmake = f"cmake_minimum_required(VERSION 3.20)\nproject(dimos_message_sources VERSION {version} LANGUAGES NONE)\ninclude(ExternalProject)\ninclude(${{CMAKE_CURRENT_LIST_DIR}}/native-toolchain.cmake)\n"
    cmake += 'string(REPLACE ";" "|" _prefixes "${CMAKE_PREFIX_PATH}")\n'
    cmake += 'string(REPLACE ";" "|" _runtime_paths "${DIMOS_RUNTIME_PATHS}")\n'
    previous = ""
    for package in order:
        cmake += f'ExternalProject_Add({package} SOURCE_DIR "${{CMAKE_CURRENT_LIST_DIR}}/{package}" DOWNLOAD_COMMAND "" LIST_SEPARATOR | '
        cmake += "CMAKE_ARGS ${_dimos_toolchain_args} -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCMAKE_PREFIX_PATH=${_prefixes} -DPython3_EXECUTABLE=${Python3_EXECUTABLE} -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_RPATH=${_dimos_origin}|${_runtime_paths}"
        if previous:
            cmake += f" DEPENDS {previous}"
        cmake += ")\n"
        previous = package
    cmake += "include(CMakePackageConfigHelpers)\n"
    cmake += f"write_basic_package_version_file(${{CMAKE_CURRENT_BINARY_DIR}}/{module}ConfigVersion.cmake VERSION {version} COMPATIBILITY ExactVersion)\n"
    cmake += f"install(FILES {module}Config.cmake ${{CMAKE_CURRENT_BINARY_DIR}}/{module}ConfigVersion.cmake DESTINATION lib/cmake/{module})\n"
    (output / "CMakeLists.txt").write_text(cmake)
    (output / (module + "Config.cmake")).write_text(
        "include(CMakeFindDependencyMacro)\n"
        + "".join(f"find_dependency({p} {version} EXACT)\n" for p in order)
        + f"if(NOT TARGET {module}::messages)\nadd_library({module}::messages INTERFACE IMPORTED)\n"
        + f'set_property(TARGET {module}::messages PROPERTY INTERFACE_LINK_LIBRARIES "'
        + ";".join(f"{p}::{p}__rosidl_typesupport_fastrtps_cpp" for p in order)
        + '")\nendif()\n'
    )
    (output / "packages.json").write_text(json.dumps(order, indent=2) + "\n")
