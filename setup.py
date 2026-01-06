# Copyright 2025 Dimensional Inc.
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

import os

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import find_packages, setup

# C++ extensions
ext_modules = [
    Pybind11Extension(
        "dimos.navigation.global_planner.min_cost_astar_ext",
        [os.path.join("dimos", "navigation", "global_planner", "min_cost_astar_cpp.cpp")],
        extra_compile_args=[
            "-O3",  # Maximum optimization
            "-march=native",  # Optimize for current CPU
            "-ffast-math",  # Fast floating point
        ],
        define_macros=[
            ("NDEBUG", "1"),
        ],
    ),
]

setup(
    packages=find_packages(),
    package_dir={"": "."},
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
