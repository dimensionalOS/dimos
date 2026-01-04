#!/usr/bin/env python3
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

from ..support import prompt_tools as p
from ..support.env_setup.direnv import setup_direnv
from ..support.env_setup.dotenv import setup_dotenv
from ..support.misc import get_project_directory


def phase5():
    p.header("Next Phase: Environment configuration")

    project_path = get_project_directory()
    env_path = f"{project_path}/.env"
    envrc_path = f"{project_path}/.envrc"

    has_dotenv = setup_dotenv(project_path, env_path)
    if not has_dotenv:
        return

    setup_direnv(envrc_path)
    print()
    print()
    print("🎉 Setup complete! 🎉")
    print("Now you can start following the dimos examples")
