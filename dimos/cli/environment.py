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

"""Load CLI settings before imports that read the environment."""

import json
import os
from pathlib import Path

from dotenv import load_dotenv

from dimos import DIMOS_PROJECT_ROOT, IS_CHECKOUT

# A daemon can re-exec the CLI. Carry the names across exec without copying values.
DOTENV_KEYS_ENV = "_DIMOS_DOTENV_KEYS"


def dotenv_keys() -> set[str]:
    return set(json.loads(os.environ.get(DOTENV_KEYS_ENV, "[]")))


def load_cli_dotenv() -> None:
    inherited = set(os.environ)
    loaded = dotenv_keys()
    work_file = Path.cwd() / ".env"
    checkout_file = DIMOS_PROJECT_ROOT / ".env"
    if IS_CHECKOUT and checkout_file != work_file:
        load_dotenv(checkout_file)
    load_dotenv(work_file)
    loaded.update(os.environ.keys() - inherited)
    if loaded:
        os.environ[DOTENV_KEYS_ENV] = json.dumps(sorted(loaded))
