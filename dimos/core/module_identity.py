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

from typing import Any


def external_module_name(module_class: type[Any]) -> str | None:
    """Qualify external classes by their definition, independent of registration."""
    module_path = module_class.__module__
    if module_path == "dimos" or module_path.startswith("dimos."):
        return None
    # Multiprocessing imports the entry script under this alias in workers.
    if module_path == "__mp_main__":
        module_path = "__main__"
    return f"{module_path}.{module_class.__qualname__}"
