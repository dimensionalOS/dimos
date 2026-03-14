# Copyright 2025-2026 Dimensional Inc.
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

"""Registry of available DIO sub-apps."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.utils.cli.dio.sub_app import SubApp


def get_sub_apps() -> list[type[SubApp]]:
    """Return all available sub-app classes in display order."""
    from dimos.utils.cli.dio.sub_apps.config import ConfigSubApp
    from dimos.utils.cli.dio.sub_apps.dtop import DtopSubApp
    from dimos.utils.cli.dio.sub_apps.humancli import HumanCLISubApp
    from dimos.utils.cli.dio.sub_apps.launcher import LauncherSubApp
    from dimos.utils.cli.dio.sub_apps.lcmspy import LCMSpySubApp
    from dimos.utils.cli.dio.sub_apps.runner import StatusSubApp

    return [
        LauncherSubApp,
        StatusSubApp,
        ConfigSubApp,
        DtopSubApp,
        LCMSpySubApp,
        HumanCLISubApp,
    ]
