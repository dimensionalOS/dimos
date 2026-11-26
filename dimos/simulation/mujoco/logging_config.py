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

import mujoco

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _log_warning(message: str) -> None:
    logger.warning("MuJoCo warning", warning=message)


def configure_mujoco_logging() -> None:
    """Route process-wide warnings through DimOS instead of ./MUJOCO_LOG.TXT.

    Call before loading models in each simulation process. The callback remains
    installed for the process lifetime, including when multiple engines share it.
    DimOS logging handles run directories, inherited worker configuration and
    standalone log destinations.
    """
    mujoco.set_mju_user_warning(_log_warning)
