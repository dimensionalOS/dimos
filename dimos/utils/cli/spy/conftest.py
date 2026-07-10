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

"""Shared fixtures for the spy tests."""

import logging

import pytest

# core.py logs through this stdlib logger name (setup_logger() derives it
# from the module's file path).
_CORE_LOGGER = "dimos/utils/cli/spy/core.py"


@pytest.fixture
def spy_warnings(caplog):
    """Capture spy core log lines via ``caplog``.

    The dimos logger is structlog over a stdlib logger with
    ``propagate=False``, so caplog's root-level handler never sees it.
    """
    lg = logging.getLogger(_CORE_LOGGER)
    lg.addHandler(caplog.handler)
    caplog.set_level(logging.WARNING, logger=_CORE_LOGGER)
    try:
        yield caplog
    finally:
        lg.removeHandler(caplog.handler)
