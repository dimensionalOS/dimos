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

"""Install the fake Damiao CAN boundary in every process started by a test run.

A test driver adds this directory to ``PYTHONPATH``. Python imports
``sitecustomize`` at interpreter startup, which is also what the worker
forkserver does, so both the driver and every forked worker see the fake
boundary before any robot code can open a bus.
"""

from fake_native import install

install()
