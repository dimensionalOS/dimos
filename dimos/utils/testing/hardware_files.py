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

"""Serve a HardwareModule built outside a blueprint, so fetch_hardware_files reaches it as it would in one."""

from typing import TypeVar

from dimos.core.hardware_files import hardware_file_topics
from dimos.core.hardware_module import HardwareModule
from dimos.core.transport_factory import make_transport
from dimos.msgs.files.FileInfo import FileInfo

HardwareT = TypeVar("HardwareT", bound=HardwareModule)


def serving_hardware_files(module: HardwareT) -> HardwareT:
    """The module, its hardware_file streams on the topics a blueprint gives them, answering requests."""
    request_topic, response_topic = hardware_file_topics(module.namespace)
    module.hardware_file_request.transport = make_transport(request_topic, FileInfo)
    module.hardware_file_response.transport = make_transport(response_topic, FileInfo)
    module.serve_hardware_files()
    return module
