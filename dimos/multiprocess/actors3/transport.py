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

import enum
import inspect
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    List,
    Protocol,
    TypeVar,
    get_args,
    get_origin,
    get_type_hints,
)


class TransportProtocol(Protocol[T]):
    def broadcast(self, selfstream: Out, value: T): ...


class DirectTransportProtocol(Protocol[T]):
    def direct_msg(self, selfstream: Out, target: RemoteIn, value: T) -> None: ...


Transport = TransportProtocol | DirectTransportProtocol


class DaskTransport(DirectTransportProtocol):
    def msg(self, selfstream: Out[T], target: RemoteIn[T], value: T) -> None: ...


daskTransport = DaskTransport()  # singleton instance for use in Out/RemoteOut
