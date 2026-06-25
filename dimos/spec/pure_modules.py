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

"""Pure modules — re-exported from :mod:`dimos.memory2.puremodule`.

A PureModule's core is a pure ``step`` over inputs aligned to a tick; the
same class runs live on pubsub ports or offline over stored memory2
streams. See the implementation module for the declaration language.
"""

from dimos.memory2.puremodule import PureModule, interpolate, latest, tick, window

__all__ = ["PureModule", "interpolate", "latest", "tick", "window"]
