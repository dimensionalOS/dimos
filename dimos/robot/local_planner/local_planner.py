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

from abc import abstractclass, abstractmethod
from dimos.types.vector import Vector, VectorLike, to_vector
from dimos.types.path import Path
from reactivex.observable import Observable


@abstractclass
class LocalPlanner:
    @abstractmethod
    def consume_path_stream(self, observable: Observable[Path]) -> bool: ...

    @abstractmethod
    def get_move_stream(self) -> Observable[Vector]: ...
