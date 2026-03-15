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

from __future__ import annotations

from abc import abstractmethod
import sys
from typing import TYPE_CHECKING

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

if TYPE_CHECKING:
    from types import TracebackType

from reactivex.abc import DisposableBase
from reactivex.disposable import CompositeDisposable


class Resource(DisposableBase):
    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    def dispose(self) -> None:
        """
        Makes a Resource disposable
        So you can do a

        from reactivex.disposable import CompositeDisposable

        disposables = CompositeDisposable()

        transport1 = LCMTransport(...)
        transport2 = LCMTransport(...)

        disposables.add(transport1)
        disposables.add(transport2)

        ...

        disposables.dispose()

        """
        self.stop()

    def __enter__(self) -> Self:
        self.start()
        return self

    def __exit__(
        self,
        exctype: type[BaseException] | None,
        excinst: BaseException | None,
        exctb: TracebackType | None,
    ) -> None:
        self.stop()


class CompositeResource(Resource):
    """Resource that owns child disposables, disposed on stop()."""

    _disposables: CompositeDisposable

    def __init__(self) -> None:
        self._disposables = CompositeDisposable()

    def register_disposables(self, *disposables: DisposableBase) -> None:
        """Register child disposables to be disposed when this resource stops."""
        for d in disposables:
            self._disposables.add(d)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._disposables.dispose()
