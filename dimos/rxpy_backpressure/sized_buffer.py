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

from typing import Any

from rxpy_backpressure.function_runner import thread_function_runner
from rxpy_backpressure.locks import BooleanLock, Lock
from rxpy_backpressure.observer import Observer
from utils.logging import Logger
from utils.stats import Counter


class SizedBufferBackPressureStrategy(Observer):
    counter: Counter = Counter()

    def __init__(self, wrapped_observer: Observer, cache_size: int):
        self.wrapped_observer: Observer = wrapped_observer
        self.__function_runner = thread_function_runner
        self.__lock: Lock = BooleanLock()
        self.__cache_size: int | None = cache_size
        self.__message_cache: list = []
        self.__error_cache: list = []
        self.__logger = Logger()

    @counter.processed_event
    @counter.time
    def on_next(self, message):
        if self.__lock.is_locked():
            if not self.__update_cache(self.__message_cache, message):
                self.__logger.warning("value not added, buffer full")
        else:
            self.__lock.lock()
            self.__function_runner(self, self.__on_next, message)

    @staticmethod
    def __on_next(self, message: any):
        self.wrapped_observer.on_next(message)
        if len(self.__message_cache) > 0:
            self.__function_runner(self, self.__on_next, self.__message_cache.pop(0))
        else:
            self.__lock.unlock()

    def on_error(self, error: any):
        if self.__lock.is_locked():
            if not self.__update_cache(self.__error_cache, error):
                self.__logger.warning("value not added, buffer full")
        else:
            self.__lock.lock()
            self.__function_runner(self, self.__on_error, error)

    @staticmethod
    def __on_error(self, error: any):
        self.wrapped_observer.on_error(error)
        if len(self.__error_cache) > 0:
            self.__function_runner(self, self.__on_error, self.__error_cache.pop(0))
        else:
            self.__lock.unlock()

    @counter.dropped_event
    def __update_cache(self, cache: list, item: Any) -> bool:
        if self.__cache_size is None or len(cache) < self.__cache_size:
            cache.append(item)
            return True
        return False

    def on_completed(self):
        self.wrapped_observer.on_completed()

    def is_locked(self):
        return self.__lock.is_locked()


def wrap_observer_with_sized_buffer_strategy(observer: Observer, cache_size: int = 50) -> Observer:
    return SizedBufferBackPressureStrategy(observer, cache_size=cache_size)
