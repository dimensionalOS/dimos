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

from typing import Optional

from rxpy_backpressure.function_runner import thread_function_runner
from rxpy_backpressure.locks import BooleanLock, Lock
from rxpy_backpressure.observer import Observer


class LatestBackPressureStrategy(Observer):
    def __init__(self, wrapped_observer: Observer):
        self.wrapped_observer: Observer = wrapped_observer
        self.__function_runner = thread_function_runner
        self.__lock: Lock = BooleanLock()
        self.__message_cache: Optional = None
        self.__error_cache: Optional = None

    def on_next(self, message):
        if self.__lock.is_locked():
            self.__message_cache = message
        else:
            self.__lock.lock()
            self.__function_runner(self, self.__on_next, message)

    @staticmethod
    def __on_next(self, message: any):
        self.wrapped_observer.on_next(message)
        if self.__message_cache is not None:
            self.__function_runner(self, self.__on_next, self.__message_cache)
            self.__message_cache = None
        else:
            self.__lock.unlock()

    def on_error(self, error: any):
        if self.__lock.is_locked():
            self.__error_cache = error
        else:
            self.__lock.lock()
            self.__function_runner(self, self.__on_error, error)

    @staticmethod
    def __on_error(self, error: any):
        self.wrapped_observer.on_error(error)
        if self.__error_cache:
            self.__function_runner(self, self.__on_error, self.__error_cache)
            self.__error_cache = None
        else:
            self.__lock.unlock()

    def on_completed(self):
        self.wrapped_observer.on_completed()

    def is_locked(self):
        return self.__lock.is_locked()


def wrap_observer_with_latest_strategy(observer: Observer) -> Observer:
    return LatestBackPressureStrategy(observer)
