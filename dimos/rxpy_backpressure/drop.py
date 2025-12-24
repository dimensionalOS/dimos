# Copyright (c) rxpy_backpressure
from typing import Any

from dimos.rxpy_backpressure.function_runner import thread_function_runner
from dimos.rxpy_backpressure.locks import BooleanLock, Lock
from dimos.rxpy_backpressure.observer import Observer


class DropBackPressureStrategy(Observer):
    def __init__(self, wrapped_observer: Observer, cache_size: int):
        self.wrapped_observer: Observer = wrapped_observer
        self.__function_runner = thread_function_runner
        self.__lock: Lock = BooleanLock()
        self.__cache_size: int | None = cache_size
        self.__message_cache: list = []
        self.__error_cache: list = []

    def on_next(self, message):
        if self.__lock.is_locked():
            self.__update_cache(self.__message_cache, message)
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
            self.__update_cache(self.__error_cache, error)
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

    def __update_cache(self, cache: list, item: Any):
        if self.__cache_size is None or len(cache) < self.__cache_size:
            cache.append(item)
        else:
            cache.pop(0)
            cache.append(item)

    def on_completed(self):
        self.wrapped_observer.on_completed()

    def is_locked(self):
        return self.__lock.is_locked()


def wrap_observer_with_drop_strategy(observer: Observer, cache_size: int = 10) -> Observer:
    return DropBackPressureStrategy(observer, cache_size=cache_size)


def wrap_observer_with_buffer_strategy(observer: Observer) -> Observer:
    return DropBackPressureStrategy(observer, cache_size=None)
