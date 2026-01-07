# Copyright (c) rxpy_backpressure
from threading import Thread


def thread_function_runner(self, func, message):
    Thread(target=func, args=(self, message)).start()
