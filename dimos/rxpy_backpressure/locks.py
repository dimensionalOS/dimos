# Copyright (c) rxpy_backpressure
from abc import abstractmethod


class Lock:
    @abstractmethod
    def is_locked(self) -> bool:
        return NotImplemented

    @abstractmethod
    def unlock(self):
        return NotImplemented

    @abstractmethod
    def lock(self):
        return NotImplemented


class BooleanLock(Lock):
    def __init__(self):
        self.locked: bool = False

    def is_locked(self) -> bool:
        return self.locked

    def unlock(self):
        self.locked = False

    def lock(self):
        self.locked = True
