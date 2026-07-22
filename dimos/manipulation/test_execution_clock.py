"""Tests for the serialized, validated execution clock."""

import threading

import pytest

from dimos.manipulation.execution_clock import InvalidMonotonicClock, ValidatedMonotonicClock


class GatedSource:
    def __init__(self, initial: float) -> None:
        self.first_entry = threading.Event()
        self.second_entry = threading.Event()
        self.first_release = threading.Event()
        self.second_release = threading.Event()
        self._lock = threading.Lock()
        self.calls = 0
        self.value = initial

    def __call__(self) -> float:
        with self._lock:
            self.calls += 1
            call = self.calls
        if call == 2:
            self.first_entry.set()
            if not self.first_release.wait(1):
                raise AssertionError("first clock sample was not released")
        elif call == 3:
            self.second_entry.set()
            if not self.second_release.wait(1):
                raise AssertionError("second clock sample was not released")
        return self.value


def test_now_serializes_source_sampling_before_validation() -> None:
    source = GatedSource(0.0)
    clock = ValidatedMonotonicClock(source)
    first_done = threading.Event()
    second_done = threading.Event()

    def first() -> None:
        clock.now()
        first_done.set()

    def second() -> None:
        clock.now()
        second_done.set()

    one = threading.Thread(target=first)
    two = threading.Thread(target=second)
    one.start()
    assert source.first_entry.wait(1)
    two.start()
    assert not source.second_entry.wait(0.02)
    source.first_release.set()
    assert first_done.wait(1)
    assert source.second_entry.wait(1)
    source.second_release.set()
    assert second_done.wait(1)
    one.join(1)
    two.join(1)


@pytest.mark.parametrize("sample", [float("nan"), float("inf"), float("-inf")])
def test_clock_rejects_nonfinite_initial_sample(sample: float) -> None:
    with pytest.raises(InvalidMonotonicClock):
        ValidatedMonotonicClock(lambda: sample)


@pytest.mark.parametrize("sample", [float("nan"), float("inf"), float("-inf")])
def test_clock_rejects_nonfinite_later_sample(sample: float) -> None:
    values = iter((0.0, sample))
    clock = ValidatedMonotonicClock(lambda: next(values))
    with pytest.raises(InvalidMonotonicClock):
        clock.now()


def test_clock_rejects_decreasing_sample() -> None:
    values = iter((1.0, 0.5))
    clock = ValidatedMonotonicClock(lambda: next(values))
    with pytest.raises(InvalidMonotonicClock):
        clock.now()
