import pytest
import time
import numpy as np
import reactivex as rx
from reactivex import operators as ops
from typing import Callable, TypeVar, Any
from dimos.utils.reactive import backpressure, getter_streaming, getter_ondemand
from reactivex.disposable import Disposable


def measure_time(func: Callable[[], Any], iterations: int = 1) -> float:
    start_time = time.time()
    result = func()
    end_time = time.time()
    total_time = end_time - start_time
    return result, total_time

def assert_time(func: Callable[[], Any], assertion: Callable[[int], bool], assert_fail_msg=None) -> None:
    [result, total_time ] = measure_time(func)
    assert assertion(total_time), assert_fail_msg + f", took {round(total_time, 2)}s"
    return result

def min_time(func: Callable[[], Any], min_t: int, assert_fail_msg="Function returned too fast"):
    return assert_time(func, (lambda t: t > min_t), assert_fail_msg + f", min: {min_t} seconds")

def max_time(func: Callable[[], Any], max_t: int, assert_fail_msg="Function took too long"):
    return assert_time(func, (lambda t: t < max_t), assert_fail_msg + f", max: {max_t} seconds")

T = TypeVar('T')

def dispose_spy(source: rx.Observable[T]) -> rx.Observable[T]:
    state = {"active": 0}

    def factory(observer, scheduler=None):
        state["active"] += 1
        upstream = source.subscribe(observer, scheduler=scheduler)
        def _dispose():
            upstream.dispose()
            state["active"] -= 1
        return Disposable(_dispose)

    proxy = rx.create(factory)
    proxy.subs_number = lambda: state["active"]
    proxy.is_disposed = lambda: state["active"] == 0
    return proxy




def test_backpressure_handling():
    received_fast = []
    received_slow = []
    # Create an observable that emits numpy arrays instead of integers
    source = dispose_spy(rx.interval(0.1).pipe(
        ops.map(lambda i: np.array([i, i+1, i+2])),
        ops.take(50)
    ))

    # Wrap with backpressure handling
    safe_source = backpressure(source)

    # Fast sub
    subscription1 = safe_source.subscribe(lambda x: received_fast.append(x))

    # Slow sub (shouldn't block above)
    subscription2 = safe_source.subscribe(lambda x: (time.sleep(0.25), received_slow.append(x)))
    
    time.sleep(2.5)
    
    subscription1.dispose()
    assert not source.is_disposed(), "Observable should not be disposed yet"
    subscription2.dispose()
    time.sleep(0.1)
    assert source.is_disposed(), "Observable should be disposed"

    # Check results
    print("Fast observer received:", len(received_fast), [arr[0] for arr in received_fast])
    print("Slow observer received:", len(received_slow), [arr[0] for arr in received_slow])
    
    # Fast observer should get all or nearly all items
    assert len(received_fast) > 15, f"Expected fast observer to receive most items, got {len(received_fast)}"
    
    # Slow observer should get fewer items due to backpressure handling
    assert len(received_slow) < len(received_fast), "Slow observer should receive fewer items than fast observer"
    # Specifically, processing at 0.25s means ~4 items per second, so expect 8-10 items
    assert 7 <= len(received_slow) <= 11, f"Expected 7-11 items, got {len(received_slow)}"
    
    # The slow observer should skip items (not process them in sequence)
    # We test this by checking that the difference between consecutive arrays is sometimes > 1
    has_skips = False
    for i in range(1, len(received_slow)):
        if received_slow[i][0] - received_slow[i-1][0] > 1:
            has_skips = True
            break
    assert has_skips, "Slow observer should skip items due to backpressure"


def test_getter_streaming_blocking():
    source = dispose_spy(rx.interval(0.2).pipe(
        ops.map(lambda i: np.array([i, i+1, i+2])),
        ops.take(50)
    ))
    assert source.is_disposed()

    getter = min_time(lambda: getter_streaming(source), 0.2, "Latest getter needs to block until first msg is ready")
    assert np.array_equal(getter(), np.array([0, 1, 2])), f"Expected to get the first array [0,1,2], got {getter()}"

    time.sleep(0.5)
    assert getter()[0] >= 2, f"Expected array with first value >= 2, got {getter()}"
    time.sleep(0.5)
    assert getter()[0] >= 4, f"Expected array with first value >= 4, got {getter()}"

    getter.dispose()
    assert source.is_disposed(), "Observable should be disposed"

def test_getter_streaming_blocking_timeout():
    source = dispose_spy(rx.interval(0.2).pipe(ops.take(50)))
    with pytest.raises(Exception):
        getter = getter_streaming(source, timeout=0.1)
        getter.dispose()
    assert source.is_disposed()

def test_getter_streaming_nonblocking():
    source = dispose_spy(rx.interval(0.2).pipe(ops.take(50)))

    getter = max_time(lambda: getter_streaming(source, nonblocking=True), 0.1, "nonblocking getter init shouldn't block")
    min_time(getter, 0.2, "Expected for first value call to block if cache is empty")
    assert getter() == 0

    time.sleep(0.5)
    assert getter() >= 2, f"Expected value >= 2, got {getter()}"

    # sub is active
    assert not source.is_disposed()

    time.sleep(0.5)
    assert getter() >= 4, f"Expected value >= 4, got {getter()}"


    getter.dispose()
    assert source.is_disposed(), "Observable should be disposed"

def test_getter_streaming_nonblocking_timeout():
    source = dispose_spy(rx.interval(0.2).pipe(ops.take(50)))
    getter = getter_streaming(source, timeout=0.1, nonblocking=True)
    with pytest.raises(Exception):
        getter()

    assert not source.is_disposed(), "is not disposed, this is a job of the caller"

def test_getter_ondemand():
    source = dispose_spy(rx.interval(0.1).pipe(ops.take(50)))
    getter = getter_ondemand(source)
    assert source.is_disposed(), "Observable should be disposed"
    assert min_time(getter, 0.05) == 0, f"Expected to get the first value of 0, got {getter()}"
    assert source.is_disposed(), "Observable should be disposed"
    assert getter() == 0, f"Expected to get the first value of 0, got {getter()}"
    assert source.is_disposed(), "Observable should be disposed"

def test_getter_ondemand_timeout():
    source = dispose_spy(rx.interval(0.2).pipe(ops.take(50)))
    getter = getter_ondemand(source, timeout=0.1)
    with pytest.raises(Exception):
        getter()
    assert source.is_disposed(), "Observable should be disposed"
