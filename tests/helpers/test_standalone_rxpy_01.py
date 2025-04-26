import tests.test_header
import os

# -----

import reactivex
from reactivex import operators as ops
from reactivex.scheduler import ThreadPoolScheduler
import multiprocessing
from threading import Event

which_test = 2
if which_test == 1:
    """
    Test 1: Periodic Emission Test

    This test creates a ThreadPoolScheduler that leverages as many threads as there are CPU 
    cores available, optimizing the execution across multiple threads. The core functionality
    revolves around an observable, secondly_emission, which emits a value every second. 
    Each emission is an incrementing integer, which is then mapped to a message indicating 
    the number of seconds since the test began. The sequence is limited to 30 emissions, 
    each logged as it occurs, and accompanied by an additional message via the 
    emission_process function to indicate the value's emission. The test subscribes to the 
    observable to print each emitted value, handle any potential errors, and confirm 
    completion of the emissions after 30 seconds.

    Key Components:
        •	ThreadPoolScheduler: Manages concurrency with multiple threads.
        •	Observable Sequence: Emits every second, indicating progression with a specific 
            message format.
        •	Subscription: Monitors and logs emissions, errors, and the completion event.
    """

    # Create a scheduler that uses as many threads as there are CPUs available
    optimal_thread_count = multiprocessing.cpu_count()
    pool_scheduler = ThreadPoolScheduler(optimal_thread_count)

    def emission_process(value):
        print(f"Emitting: {value}")

    # Create an observable that emits every second
    secondly_emission = reactivex.interval(1.0, scheduler=pool_scheduler).pipe(
        ops.map(lambda x: f"Value {x} emitted after {x+1} second(s)"),
        ops.do_action(emission_process),
        ops.take(30),  # Limit the emission to 30 times
    )

    # Subscribe to the observable to start emitting
    secondly_emission.subscribe(
        on_next=lambda x: print(x),
        on_error=lambda e: print(e),
        on_completed=lambda: print("Emission completed."),
        scheduler=pool_scheduler
    )

elif which_test == 2:
    """
    Test 2: Combined Emission Test

    In this test, a similar ThreadPoolScheduler setup is used to handle tasks across multiple
    CPU cores efficiently. This setup includes two observables. The first, secondly_emission,
    emits an incrementing integer every second, indicating the passage of time. The second 
    observable, immediate_emission, emits a predefined sequence of characters (['a', 'b', 
    'c', 'd', 'e']) repeatedly and immediately. These two streams are combined using the zip 
    operator, which synchronizes their emissions into pairs. Each combined pair is formatted 
    and logged, indicating both the time elapsed and the immediate value emitted at that 
    second.

    A synchronization mechanism via an Event (completed_event) ensures that the main program 
    thread waits until all planned emissions are completed before exiting. This test not only
    checks the functionality of zipping different rhythmic emissions but also demonstrates
    handling of asynchronous task completion in Python using event-driven programming.

    Key Components:
        •	Combined Observable Emissions: Synchronizes periodic and immediate emissions into
            a single stream.
        •	Event Synchronization: Uses a threading event to manage program lifecycle and 
            ensure that all emissions are processed before shutdown.
        •	Complex Subscription Management: Handles errors and completion, including 
            setting an event to signal the end of task processing.
    """

    # Create a scheduler with optimal threads
    optimal_thread_count = multiprocessing.cpu_count()
    pool_scheduler = ThreadPoolScheduler(optimal_thread_count)

    # Define an event to wait for the observable to complete
    completed_event = Event()

    def emission_process(value):
        print(f"Emitting: {value}")

    # Observable that emits every second
    secondly_emission = reactivex.interval(1.0, scheduler=pool_scheduler).pipe(
        ops.map(lambda x: f"Second {x+1}"),
        ops.take(30)
    )

    # Observable that emits values immediately and repeatedly
    immediate_emission = reactivex.from_(['a', 'b', 'c', 'd', 'e']).pipe(
        ops.repeat()
    )

    # Combine emissions using zip
    combined_emissions = reactivex.zip(secondly_emission, immediate_emission).pipe(
        ops.map(lambda combined: f"{combined[0]} - Value: {combined[1]}"),
        ops.do_action(lambda s: print(f"Combined emission: {s}"))
    )

    # Subscribe to the combined emissions
    combined_emissions.subscribe(
        on_next=lambda x: print(x),
        on_error=lambda e: print(f"Error: {e}"),
        on_completed=lambda: {
            print("Combined emission completed."),
            completed_event.set()  # Set the event to signal completion
        },
        scheduler=pool_scheduler
    )

    # Wait for the observable to complete
    completed_event.wait()