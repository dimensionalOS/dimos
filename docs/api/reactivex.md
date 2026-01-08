# ReactiveX (RxPY) Quick Reference

RxPY provides composable asynchronous data streams. This is a practical guide focused on common patterns in this codebase.

## Quick Start: Using an Observable

Given a function that returns an `Observable`, here's how to use it:

```python session=rx
import reactivex as rx
from reactivex import operators as ops

# Create an observable that emits 0,1,2,3,4
source = rx.of(0, 1, 2, 3, 4)

# Subscribe and print each value
received = []
source.subscribe(lambda x: received.append(x))
print("received:", received)
```

<!--Result:-->
```
received: [0, 1, 2, 3, 4]
```

## The `.pipe()` Pattern

Chain operators using `.pipe()`:

```python session=rx
# Transform values: multiply by 2, then filter > 4
result = []

# we build another observable, it's passive until subscribe is called
observable = source.pipe(
    ops.map(lambda x: x * 2),
    ops.filter(lambda x: x > 4),
)

observable.subscribe(lambda x: result.append(x))

print("transformed:", result)
```

<!--Result:-->
```
transformed: [6, 8]
```

## Common Operators

### Transform: `map`

```python session=rx
rx.of(1, 2, 3).pipe(
    ops.map(lambda x: f"item_{x}")
).subscribe(print)
```

<!--Result:-->
```
item_1
item_2
item_3
<reactivex.disposable.disposable.Disposable object at 0x7f3c2c9448c0>
```

### Filter: `filter`

```python session=rx
rx.of(1, 2, 3, 4, 5).pipe(
    ops.filter(lambda x: x % 2 == 0)
).subscribe(print)
```

<!--Result:-->
```
2
4
<reactivex.disposable.disposable.Disposable object at 0x7f3c2c9447d0>
```

### Limit emissions: `take`

```python session=rx
rx.of(1, 2, 3, 4, 5).pipe(
    ops.take(3)
).subscribe(print)
```

<!--Result:-->
```
1
2
3
<reactivex.disposable.disposable.Disposable object at 0x7f3c2c944a10>
```

### Flatten nested observables: `flat_map`

```python session=rx
# For each input, emit multiple values
rx.of(1, 2).pipe(
    ops.flat_map(lambda x: rx.of(x, x * 10, x * 100))
).subscribe(print)
```

<!--Result:-->
```
1
10
100
2
20
200
<reactivex.disposable.disposable.Disposable object at 0x7f3c2c945700>
```

## Rate Limiting

### `sample(interval)` - Emit latest value every N seconds

Takes the most recent value at each interval. Good for continuous streams where you want the freshest data.

```python session=rx
# Use blocking .run() to collect results properly
results = rx.interval(0.05).pipe(
    ops.take(10),
    ops.sample(0.2),
    ops.to_list(),
).run()
print("sample() got:", results)
```

<!--Result:-->
```
sample() got: [2, 6, 9]
```

### `throttle_first(interval)` - Emit first, then block for N seconds

Takes the first value then ignores subsequent values for the interval. Good for user input debouncing.

```python session=rx
results = rx.interval(0.05).pipe(
    ops.take(10),
    ops.throttle_first(0.15),
    ops.to_list(),
).run()
print("throttle_first() got:", results)
```

<!--Result:-->
```
throttle_first() got: [0, 3, 6, 9]
```

### Difference between sample and throttle_first

```python session=rx
# sample: takes LATEST value at each interval tick
# throttle_first: takes FIRST value then blocks

# With fast emissions (0,1,2,3,4,5,6,7,8,9) every 50ms:
# sample(0.2s)        -> gets value at 200ms, 400ms marks -> [2, 6, 9]
# throttle_first(0.15s) -> gets 0, blocks, then 3, blocks, then 6... -> [0,3,6,9]
print("sample: latest value at each tick")
print("throttle_first: first value, then block")
```

<!--Result:-->
```
sample: latest value at each tick
throttle_first: first value, then block
```


## What is an Observable?

An Observable is like a list, but instead of holding all values at once, it produces values over time.

|             | List                  | Iterator              | Observable       |
|-------------|-----------------------|-----------------------|------------------|
| **Values**  | All exist now         | Generated on demand   | Arrive over time |
| **Control** | You pull (`for x in`) | You pull (`next()`)   | Pushed to you    |
| **Size**    | Finite                | Can be infinite       | Can be infinite  |
| **Async**   | No                    | Yes (with asyncio)    | Yes              |
| **Cancel**  | N/A                   | Stop calling `next()` | `.dispose()`     |

The key difference from iterators: with an Observable, **you don't control when values arrive**. A camera produces frames at 30fps whether you're ready or not. An iterator waits for you to call `next()`.

**Observables are lazy.** An Observable is just a description of work to be done - it sits there doing nothing until you call `.subscribe()`. That's when it "wakes up" and starts producing values.

This means you can build complex pipelines, pass them around, and nothing happens until someone subscribes.

**The three things an Observable can tell you:**

1. **"Here's a value"** (`on_next`) - A new value arrived
2. **"Something went wrong"** (`on_error`) - An error occurred, stream stops
3. **"I'm done"** (`on_completed`) - No more values coming

**The basic pattern:**

```
observable.subscribe(what_to_do_with_each_value)
```

That's it. You create or receive an Observable, then subscribe to start receiving values.

When you subscribe, data flows through a pipeline:

<details>
<summary>diagram source</summary>

```pikchr output=assets/observable_flow.svg
color = white
fill = none

Obs: box "observable" rad 5px fit wid 170% ht 170%
arrow right 0.3in
Pipe: box ".pipe(ops)" rad 5px fit wid 170% ht 170%
arrow right 0.3in
Sub: box ".subscribe()" rad 5px fit wid 170% ht 170%
arrow right 0.3in
Handler: box "callback" rad 5px fit wid 170% ht 170%
```

<!--Result:-->
![output](assets/observable_flow.svg)

**Key property: Observables are lazy.** Nothing happens until you call `.subscribe()`. This means you can build up complex pipelines without any work being done, then start the flow when ready.

Here's the full subscribe signature with all three callbacks:

```python session=rx
rx.of(1, 2, 3).subscribe(
    on_next=lambda x: print(f"value: {x}"),
    on_error=lambda e: print(f"error: {e}"),
    on_completed=lambda: print("done")
)
```

<!--Result:-->
```
value: 1
value: 2
value: 3
done
<reactivex.disposable.disposable.Disposable object at 0x7f3c2c947680>
```

## Disposables: Cancelling Subscriptions

When you subscribe, you get back a `Disposable`. This is your "cancel button":

```python session=rx
import reactivex as rx

source = rx.interval(0.1)  # emits 0, 1, 2, ... every 100ms forever
subscription = source.subscribe(lambda x: print(x))

# Later, when you're done:
subscription.dispose()  # Stop receiving values, clean up resources
print("disposed")
```

<!--Result:-->
```
disposed
```

**Why does this matter?**

- Observables can be infinite (sensor feeds, websockets, timers)
- Without disposing, you leak memory and keep processing values forever
- Disposing also cleans up any resources the Observable opened (connections, file handles, etc.)

**Rule of thumb:** If you subscribe to something that doesn't naturally complete, save the disposable and call `.dispose()` when done.

**In dimos modules:** Every `Module` has a `self._disposables` (a `CompositeDisposable`) that automatically disposes everything when the module closes:

```python session=rx
import time
from dimos.core import Module

class MyModule(Module):
    def start(self):
        source = rx.interval(0.05)
        self._disposables.add(source.subscribe(lambda x: print(f"got {x}")))

module = MyModule()
module.start()
time.sleep(0.25)

# unsubscribes disposables
module.stop()
```

<!--Result:-->
```
got 0
got 1
got 2
got 3
```

## Backpressure and parallel subscribers to hardware

In robotics, we deal with hardware that produces data at its own pace - a camera outputs 30fps whether you're ready or not. We can't tell the camera to slow down. And we often have multiple consumers: one module wants every frame for recording, another runs slow ML inference and only needs the latest frame.

**The problem:** A fast producer can overwhelm a slow consumer, causing memory buildup or dropped frames. We might have multiple subscribers to the same hardware that operate at different speeds.

<details>
<summary>diagram source</summary>

```pikchr output=assets/backpressure.svg
color = white
fill = none

Fast: box "Camera" "60 fps" rad 5px fit wid 130% ht 130%
arrow right 0.4in
Queue: box "queue" rad 5px fit wid 170% ht 170%
arrow right 0.4in
Slow: box "ML Model" "2 fps" rad 5px fit wid 130% ht 130%

text "items pile up!" at (Queue.x, Queue.y - 0.45in)
```

<!--Result:-->
![output](assets/backpressure.svg)

**The solution:** The `backpressure()` wrapper handles this by:

1. **Sharing the source** - Camera runs once, all subscribers share the stream
2. **Per-subscriber speed** - Fast subscribers get every frame, slow ones get the latest when ready
3. **No blocking** - Slow subscribers never block the source or each other

```python session=bp
import time
import reactivex as rx
from reactivex import operators as ops
from reactivex.scheduler import ThreadPoolScheduler
from dimos.utils.reactive import backpressure

# we need this scaffolding here, normally dimos handles this
scheduler = ThreadPoolScheduler(max_workers=4)

# Simulate fast source
source = rx.interval(0.05).pipe(ops.take(20))
safe = backpressure(source, scheduler=scheduler)

fast_results = []
slow_results = []

safe.subscribe(lambda x: fast_results.append(x))

def slow_handler(x):
    time.sleep(0.15)
    slow_results.append(x)

safe.subscribe(slow_handler)

time.sleep(1.5)
print(f"fast got {len(fast_results)} items: {fast_results[:5]}...")
print(f"slow got {len(slow_results)} items (skipped {len(fast_results) - len(slow_results)})")
scheduler.executor.shutdown(wait=True)
```

<!--Result:-->
```
fast got 20 items: [0, 1, 2, 3, 4]...
slow got 7 items (skipped 13)
```

### How it works

<details>
<summary>diagram source</summary>

```pikchr output=assets/backpressure_solution.svg
color = white
fill = none
linewid = 0.3in

Source: box "Camera" "60 fps" rad 5px fit wid 170% ht 170%
arrow
Core: box "backpressure" rad 5px fit wid 170% ht 170%
arrow from Core.e right 0.3in then up 0.35in then right 0.3in
Fast: box "Fast Sub" rad 5px fit wid 170% ht 170%
arrow from Core.e right 0.3in then down 0.35in then right 0.3in
SlowPre: box "LATEST" rad 5px fit wid 170% ht 170%
arrow
Slow: box "Slow Sub" rad 5px fit wid 170% ht 170%
```

<!--Result:-->
![output](assets/backpressure_solution.svg)

The `LATEST` strategy means: when the slow subscriber finishes processing, it gets whatever the most recent value is, skipping any values that arrived while it was busy.

### Usage in modules

Most module streams offer backpressured observables

```python session=bp
from dimos.core import Module, In
from dimos.msgs.sensor_msgs import Image

class MLModel(Module):
    color_image: In[Image]
    def start(self):
       # no reactivex, simple callback
       self.color_image.subscribe(...)
       # backpressured
       self.color_image.observable().subscribe(...)
       # non-backpressured - will pile up queue
       self.color_image.pure_observable().subscribe(...)


```


## Getting Values Synchronously

Sometimes you don't want a stream - you just want to call a function and get the latest value. We provide two approaches:

|                  | `getter_hot()`                 | `getter_cold()`                  |
|------------------|--------------------------------|----------------------------------|
| **Subscription** | Stays active in background     | Fresh subscription each call     |
| **Read speed**   | Instant (value already cached) | Slower (waits for value)         |
| **Resources**    | Keeps connection open          | Opens/closes each call           |
| **Use when**     | Frequent reads, need latest    | Occasional reads, save resources |

**Prefer `getter_cold()`** when you can afford to wait and warmup isn't expensive. It's simpler (no cleanup needed) and doesn't hold resources. Only use `getter_hot()` when you need instant reads or the source is expensive to start.

### `getter_hot()` - Background subscription, instant reads

Subscribes immediately and keeps updating in the background. Each call returns the cached latest value instantly.

```python session=sync
import time
import reactivex as rx
from reactivex import operators as ops
from dimos.utils.reactive import getter_hot

source = rx.interval(0.1).pipe(ops.take(10))
get_val = getter_hot(source, timeout=5.0)

print("first call:", get_val())  # instant - value already there
time.sleep(0.35)
print("after 350ms:", get_val())  # instant - returns cached latest
time.sleep(0.35)
print("after 700ms:", get_val())

get_val.dispose()  # Don't forget to clean up!
```

<!--Result:-->
```
first call: 0
after 350ms: 3
after 700ms: 6
```

### `getter_cold()` - Fresh subscription each call

Each call creates a new subscription, waits for one value, and cleans up. Slower but doesn't hold resources:

```python session=sync
from dimos.utils.reactive import getter_cold

source = rx.of(0, 1, 2, 3, 4)
get_val = getter_cold(source, timeout=5.0)

# Each call creates fresh subscription, gets first value
print("call 1:", get_val())  # subscribes, gets 0, disposes
print("call 2:", get_val())  # subscribes again, gets 0, disposes
print("call 3:", get_val())  # subscribes again, gets 0, disposes
```

<!--Result:-->
```
call 1: 0
call 2: 0
call 3: 0
```


## Creating Observables

### From callback-based APIs

```python session=create
import reactivex as rx
from reactivex import operators as ops
from dimos.utils.reactive import callback_to_observable

class MockSensor:
    def __init__(self):
        self._callbacks = []
    def register(self, cb):
        self._callbacks.append(cb)
    def unregister(self, cb):
        self._callbacks.remove(cb)
    def emit(self, value):
        for cb in self._callbacks:
            cb(value)

sensor = MockSensor()

obs = callback_to_observable(
    start=sensor.register,
    stop=sensor.unregister
)

received = []
sub = obs.subscribe(lambda x: received.append(x))

sensor.emit("reading_1")
sensor.emit("reading_2")
print("received:", received)

sub.dispose()
print("callbacks after dispose:", len(sensor._callbacks))
```

<!--Result:-->
```
received: ['reading_1', 'reading_2']
callbacks after dispose: 0
```

### From scratch with `rx.create`

```python session=create
from reactivex.disposable import Disposable

def custom_subscribe(observer, scheduler=None):
    observer.on_next("first")
    observer.on_next("second")
    observer.on_completed()
    return Disposable(lambda: print("cleaned up"))

obs = rx.create(custom_subscribe)

results = []
obs.subscribe(
    on_next=lambda x: results.append(x),
    on_completed=lambda: results.append("DONE")
)
print("results:", results)
```

<!--Result:-->
```
cleaned up
results: ['first', 'second', 'DONE']
```

## CompositeDIsposable

As we know we can always dispose subscriptions when done to prevent leaks:

```python session=dispose
import time
import reactivex as rx
from reactivex import operators as ops

source = rx.interval(0.1).pipe(ops.take(100))
received = []

subscription = source.subscribe(lambda x: received.append(x))
time.sleep(0.25)
subscription.dispose()
time.sleep(0.2)

print(f"received {len(received)} items before dispose")
```

<!--Result:-->
```
received 2 items before dispose
```

For multiple subscriptions, use `CompositeDisposable`:

```python session=dispose
from reactivex.disposable import CompositeDisposable

disposables = CompositeDisposable()

s1 = rx.of(1,2,3).subscribe(lambda x: None)
s2 = rx.of(4,5,6).subscribe(lambda x: None)

disposables.add(s1)
disposables.add(s2)

print("subscriptions:", len(disposables))
disposables.dispose()
print("after dispose:", disposables.is_disposed)
```

<!--Result:-->
```
subscriptions: 2
after dispose: True
```

## Reference

| Operator              | Purpose                                   | Example                               |
|-----------------------|-------------------------------------------|---------------------------------------|
| `map(fn)`             | Transform each value                      | `ops.map(lambda x: x * 2)`            |
| `filter(pred)`        | Keep values matching predicate            | `ops.filter(lambda x: x > 0)`         |
| `take(n)`             | Take first n values                       | `ops.take(10)`                        |
| `first()`             | Take first value only                     | `ops.first()`                         |
| `sample(sec)`         | Emit latest every interval                | `ops.sample(0.5)`                     |
| `throttle_first(sec)` | Emit first, block for interval            | `ops.throttle_first(0.5)`             |
| `flat_map(fn)`        | Map + flatten nested observables          | `ops.flat_map(lambda x: rx.of(x, x))` |
| `observe_on(sched)`   | Switch scheduler                          | `ops.observe_on(pool_scheduler)`      |
| `replay(n)`           | Cache last n values for late subscribers  | `ops.replay(buffer_size=1)`           |
| `ref_count()`         | Auto-connect/disconnect shared observable | `ops.ref_count()`                     |
| `share()`             | Shorthand for `publish().ref_count()`     | `ops.share()`                         |
| `timeout(sec)`        | Error if no value within timeout          | `ops.timeout(5.0)`                    |

See [RxPY documentation](https://rxpy.readthedocs.io/) for complete operator reference.
