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
<reactivex.disposable.disposable.Disposable object at 0x7fcedec40b90>
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
<reactivex.disposable.disposable.Disposable object at 0x7fcedec40c50>
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
<reactivex.disposable.disposable.Disposable object at 0x7fcedec40a40>
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
<reactivex.disposable.disposable.Disposable object at 0x7fcedec41a60>
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

```pikchr fold output=assets/observable_flow.svg
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

</details>

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
<reactivex.disposable.disposable.Disposable object at 0x7fcedec42d20>
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

**Rule of thumb:** Whenever you subscribe, save the disposable because you have to unsubscribe at some point by calling `disposable.dispose()`.

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
got 4
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

## CompositeDisposable

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

| Operator              | Purpose                                  | Example                               |
|-----------------------|------------------------------------------|---------------------------------------|
| `map(fn)`             | Transform each value                     | `ops.map(lambda x: x * 2)`            |
| `filter(pred)`        | Keep values matching predicate           | `ops.filter(lambda x: x > 0)`         |
| `take(n)`             | Take first n values                      | `ops.take(10)`                        |
| `first()`             | Take first value only                    | `ops.first()`                         |
| `sample(sec)`         | Emit latest every interval               | `ops.sample(0.5)`                     |
| `throttle_first(sec)` | Emit first, block for interval           | `ops.throttle_first(0.5)`             |
| `flat_map(fn)`        | Map + flatten nested observables         | `ops.flat_map(lambda x: rx.of(x, x))` |
| `observe_on(sched)`   | Switch scheduler                         | `ops.observe_on(pool_scheduler)`      |
| `replay(n)`           | Cache last n values for late subscribers | `ops.replay(buffer_size=1)`           |
| `timeout(sec)`        | Error if no value within timeout         | `ops.timeout(5.0)`                    |

See [RxPY documentation](https://rxpy.readthedocs.io/) for complete operator reference.
