# Advanced Stream Handling

> **Prerequisite:** Read [ReactiveX Fundamentals](reactivex.md) first for Observable basics.

## Backpressure and Parallel Subscribers to Hardware

In robotics, we deal with hardware that produces data at its own pace - a camera outputs 30fps whether you're ready or not. We can't tell the camera to slow down. And we often have multiple consumers: one module wants every frame for recording, another runs slow ML inference and only needs the latest frame.

**The problem:** A fast producer can overwhelm a slow consumer, causing memory buildup or dropped frames. We might have multiple subscribers to the same hardware that operate at different speeds.


<details><summary>Pikchr</summary>

```pikchr fold output=assets/backpressure.svg
color = white
fill = none

Fast: box "Camera" "60 fps" rad 5px fit wid 130% ht 130%
arrow right 0.4in
Queue: box "queue" rad 5px fit wid 170% ht 170%
arrow right 0.4in
Slow: box "ML Model" "2 fps" rad 5px fit wid 130% ht 130%

text "items pile up!" at (Queue.x, Queue.y - 0.45in)
```

</details>

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

# We need this scaffolding here. Normally DimOS handles this.
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


<details><summary>Pikchr</summary>

```pikchr fold output=assets/backpressure_solution.svg
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

</details>

<!--Result:-->
![output](assets/backpressure_solution.svg)

The `LATEST` strategy means: when the slow subscriber finishes processing, it gets whatever the most recent value is, skipping any values that arrived while it was busy.

### Usage in modules

Most module streams offer backpressured observables.

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

Sometimes you don't want a stream, you just want to call a function and get the latest value. We provide two approaches:

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

get_val = getter_hot(source, timeout=5.0) # blocks until first message, with 5s timeout
# alternatively not to block (but get_val() might return None)
# get_val = getter_hot(source, nonblocking=True)

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
