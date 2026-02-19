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

Sometimes you don't want a stream, you just want to call a function and get the latest value.

If you are doing this periodically as a part of a processing loop, it is very likely that your code will be much cleaner and safer using actual reactivex pipeline. So bias towards checking our [reactivex quick guide](reactivex.md) and [official docs](https://rxpy.readthedocs.io/)

(TODO we should actually make this example actually executable)

```python skip
    self.color_image.observable().pipe(
        # takes the best image from a stream every 200ms,
        # ensuring we are feeding our detector with highest quality frames
        quality_barrier(lambda x: x["quality"], target_frequency=0.2),

        # converts Image into Person detections
        ops.map(detect_person),

        # converts Detection2D to Twist pointing in the direction of a detection
        ops.map(detection2d_to_twist),

        # emits the latest value every 50ms making our control loop run at 20hz
        # despite detections running at 200ms
        ops.sample(0.05),
    ).subscribe(self.twist.publish) # shoots off the Twist out of the module
```


If you'd still like to switch to synchronous fetching, we provide two approaches, `getter_hot()` and `getter_cold()`

|                  | `getter_hot()`                 | `getter_cold()`                  |
|------------------|--------------------------------|----------------------------------|
| **Subscription** | Stays active in background     | Fresh subscription each call     |
| **Read speed**   | Instant (value already cached) | Slower (waits for value)         |
| **Resources**    | Keeps connection open          | Opens/closes each call           |
| **Use when**     | Frequent reads, need latest    | Occasional reads, save resources |

<details>
<summary>diagram source</summary>

```pikchr fold output=assets/getter_hot_cold.svg
color = white
fill = none

H_Title: box "getter_hot()" rad 5px fit wid 170% ht 170%

Sub: box "subscribe" rad 5px fit wid 170% ht 170% with .n at H_Title.s + (0, -0.5in)
arrow from H_Title.s to Sub.n
arrow right from Sub.e
Cache: box "Cache" rad 5px fit wid 170% ht 170%

# blocking box around subscribe->cache (one-time setup)
Blk0: box dashed color 0x5c9ff0 with .nw at Sub.nw + (-0.1in, 0.25in) wid (Cache.e.x - Sub.w.x + 0.2in) ht 0.7in rad 5px
text "blocking" italic with .n at Blk0.n + (0, -0.05in)

arrow right from Cache.e
Getter: box "getter" rad 5px fit wid 170% ht 170%

arrow from Getter.e right 0.3in then down 0.25in then right 0.2in
G1: box invis "call()" color 0x8cbdf2 fit wid 150%
arrow right 0.4in from G1.e
box invis "instant" fit wid 150%

arrow from Getter.e right 0.3in then down 0.7in then right 0.2in
G2: box invis "call()" color 0x8cbdf2 fit wid 150%
arrow right 0.4in from G2.e
box invis "instant" fit wid 150%

text "always subscribed" italic with .n at Blk0.s + (0, -0.1in)


# === getter_cold section ===
C_Title: box "getter_cold()" rad 5px fit wid 170% ht 170% with .nw at H_Title.sw + (0, -1.6in)

arrow down 0.3in from C_Title.s
ColdGetter: box "getter" rad 5px fit wid 170% ht 170%

# Branch to first call
arrow from ColdGetter.e right 0.3in then down 0.3in then right 0.2in
Cold1: box invis "call()" color 0x8cbdf2 fit wid 150%
arrow right 0.4in from Cold1.e
Sub1: box invis "subscribe" fit wid 150%
arrow right 0.4in from Sub1.e
Wait1: box invis "wait" fit wid 150%
arrow right 0.4in from Wait1.e
Val1: box invis "value" fit wid 150%
arrow right 0.4in from Val1.e
Disp1: box invis "dispose  " fit wid 150%

# blocking box around first row
Blk1: box dashed color 0x5c9ff0 with .nw at Cold1.nw + (-0.1in, 0.25in) wid (Disp1.e.x - Cold1.w.x + 0.2in) ht 0.7in rad 5px
text "blocking" italic with .n at Blk1.n + (0, -0.05in)

# Branch to second call
arrow from ColdGetter.e right 0.3in then down 1.2in then right 0.2in
Cold2: box invis "call()" color 0x8cbdf2 fit wid 150%
arrow right 0.4in from Cold2.e
Sub2: box invis "subscribe" fit wid 150%
arrow right 0.4in from Sub2.e
Wait2: box invis "wait" fit wid 150%
arrow right 0.4in from Wait2.e
Val2: box invis "value" fit wid 150%
arrow right 0.4in from Val2.e
Disp2: box invis "dispose  " fit wid 150%

# blocking box around second row
Blk2: box dashed color 0x5c9ff0 with .nw at Cold2.nw + (-0.1in, 0.25in) wid (Disp2.e.x - Cold2.w.x + 0.2in) ht 0.7in rad 5px
text "blocking" italic with .n at Blk2.n + (0, -0.05in)
```

</details>

<!--Result:-->
![output](assets/getter_hot_cold.svg)


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
