
Example on how we can use memory to compare two algos on a real data.

```python
import time

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.transform import throttle
from dimos.memory.vis import color
from dimos.memory.vis.plot.elements import Style
from dimos.memory.vis.plot.plot import Plot
from dimos.messages.sensor_msgs.msg import Image
from dimos.message_utils import image as image_utils
from dimos.utils.data import get_data

store = SqliteStore(path=get_data("go2_bigoffice.db"))
images = store.streams.color_image


def slow_brightness(img: Image) -> float:
    """Naive full-pixel mean, for reference."""
    pixels = image_utils.view(img)
    max_val = 65535.0 if img.encoding in ("mono16", "16UC1", "16SC1") else 255.0
    return float(pixels.mean() / max_val)


def timed(fn):
    """Wrap ``fn(img) -> float`` so it returns execution time in ms instead.

    Reads ``obs.data`` first so the lazy blob load is not counted.
    """
    def _fn(obs):
        img = obs.data
        _ = img.data  # warm lazy load, this actually loads from sql
        t0 = time.perf_counter()
        fn(img)
        return (time.perf_counter() - t0) * 1000
    return _fn


plot = Plot()

plot.add(
    images.transform(throttle(0.5)).map_data(lambda obs: image_utils.brightness(obs.data)),
    label="brightness",
    color=color.blue,
)

plot.add(
    images.transform(throttle(0.5)).map_data(lambda obs: slow_brightness(obs.data)),
    label="slow_brightness",
    style=Style.dashed,
    color=color.red,
)

plot.add(
    images.transform(throttle(0.5)).map_data(timed(lambda img: image_utils.brightness(img))),
    label="brightness (ms)",
    axis="time",
    color=color.blue,
    opacity=0.5,
)

plot.add(
    images.transform(throttle(0.5)).map_data(timed(slow_brightness)),
    label="slow_brightness (ms)",
    axis="time",
    color=color.red,
    opacity=0.5,
)


plot.to_svg("assets/plot_brightness_algo.svg")

delta_plot = Plot()

delta_plot.add(
    images.transform(throttle(0.5)).map_data(
        lambda obs: image_utils.brightness(obs.data) - slow_brightness(obs.data)
    ),
    label="delta (fast - slow)",
    color=color.green,
)

delta_plot.to_svg("assets/plot_brightness_algo_delta.svg")

```

![output](assets/plot_brightness_algo.svg)

![output](assets/plot_brightness_algo_delta.svg)

The plots compare the sampled brightness helper with the full-pixel mean on this recording. Rerun the example before drawing performance conclusions on another recording.

Above example loads the same data and iterates it for each plot line, it's a bit slow but readable and easy to write during development. Below is an example that generates the same results but more efficiently

```python
import time

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.transform import throttle
from dimos.memory.vis import color
from dimos.memory.vis.plot.elements import HLine, Series, Style
from dimos.memory.vis.plot.plot import Plot
from dimos.messages.sensor_msgs.msg import Image
from dimos.message_utils import image as image_utils
from dimos.utils.data import get_data

store = SqliteStore(path=get_data("go2_bigoffice.db"))
images = store.streams.color_image


def slow_brightness(img: Image) -> float:
    """Naive full-pixel mean, for reference."""
    pixels = image_utils.view(img)
    max_val = 65535.0 if img.encoding in ("mono16", "16UC1", "16SC1") else 255.0
    return float(pixels.mean() / max_val)


def timed(fn, img):
    """Call ``fn(img)`` once, return (value, ms)."""
    t0 = time.perf_counter()
    v = fn(img)
    return v, (time.perf_counter() - t0) * 1000


def compute(obs):
    """One pass per image: both values, both times, delta."""
    img = obs.data
    _ = img.data  # warm lazy load so only compute is timed
    fast_v, fast_ms = timed(lambda i: image_utils.brightness(i), img)
    slow_v, slow_ms = timed(slow_brightness, img)
    return {
        "fast": fast_v,
        "slow": slow_v,
        "fast_ms": fast_ms,
        "slow_ms": slow_ms,
        "delta": fast_v - slow_v,
    }


# Iterate the source once; all five series below read from the cache.
metrics = images.transform(throttle(0.5)).map_data(compute).materialize()

plot = Plot()
plot.add(metrics.map_data(lambda o: o.data["fast"]),
         label="brightness", color=color.blue)
plot.add(metrics.map_data(lambda o: o.data["slow"]),
         label="slow_brightness", color=color.red, style=Style.dashed)
plot.add(metrics.map_data(lambda o: o.data["fast_ms"]),
         label="brightness (ms)", axis="time", color=color.blue, opacity=0.5)
plot.add(metrics.map_data(lambda o: o.data["slow_ms"]),
         label="slow_brightness (ms)", axis="time", color=color.red, opacity=0.5)
plot.to_svg("assets/plot_brightness_algo.svg")

delta_plot = Plot()
delta_plot.add(metrics.map_data(lambda o: o.data["delta"]),
               label="delta (fast - slow)", color=color.green)
delta_plot.add(HLine(y=0, style=Style.dashed, color=color.red))
delta_plot.to_svg("assets/plot_brightness_algo_delta.svg")
```
