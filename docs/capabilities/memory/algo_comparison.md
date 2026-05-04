
Example on how we can use memory to compare two algos on a real data.

```python
import time

from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import throttle
from dimos.memory2.vis import color
from dimos.memory2.vis.plot.elements import Style
from dimos.memory2.vis.plot.plot import Plot
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.data import get_data

store = SqliteStore(path=get_data("go2_bigoffice.db"))
images = store.streams.color_image


def slow_brightness(img: Image) -> float:
    """Naive full-pixel mean, for reference."""
    max_val = 65535.0 if img.format in (ImageFormat.GRAY16, ImageFormat.DEPTH16) else 255.0
    return float(img.data.mean() / max_val)


def timed(fn):
    """Wrap ``fn(img) -> float`` so it returns execution time in ms instead.

    Touches ``img.data.shape`` first so the lazy blob load isn't counted.
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
    images.transform(throttle(0.5)).map_data(lambda obs: obs.data.brightness),
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
    images.transform(throttle(0.5)).map_data(timed(lambda img: img.brightness)),
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
        lambda obs: obs.data.brightness - slow_brightness(obs.data)
    ),
    label="delta (fast - slow)",
    color=color.green,
)

delta_plot.to_svg("assets/plot_brightness_algo_delta.svg")

```

![output](assets/plot_brightness_algo.svg)

![output](assets/plot_brightness_algo_delta.svg)

We see that new algo is strictly better.

Above example loads the same data and iterates it for each plot line, it's a bit slow but readable and easy to write during development. Below is an example that generates the same results but more efficiently

```python
import time

from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import throttle
from dimos.memory2.vis import color
from dimos.memory2.vis.plot.elements import HLine, Series, Style
from dimos.memory2.vis.plot.plot import Plot
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.data import get_data

store = SqliteStore(path=get_data("go2_bigoffice.db"))
images = store.streams.color_image


def slow_brightness(img: Image) -> float:
    """Naive full-pixel mean, for reference."""
    max_val = 65535.0 if img.format in (ImageFormat.GRAY16, ImageFormat.DEPTH16) else 255.0
    return float(img.data.mean() / max_val)


def timed(fn, img):
    """Call ``fn(img)`` once, return (value, ms)."""
    t0 = time.perf_counter()
    v = fn(img)
    return v, (time.perf_counter() - t0) * 1000


def compute(obs):
    """One pass per image: both values, both times, delta."""
    img = obs.data
    _ = img.data  # warm lazy load so only compute is timed
    fast_v, fast_ms = timed(lambda i: i.brightness, img)
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
