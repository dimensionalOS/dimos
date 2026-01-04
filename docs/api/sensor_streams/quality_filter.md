# Quality-Based Stream Filtering

When processing sensor streams, you often want to reduce frequency while keeping the best quality data. Instead of blindly dropping frames, `quality_barrier` selects the highest quality item within each time window.

## The Problem

A camera outputs 30fps, but your ML model only needs 2fps. Simple approaches:

- **`sample(0.5)`** - Takes whatever frame happens to land on the interval tick
- **`throttle_first(0.5)`** - Takes the first frame, ignores the rest

Both ignore quality. You might get a blurry frame when a sharp one was available.

## The Solution: `quality_barrier`

```python session=qb
import reactivex as rx
from reactivex import operators as ops
from dimos.utils.reactive import quality_barrier

# Simulated sensor data with quality scores
data = [
    {"id": 1, "quality": 0.3},
    {"id": 2, "quality": 0.9},  # best in first window
    {"id": 3, "quality": 0.5},
    {"id": 4, "quality": 0.2},
    {"id": 5, "quality": 0.8},  # best in second window
    {"id": 6, "quality": 0.4},
]

source = rx.of(*data)

# Select best quality item per window (2 items per second = 0.5s windows)
result = source.pipe(
    quality_barrier(lambda x: x["quality"], target_frequency=2.0),
    ops.to_list(),
).run()

print("Selected:", [r["id"] for r in result])
print("Qualities:", [r["quality"] for r in result])
```

<!--Result:-->
```
Selected: [2]
Qualities: [0.9]
```

## Image Sharpness Filtering

For camera streams, we provide `sharpness_barrier` which uses the image's sharpness score:

```python session=qb
import numpy as np
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier

# Create test images with different sharpness levels
def make_image(sharpness_level: str) -> Image:
    """Create a test image. Sharp = high contrast edges, blurry = smooth gradients."""
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    if sharpness_level == "sharp":
        # Sharp edges - high gradient
        img[40:60, 40:60] = 255
    elif sharpness_level == "medium":
        # Softer edges
        for i in range(20):
            val = int(255 * i / 20)
            img[40+i:41+i, 30:70] = val
    else:
        # Blurry - very low gradient
        img[:, :] = 128
    return Image(data=img)  # Note: use data= keyword

sharp = make_image("sharp")
medium = make_image("medium")
blurry = make_image("blurry")

print(f"Sharp image sharpness:  {sharp.sharpness:.3f}")
print(f"Medium image sharpness: {medium.sharpness:.3f}")
print(f"Blurry image sharpness: {blurry.sharpness:.3f}")
```

<!--Result:-->
```
Sharp image sharpness:  0.351
Medium image sharpness: 0.372
Blurry image sharpness: 0.000
```

Using `sharpness_barrier` in a stream:

```python session=qb
# Stream of images arriving over time
images = rx.of(blurry, medium, sharp, blurry, medium)

# Select sharpest image per window at 2Hz
result = images.pipe(
    sharpness_barrier(2.0),
    ops.to_list(),
).run()

print(f"Selected {len(result)} image(s)")
print(f"Selected sharpness: {[f'{img.sharpness:.3f}' for img in result]}")
```

<!--Result:-->
```
Selected 1 image(s)
Selected sharpness: ['0.372']
```

### Usage in Camera Module

Here's how it's used in the actual camera module:

```python session=qb
from dimos.core.module import Module

class CameraModule(Module):
    frequency: float = 2.0  # Target output frequency

    def start(self):
        # Simulated camera stream
        stream = rx.of(blurry, sharp, medium, blurry, sharp)

        # Apply sharpness filter if frequency is set
        if self.frequency > 0:
            stream = stream.pipe(sharpness_barrier(self.frequency))

        # Collect results
        results = []
        stream.subscribe(lambda img: results.append(img.sharpness))
        return results

cam = CameraModule()
sharpnesses = cam.start()
print(f"Output sharpnesses: {[f'{s:.3f}' for s in sharpnesses]}")
```

<!--Result:-->
```
Output sharpnesses: ['0.372']
```

### How Sharpness is Calculated

The sharpness score (0.0 to 1.0) is computed using Sobel edge detection:

```python session=qb
import cv2

# Get the sharp image and show the calculation
img = sharp
gray = img.to_grayscale()

# Sobel gradients - use .data to get the underlying numpy array
sx = cv2.Sobel(gray.data, cv2.CV_32F, 1, 0, ksize=5)
sy = cv2.Sobel(gray.data, cv2.CV_32F, 0, 1, ksize=5)
magnitude = cv2.magnitude(sx, sy)

print(f"Mean gradient magnitude: {magnitude.mean():.2f}")
print(f"Normalized sharpness:    {img.sharpness:.3f}")
```

<!--Result:-->
```
Mean gradient magnitude: 250.79
Normalized sharpness:    0.351
```

## Custom Quality Functions

You can use `quality_barrier` with any quality metric:

```python session=qb
# Example: select by "confidence" field
detections = [
    {"name": "cat", "confidence": 0.7},
    {"name": "dog", "confidence": 0.95},  # best
    {"name": "bird", "confidence": 0.6},
]

result = rx.of(*detections).pipe(
    quality_barrier(lambda d: d["confidence"], target_frequency=2.0),
    ops.to_list(),
).run()

print(f"Selected: {result[0]['name']} (conf: {result[0]['confidence']})")
```

<!--Result:-->
```
Selected: dog (conf: 0.95)
```

## API Reference

### `quality_barrier(quality_func, target_frequency)`

RxPY pipe operator that selects the highest quality item within each time window.

| Parameter          | Type                   | Description                                          |
|--------------------|------------------------|------------------------------------------------------|
| `quality_func`     | `Callable[[T], float]` | Function that returns a quality score for each item  |
| `target_frequency` | `float`                | Output frequency in Hz (e.g., 2.0 for 2 items/second)|

**Returns:** A pipe operator for use with `.pipe()`

### `sharpness_barrier(target_frequency)`

Convenience wrapper for images that uses `image.sharpness` as the quality function.

| Parameter          | Type    | Description              |
|--------------------|---------|--------------------------|
| `target_frequency` | `float` | Output frequency in Hz   |

**Returns:** A pipe operator for use with `.pipe()`
