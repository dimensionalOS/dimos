# Vision Language Models

This provides vision language model implementations for processing images and text queries.

## QwenVL Model

The `QwenVlModel` class provides access to Alibaba's Qwen2.5-VL model for vision-language tasks.

### Example Usage

```python
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.sensor_msgs.Image import Image

# Initialize the model (requires ALIBABA_API_KEY environment variable)
model = QwenVlModel()

image = Image.from_file("path/to/your/image.jpg")

response = model.query(image.data, "What do you see in this image?")
print(response)
```

## Moondream Hosted Model

The `MoondreamHostedVlModel` class provides access to the hosted Moondream API for fast vision-language tasks.

**Prerequisites:**

You must export your API key before using the model:
```bash
export MOONDREAM_API_KEY="your_api_key_here"
```

### Capabilities

The model supports four modes of operation:

1. **Caption**: Generate a description of the image.
2. **Query**: Ask natural language questions about the image.
3. **Detect**: Find bounding boxes for specific objects.
4. **Point**: Locate the center points of specific objects.

### Example Usage

```python
from dimos.models.vl.moondream_hosted import MoondreamHostedVlModel
from dimos.msgs.sensor_msgs import Image

model = MoondreamHostedVlModel()
image = Image.from_file("path/to/image.jpg")

# 1. Caption
print(f"Caption: {model.caption(image)}")

# 2. Query
print(f"Answer: {model.query(image, 'Is there a person in the image?')}")

# 3. Detect (returns ImageDetections2D)
detections = model.query_detections(image, "person")
for det in detections.detections:
    print(f"Found person at {det.bbox}")

# 4. Point (returns list of (x, y) coordinates)
points = model.point(image, "person")
print(f"Person centers: {points}")
```
