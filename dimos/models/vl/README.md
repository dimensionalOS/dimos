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
