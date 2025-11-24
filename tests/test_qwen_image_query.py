"""Test the Qwen image query functionality."""

import os
from PIL import Image
from dimos.models.qwen.video_query import query_single_frame

def test_qwen_image_query():
    """Test querying Qwen with a single image."""
    # Skip if no API key
    if not os.getenv('ALIBABA_API_KEY'):
        print("ALIBABA_API_KEY not set")
        return
        
    # Load test image
    image_path = os.path.join(os.getcwd(), "assets", "test_spatial_memory", "frame_038.jpg")
    image = Image.open(image_path)
    
    # Test basic object detection query
    response = query_single_frame(
        image=image,
        query="What objects do you see in this image? Return as a comma-separated list."
    )
    print(response)
    
    # Test coordinate query
    response = query_single_frame(
        image=image,
        query="Return the center coordinates of any person in the image as a tuple (x,y)"
    )
    print(response)
    
if __name__ == "__main__":
    test_qwen_image_query()