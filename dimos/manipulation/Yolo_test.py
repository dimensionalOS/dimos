# Quick YOLO test
from ultralytics import YOLO
import cv2

# Load model directly
model = YOLO('yolov8n.pt')  # Use a known good model

# Test on your camera image
img = cv2.imread('/tmp/test_rgb.jpg')  # Use your saved test image
results = model(img, conf=0.25)

print(f"Detected {len(results[0].boxes)} objects")
for box in results[0].boxes[:5]:
    print(f"  Class: {model.names[int(box.cls)]}, Conf: {box.conf:.2f}")