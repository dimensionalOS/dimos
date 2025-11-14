"""Detects COCO objects in image and publishes in ROS2.

Subscribes to /image and publishes Detection2DArray message on topic /detected_objects.
Also publishes (by default) annotated image with bounding boxes on /annotated_image.
Uses PyTorch and FasterRCNN_MobileNet model from torchvision.
Bounding Boxes use image convention, ie center.y = 0 means top of image.
"""

import collections
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, ObjectHypothesis, ObjectHypothesisWithPose
from vision_msgs.msg import Detection2D, Detection2DArray
from cv_bridge import CvBridge
import torch
from torchvision.models import detection as detection_model
from torchvision.utils import draw_bounding_boxes

Detection = collections.namedtuple("Detection", "label, bbox, score")

class CocoDetectorNode(Node):
    """Detects COCO objects in image and publishes on ROS2.

    Subscribes to /image and publishes Detection2DArray on /detected_objects.
    Also publishes augmented image with bounding boxes on /annotated_image.
    """

    # pylint: disable=R0902 disable too many instance variables warning for this class
    def __init__(self):
        super().__init__("coco_detector_node")
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('detection_threshold', 0.9)
        self.declare_parameter('publish_annotated_image', True)
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.detection_threshold = \
            self.get_parameter('detection_threshold').get_parameter_value().double_value
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.listener_callback,
            10)
        self.detected_objects_publisher = \
            self.create_publisher(Detection2DArray, "detected_objects", 10)
        if self.get_parameter('publish_annotated_image').get_parameter_value().bool_value:
            self.annotated_image_publisher = \
                self.create_publisher(Image, "annotated_image", 10)
        else:
            self.annotated_image_publisher = None
        self.bridge = CvBridge()
        self.model = detection_model.fasterrcnn_mobilenet_v3_large_320_fpn(
            weights="FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.COCO_V1",
            progress=True,
            weights_backbone="MobileNet_V3_Large_Weights.IMAGENET1K_V1").to(self.device)
        self.class_labels = \
            detection_model.FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT.meta["categories"]
        self.model.eval()
        self.get_logger().info("Node has started.")

    def mobilenet_to_ros2(self, detection, header):
        """Converts a Detection tuple(label, bbox, score) to a ROS2 Detection2D message."""

        detection2d = Detection2D()
        detection2d.header = header
        object_hypothesis_with_pose = ObjectHypothesisWithPose()
        object_hypothesis = ObjectHypothesis()
        object_hypothesis.class_id = self.class_labels[detection.label]
        object_hypothesis.score = detection.score.detach().item()
        object_hypothesis_with_pose.hypothesis = object_hypothesis
        detection2d.results.append(object_hypothesis_with_pose)
        bounding_box = BoundingBox2D()
        bounding_box.center.position.x = float((detection.bbox[0] + detection.bbox[2]) / 2)
        bounding_box.center.position.y = float((detection.bbox[1] + detection.bbox[3]) / 2)
        bounding_box.center.theta = 0.0
        bounding_box.size_x = float(2 * (bounding_box.center.position.x - detection.bbox[0]))
        bounding_box.size_y = float(2 * (bounding_box.center.position.y - detection.bbox[1]))
        detection2d.bbox = bounding_box
        return detection2d

    def publish_annotated_image(self, filtered_detections, header, image):
        """Draws the bounding boxes on the image and publishes to /annotated_image"""

        if len(filtered_detections) > 0:
            pred_boxes = torch.stack([detection.bbox for detection in filtered_detections])
            pred_labels = [self.class_labels[detection.label] for detection in filtered_detections]
            annotated_image = draw_bounding_boxes(torch.tensor(image), pred_boxes,
                                                  pred_labels, colors="yellow")
        else:
            annotated_image = torch.tensor(image)
        ros2_image_msg = self.bridge.cv2_to_imgmsg(annotated_image.numpy().transpose(1, 2, 0),
                                                   encoding="rgb8")
        ros2_image_msg.header = header
        self.annotated_image_publisher.publish(ros2_image_msg)

    def listener_callback(self, msg):
        """Reads image and publishes on /detected_objects and /annotated_image."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        image = cv_image.copy().transpose((2, 0, 1))
        batch_image = np.expand_dims(image, axis=0)
        tensor_image = torch.tensor(batch_image/255.0, dtype=torch.float, device=self.device)
        mobilenet_detections = self.model(tensor_image)[0]  # pylint: disable=E1102 disable not callable warning
        filtered_detections = [Detection(label_id, box, score) for label_id, box, score in
            zip(mobilenet_detections["labels"],
            mobilenet_detections["boxes"],
            mobilenet_detections["scores"]) if score >= self.detection_threshold]
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        detection_array.detections = \
            [self.mobilenet_to_ros2(detection, msg.header) for detection in filtered_detections]
        self.detected_objects_publisher.publish(detection_array)
        if self.annotated_image_publisher is not None:
            self.publish_annotated_image(filtered_detections, msg.header, image)


rclpy.init()
coco_detector_node = CocoDetectorNode()
rclpy.spin(coco_detector_node)
coco_detector_node.destroy_node()
rclpy.shutdown()
