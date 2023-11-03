#!/usr/bin/env python3
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

package_name = "detector_ros"
package_share_directory = get_package_share_directory(package_name)


class ObjectBboxNode(Node):
    def __init__(self):
        super().__init__("object_bbox_node")
        self.logger = self.get_logger()

        self.declare_parameter("show_bbox_image", True)
        self.declare_parameter("dps", 15)

        self.declare_parameter("weights", "yolov8n.pt")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("max_det", 300)

        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("bbox_result_topic", "image_raw/bbox/result")
        self.declare_parameter("bbox_image_topic", "image_raw/bbox/image")

        self.show_bbox_image = (
            self.get_parameter("show_bbox_image").get_parameter_value().bool_value
        )
        dps = self.get_parameter("dps").get_parameter_value().integer_value

        self.conf = self.get_parameter("conf").get_parameter_value().double_value
        self.iou = self.get_parameter("iou").get_parameter_value().double_value
        self.max_det = self.get_parameter("max_det").get_parameter_value().integer_value
        weights = self.get_parameter("weights").get_parameter_value().string_value

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        bbox_result_topic = (
            self.get_parameter("bbox_result_topic").get_parameter_value().string_value
        )
        bbox_image_topic = (
            self.get_parameter("bbox_image_topic").get_parameter_value().string_value
        )

        self.sub_image = self.create_subscription(Image, image_topic, self.sub_cb, 1)
        self.pub_timer = self.create_timer(1 / dps, self.pub_cb)
        self.pub_result = self.create_publisher(Detection2DArray, bbox_result_topic, 1)
        self.pub_image = self.create_publisher(Image, bbox_image_topic, 1)

        self.bridge = CvBridge()

        self.model = YOLO(f"{package_share_directory}/weights/{weights}")
        self.logger.info(f"Loaded weights {weights}")

        self.msg = None

    def pub_cb(self):
        if self.msg is None:
            return

        results = self.model.predict(
            source=self.bridge.imgmsg_to_cv2(self.msg, desired_encoding="bgr8"),
            conf=self.conf,
            iou=self.iou,
            max_det=self.max_det,
            verbose=False,
        )
        result = results[0]

        detections_msg = self.create_detections_msg(result)
        self.pub_result.publish(detections_msg)
        if self.show_bbox_image:
            self.pub_bbox_image(result)

    def create_detections_msg(self, result):
        detections_msg = Detection2DArray()
        detections_msg.header = self.msg.header

        boxes = result.boxes.xywhn
        clss = result.boxes.cls
        confs = result.boxes.conf
        for bbox, cls, conf in zip(boxes, clss, confs):
            detection = Detection2D()
            detection.bbox.center.x = float(bbox[0])
            detection.bbox.center.y = float(bbox[1])
            detection.bbox.size_x = float(bbox[2])
            detection.bbox.size_y = float(bbox[3])
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = result.names.get(int(cls))
            hypothesis.score = float(conf)
            detection.results.append(hypothesis)
            detections_msg.detections.append(detection)

        return detections_msg

    def pub_bbox_image(self, result):
        bbox_image_msg = self.bridge.cv2_to_imgmsg(result.plot(), encoding="bgr8")
        bbox_image_msg.header = self.msg.header
        self.pub_image.publish(bbox_image_msg)

    def sub_cb(self, msg):
        self.msg = msg


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ObjectBboxNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
