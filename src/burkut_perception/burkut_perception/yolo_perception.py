#!/usr/bin/env python3
"""
YOLO Perception Node
/camera/image_raw → YOLO tespiti → /yolo/detections (Detection2DArray)
                                  → /yolo/image (Image) [debug görüntü]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class YoloPerceptionNode(Node):

    def __init__(self):
        super().__init__('yolo_perception')

        self.declare_parameter('model_path',
            '/home/tamer/burkut_dataset/runs/detect/yolo_kirmizi/runs/detect/train_color/weights/best.pt')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('device', 'cuda')  # 'cuda' veya 'cpu'

        model_path = self.get_parameter('model_path').value
        self.conf  = self.get_parameter('confidence').value
        device     = self.get_parameter('device').value

        if not YOLO_AVAILABLE:
            self.get_logger().error('ultralytics yuklu degil! pip install ultralytics')
            return

        self.get_logger().info(f'Model yukleniyor: {model_path}')
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info(f'Model yuklendi. Device: {device}  Conf: {self.conf}')

        self.bridge = CvBridge()

        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.pub_detections = self.create_publisher(
            Detection2DArray, '/yolo/detections', 10)

        self.pub_debug = self.create_publisher(
            Image, '/yolo/image', 10)

        self.get_logger().info('YOLO Perception Node baslatildi.')

    def image_callback(self, msg: Image):
        # ROS Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO inference
        results = self.model(frame, conf=self.conf, verbose=False)

        now = self.get_clock().now().to_msg()

        det_array = Detection2DArray()
        det_array.header = Header()
        det_array.header.stamp = now
        det_array.header.frame_id = 'camera_optical_frame'

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf  = float(box.conf[0])
                cls   = int(box.cls[0])

                det = Detection2D()
                det.header = det_array.header

                # BoundingBox — merkez + boyut
                bbox = BoundingBox2D()
                bbox.center.position.x = (x1 + x2) / 2.0
                bbox.center.position.y = (y1 + y2) / 2.0
                bbox.size_x = float(x2 - x1)
                bbox.size_y = float(y2 - y1)
                det.bbox = bbox

                # Hypothesis
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(cls)
                hyp.hypothesis.score = conf
                det.results.append(hyp)

                det_array.detections.append(det)

        self.pub_detections.publish(det_array)

        # Debug görüntü — bbox çiz
        debug = frame.copy()
        for det in det_array.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w  = det.bbox.size_x
            h  = det.bbox.size_y
            x1 = int(cx - w/2); y1 = int(cy - h/2)
            x2 = int(cx + w/2); y2 = int(cy + h/2)
            conf = det.results[0].hypothesis.score

            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(debug, f'pole {conf:.2f}',
                        (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

        self.pub_debug.publish(
            self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

        if det_array.detections:
            self.get_logger().info(
                f'{len(det_array.detections)} direk tespit edildi')


def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
