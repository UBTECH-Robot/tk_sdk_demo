import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

# ros2 run grab_demo yolo_grab_node

class YoloGrabNode(Node):
    def __init__(self):
        super().__init__("yolo_grab_node")

        self.bridge = CvBridge()

        # 加载 YOLO（CPU）
        self.model = YOLO("yolo_models/yolov8n.pt")

        # 订阅相机
        self.sub = self.create_subscription(
            Image,
            "/ob_camera_head/color/image_raw",
            self.image_cb,
            10
        )

        # 发布抓取点
        self.pub = self.create_publisher(Pose, "/grasp_pose", 10)

        self.get_logger().info("YOLO Grab Node started")

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # YOLO 推理
        results = self.model.predict(
            source=img,
            device="cpu",
            conf=0.4,
            verbose=False
        )

        if len(results[0].boxes) == 0:
            return

        # 取第一个检测框
        box = results[0].boxes[0]
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

        # 计算抓取点（bbox 中心）
        cx = float((x1 + x2) / 2)
        cy = float((y1 + y2) / 2)

        # 发布 Pose（像素坐标，后续可转3D）
        pose = Pose()
        pose.position.x = cx
        pose.position.y = cy
        pose.position.z = 0.0

        pose.orientation.w = 1.0

        self.pub.publish(pose)

        self.get_logger().info(
            f"Grasp pixel: ({cx:.1f}, {cy:.1f})"
        )


def main():
    rclpy.init()
    node = YoloGrabNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
