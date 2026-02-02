import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import shutil
from datetime import datetime

# ros2 run grab_demo yolo_grab_node

class YoloGrabNode(Node):
    def __init__(self):
        super().__init__("yolo_grab_node")

        self.bridge = CvBridge()

        # 创建保存目录
        self.save_dir = "saved_data/grab_node"
        if os.path.exists(self.save_dir):
            shutil.rmtree(self.save_dir)
        os.makedirs(self.save_dir)

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

    def save_image(self, img, timestamp, suffix="image"):
        """保存图片到指定目录"""
        filename = f"{timestamp}_{suffix}.jpg"
        filepath = os.path.join(self.save_dir, filename)
        cv2.imwrite(filepath, img)
        self.get_logger().info(f"Saved: {filename}")
        return filepath

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 生成时间戳（同一帧使用相同时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

        # 1. 保存原始图片
        self.save_image(img, timestamp, suffix="raw")

        # YOLO 推理
        results = self.model.predict(
            source=img,
            device="cpu",
            conf=0.4,
            verbose=False
        )

        if len(results[0].boxes) == 0:
            return

        # 创建带标注的图片副本
        annotated_img = img.copy()

        # 定义颜色列表（BGR格式）
        colors = [
            (0, 255, 0),    # 绿色
            (255, 0, 0),    # 蓝色
            (0, 0, 255),    # 红色
            (255, 255, 0),  # 青色
            (255, 0, 255),  # 品红
            (0, 255, 255),  # 黄色
        ]

        # 遍历所有检测框
        for i, box in enumerate(results[0].boxes):
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # 获取类别和置信度
            cls = int(box.cls[0].cpu().numpy())
            conf = float(box.conf[0].cpu().numpy())
            label = f"{results[0].names[cls]} {conf:.2f}"
            
            # 选择颜色
            color = colors[i % len(colors)]
            
            # 画检测框
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), color, 2)
            
            # 画标签背景
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(
                annotated_img,
                (x1, y1 - text_height - baseline - 5),
                (x1 + text_width, y1),
                color,
                -1
            )
            
            # 画标签文字
            cv2.putText(
                annotated_img,
                label,
                (x1, y1 + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2
            )

        # 2. 保存带标注的图片
        self.save_image(annotated_img, timestamp, suffix="detected")

        # 取第一个检测框计算抓取点
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
            f"Grasp pixel: ({cx:.1f}, {cy:.1f}), Detected {len(results[0].boxes)} objects"
        )


def main():
    rclpy.init()
    node = YoloGrabNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 时捕获中断信号
        print("\n用户中断程序")
    finally:
        # 清理资源：销毁节点、关闭 ROS2 通信
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
