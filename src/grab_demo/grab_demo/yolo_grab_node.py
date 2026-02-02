import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import shutil
import sys
from datetime import datetime
from message_filters import ApproximateTimeSynchronizer, Subscriber

# ros2 run grab_demo yolo_grab_node --target_classes apple
# 使用 ApproximateTimeSynchronizer 对彩色图和深度图进行帧同步处理，只能运行在41.2的orin板上（也就是头部相机所连接的板）。
# 在41.1的x86板上运行则会出现无法进行同步的问题，彩色图和深度图的传输会大量无效占用带宽，导致 synchronized_image_cb 回调长时间无法被调用。

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
        
        # ============ 目标类别配置 ============
        # 指定要检测的物体类别（大小写不敏感）
        # 
        # 使用方法 1 - 直接修改代码中的默认值：
        #   在 __init__ 方法中修改 target_classes 的值
        #
        # 示例用法：
        #   1. 只检测苹果：--target_classes apple
        #   2. 检测苹果和橙子：--target_classes apple orange
        #   3. 检测人和车：--target_classes person car
        #   4. 检测所有物体：--target_classes all
        #
        # 可用的类别列表（YOLO v8n 支持的 COCO 数据集类别，共 80 类）：
        #   person, bicycle, car, motorbike, aeroplane, bus, train, truck,
        #   boat, traffic light, fire hydrant, stop sign, parking meter, bench,
        #   cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe, backpack,
        #   umbrella, handbag, tie, suitcase, frisbee, skis, snowboard, sports ball,
        #   kite, baseball bat, baseball glove, skateboard, surfboard, tennis racket,
        #   bottle, wine glass, cup, fork, knife, spoon, bowl, banana, apple, sandwich,
        #   orange, broccoli, carrot, hot dog, pizza, donut, cake, chair, couch,
        #   potted plant, bed, dining table, toilet, tv, laptop, mouse, remote,
        #   keyboard, microwave, oven, toaster, sink, refrigerator, book, clock,
        #   vase, scissors, teddy bear, hair drier, toothbrush
        #
        
        # 从命令行参数解析目标类别
        target_classes = self._parse_target_classes_from_args()
        
        # 获取模型的所有类别名称（字典：类别ID -> 类别名称）
        self.class_names = self.model.names  # {'0': 'person', '1': 'bicycle', ..., '47': 'apple'}
        
        # 构建类别名称到 ID 的映射（小写以支持大小写不敏感的搜索）
        self.class_id_map = {v.lower(): k for k, v in self.class_names.items()}
        
        # 将指定的目标类别名称转换为 YOLO 类别 ID
        self.target_class_ids = []
        if target_classes and target_classes != ['all']:  # 只有当 target_classes 非空且不是 'all' 时才过滤
            for class_name in target_classes:
                class_name_lower = class_name.lower()
                if class_name_lower in self.class_id_map:
                    class_id = self.class_id_map[class_name_lower]
                    self.target_class_ids.append(class_id)
                    self.get_logger().info(f"✓ 目标类别 '{class_name}' -> ID {class_id}")
                else:
                    self.get_logger().warn(
                        f"✗ 类别 '{class_name}' 不在模型中。"
                        f"可用类别: {sorted(set(self.class_names.values()))}"
                    )
        
        # 如果指定了类别但都无效，打印警告
        if target_classes and target_classes != ['all'] and not self.target_class_ids:
            self.get_logger().warn("⚠ 未找到有效的目标类别，将检测所有物体")
        
        # 如果 target_classes 为空或是 'all'，则不进行类别过滤
        if self.target_class_ids:
            self.get_logger().info(f"类别过滤已启用，检测目标: {target_classes}")
        else:
            self.get_logger().info("类别过滤已禁用，检测所有物体")

        # ============ 消息同步配置 ============
        # 使用 message_filters 进行帧同步
        # ApproximateTimeSynchronizer: 近似时间同步器
        # 功能：同步来自不同话题的消息，允许时间戳有小偏差，
        # 应用场景：不同传感器采样频率和发布时间不同，时间戳有轻微偏差，如彩色相机和深度相机同步、激光雷达和IMU数据融合。
        
        
        # 创建彩色图像订阅器
        color_sub = Subscriber(self, Image, "/ob_camera_head/color/image_raw")
        
        # 创建深度图像订阅器
        depth_sub = Subscriber(self, Image, "/ob_camera_head/depth/image_raw")
        
        # 创建同步器：queue_size=10表示缓冲区大小，slop=0.1表示允许100ms的时间差
        # 当接收到来自两个话题的消息时，如果时间差在slop范围内，就认为是同一帧
        self.ts = ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=10,
            slop=0.1  # 允许100ms的时间差
        )
        
        # 注册同步回调函数
        self.ts.registerCallback(self.synchronized_image_cb)

        # 发布抓取点
        self.pub = self.create_publisher(Pose, "/grasp_pose", 10)
        
        # 存储最新的深度数据（用于在回调中访问）
        self.latest_depth_img = None
        self.latest_color_timestamp = None
        
        # ============ 获取深度相机校准信息 ============
        # 订阅深度相机的 camera_info 话题
        # camera_info 包含相机的内参矩阵、畸变系数等
        # 使用默认 QoS 策略以匹配发布者
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/ob_camera_head/depth/camera_info",
            self.depth_camera_info_cb,
            10  # 使用默认 QoS，queue size = 10
        )
        
        # 标志位：记录是否已获取过camera_info
        self.camera_info_received = False
        self.depth_unit = None  # 深度单位（如果能从camera_info获取）
        
        # ============ 相机内参矩阵（用于像素坐标到3D坐标的反投影） ============
        # 相机内参矩阵 K 的形式：
        #   K = [[fx,  0, cx],
        #        [ 0, fy, cy],
        #        [ 0,  0,  1]]
        # 其中：
        #   fx, fy: 焦距（像素单位）
        #   cx, cy: 主点坐标（图像中心，像素坐标）
        # 这些参数在 depth_camera_info_cb 回调中从 camera_info 消息中获取
        self.camera_matrix_K = None  # 相机内参矩阵（3x3）
        self.fx = None  # 焦距 x
        self.fy = None  # 焦距 y
        self.cx = None  # 主点 x
        self.cy = None  # 主点 y

        self.get_logger().info("YOLO Grab Node started with frame synchronization")

    def _parse_target_classes_from_args(self):
        """
        从命令行参数解析目标类别
        
        使用方法：
            python3 -m yolo_grab_node --target_classes apple
            python3 -m yolo_grab_node --target_classes apple orange
            python3 -m yolo_grab_node --target_classes person car bicycle
            python3 -m yolo_grab_node  # 使用默认值 ['apple']
        
        Returns:
            list: 目标类别列表，如 ['apple', 'orange']，或默认值 ['apple']
        """
        target_classes = ['apple']  # 默认值：检测苹果
        
        # 检查命令行中是否有 --target_classes 参数
        if '--target_classes' in sys.argv:
            try:
                idx = sys.argv.index('--target_classes')
                # 获取 --target_classes 后面的所有参数（直到下一个以 -- 开头的参数或列表结束）
                classes = []
                for i in range(idx + 1, len(sys.argv)):
                    arg = sys.argv[i]
                    # 如果遇到以 -- 开头的参数，停止收集
                    if arg.startswith('--'):
                        break
                    classes.append(arg)
                
                # 如果有提供类别参数，则使用提供的值
                if classes:
                    target_classes = classes
                    print(f"[INFO] 从命令行解析到目标类别: {target_classes}")
                else:
                    print(f"[WARN] --target_classes 后没有指定类别，使用默认值: {target_classes}")
            except Exception as e:
                print(f"[ERROR] 解析命令行参数失败: {e}，使用默认值: {target_classes}")
        else:
            print(f"[INFO] 未指定 --target_classes 参数，使用默认值: {target_classes}")
            print(f"[INFO] 使用方法: python3 -m grab_demo.yolo_grab_node --target_classes apple orange")
        
        return target_classes

    def save_image(self, img, timestamp, suffix="image"):
        """保存图片到指定目录"""
        filename = f"{timestamp}_{suffix}.jpg"
        filepath = os.path.join(self.save_dir, filename)
        cv2.imwrite(filepath, img)
        self.get_logger().info(f"Saved: {filename}")
        return filepath

    def depth_camera_info_cb(self, msg):
        """获取深度相机的校准信息
        
        camera_info 话题包含相机的内参矩阵、图像分辨率等信息
        某些Orbbec摄像头的camera_info中的 binning_x 可能用于存储深度单位信息
        
        Args:
            msg: sensor_msgs/msg/CameraInfo 消息
        """
        if self.camera_info_received:
            return
        
        self.camera_info_received = True
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("深度相机校准信息 (Depth Camera Info):")
        self.get_logger().info("=" * 60)
        
        # 打印基本信息
        self.get_logger().info(f"相机名称 (Camera Name): {msg.header.frame_id}")
        self.get_logger().info(f"图像分辨率 (Image Size): {msg.width} x {msg.height}")
        
        # 打印内参矩阵（K矩阵）
        self.get_logger().info("相机内参矩阵 (Camera Matrix K):")
        K = np.array(msg.k).reshape(3, 3)
        self.get_logger().info(f"\n{K}")
        
        # 打印畸变系数
        self.get_logger().info(f"畸变系数 (Distortion Coefficients): {msg.d}")
        
        # ============ 提取并保存相机内参矩阵 ============
        # 从 camera_info 消息中提取内参矩阵 K
        # 这些参数用于将像素坐标反投影到相机坐标系下的3D坐标
        self.camera_matrix_K = np.array(msg.k).reshape(3, 3)
        self.fx = self.camera_matrix_K[0, 0]  # 焦距 x
        self.fy = self.camera_matrix_K[1, 1]  # 焦距 y
        self.cx = self.camera_matrix_K[0, 2]  # 主点 x
        self.cy = self.camera_matrix_K[1, 2]  # 主点 y
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("相机内参矩阵参数已保存：")
        self.get_logger().info(f"  焦距 fx: {self.fx:.2f} px")
        self.get_logger().info(f"  焦距 fy: {self.fy:.2f} px")
        self.get_logger().info(f"  主点 cx: {self.cx:.2f} px")
        self.get_logger().info(f"  主点 cy: {self.cy:.2f} px")
        self.get_logger().info("这些参数用于像素坐标到3D相机坐标系的反投影转换")
        self.get_logger().info("="*60 + "\n")
        
        # 检查是否有特殊标记表示深度单位
        # Orbbec SDK 某些版本在 camera_info 的其他字段中标记深度单位
        # 常见的有：
        # - binning_x 可能存储深度缩放因子
        # - binning_y 可能存储其他信息
        self.get_logger().info(f"Binning X (可能包含深度缩放信息): {msg.binning_x}")
        self.get_logger().info(f"Binning Y: {msg.binning_y}")
        
        # ROI (Region of Interest) 信息
        # self.get_logger().info(f"ROI: {msg.roi}")
        
        # self.get_logger().info("=" * 60)
        # self.get_logger().info("深度单位确认方法:")
        # self.get_logger().info("1. 查看 Orbbec SDK 文档或ROS驱动说明")
        # self.get_logger().info("2. 查看日志中实际深度值范围 (下面的YOLO回调会输出)")
        # self.get_logger().info("3. 通过对已知距离物体的测量来校准")
        # self.get_logger().info("=" * 60)

    def draw_label_with_background(self, img, label, x1, y1, x2, y2, color):
        """绘制标签背景和文字，智能处理边界
        
        如果框上方在图片边缘，则标签显示在框外下方
        如果框下方在图片边缘，则标签显示在框外上方
        
        Args:
            img: 输入图片
            label: 标签文字
            x1, y1, x2, y2: 检测框坐标
            color: 标签背景颜色
        """
        img_height, img_width = img.shape[:2]
        
        # 获取文字尺寸
        (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        
        # 标签背景的高度
        label_bg_height = text_height + baseline + 5
        
        # 默认优先显示在框的上方
        label_top = y1 - label_bg_height
        label_bottom = y1
        text_y = y1 - baseline - 2
        
        # 检查上方空间是否足够，如果不足则显示在下方
        if label_top < 0:
            label_top = y2
            label_bottom = y2 + label_bg_height
            text_y = y2 + text_height + 5
        
        # 检查右边界
        label_right = x1 + text_width
        x1_adjusted = x1
        if label_right > img_width:
            x1_adjusted = img_width - text_width
        
        # 绘制标签背景
        cv2.rectangle(
            img,
            (x1_adjusted, label_top),
            (min(label_right, img_width), label_bottom),
            color,
            -1
        )
        
        # 绘制标签文字
        cv2.putText(
            img,
            label,
            (x1_adjusted, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2
        )

    def draw_center_point(self, img, x1, y1, x2, y2, color):
        """绘制检测框的中心点并标注坐标
        
        在中心点绘制一个圆形标记和十字叉，并在旁边标注中心点的像素坐标
        
        Args:
            img: 输入图片
            x1, y1, x2, y2: 检测框坐标
            color: 标记颜色
        """
        img_height, img_width = img.shape[:2]
        
        # 计算中心点坐标
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        
        # 圆形标记半径
        circle_radius = 5
        
        # 绘制中心点圆形标记
        cv2.circle(img, (center_x, center_y), circle_radius, color, 2)
        
        # 绘制十字叉标记（增加识别度）
        cross_size = 8
        cv2.line(img, (center_x - cross_size, center_y), (center_x + cross_size, center_y), color, 2)
        cv2.line(img, (center_x, center_y - cross_size), (center_x, center_y + cross_size), color, 2)
        
        # 准备中心点坐标标注文本
        center_text = f"({center_x}, {center_y})"
        
        # 获取坐标文本尺寸
        (text_width, text_height), baseline = cv2.getTextSize(center_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
        
        # 坐标显示在中心点右下方，避免与其他标签重叠
        text_x = center_x + circle_radius + 5
        text_y = center_y + circle_radius + 5
        
        # 检查边界，防止文字超出图片范围
        if text_x + text_width > img_width:
            text_x = center_x - circle_radius - 5 - text_width
        if text_y + text_height > img_height:
            text_y = center_y - circle_radius - 5
        
        # 绘制坐标背景框（半透明效果，使用浅色背景便于阅读）
        bg_padding = 2
        cv2.rectangle(
            img,
            (text_x - bg_padding, text_y - text_height - bg_padding),
            (text_x + text_width + bg_padding, text_y + baseline + bg_padding),
            (0, 0, 0),
            -1
        )
        
        # 绘制坐标文字
        cv2.putText(
            img,
            center_text,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 0),  # 黄色文字便于与其他标签区分
            1
        )

    def get_depth_at_point(self, depth_img, x, y):
        """从深度图获取指定像素点的深度值
        
        深度图通常是16位无符号整数格式
        单位需要根据相机驱动确认（通常是毫米或100微米）
        
        Args:
            depth_img: 深度图像数据（numpy数组）
            x, y: 像素坐标
            
        Returns:
            depth_value: 深度原始值（单位待确认），如果无效则返回0
        """
        # 获取图像高度和宽度，防止坐标越界
        height, width = depth_img.shape[:2]
        
        # 边界检查
        if x < 0 or x >= width or y < 0 or y >= height:
            self.get_logger().warn(f"Depth point ({x}, {y}) out of bounds ({width}x{height})")
            return 0
        
        # 获取该点的深度值（深度图是16位整数）
        depth_value = depth_img[int(y), int(x)]
        
        # 检查深度值是否有效（0表示无效/无法识别）
        if depth_value == 0:
            self.get_logger().warn(f"Invalid depth value at ({x}, {y})")
        
        return depth_value

    def calculate_center_point_with_depth(self, box, depth_img):
        """计算检测框的中心点坐标和深度值
        
        从检测框计算中心点的像素坐标，并从深度图获取对应的深度值
        
        Args:
            box: YOLO检测框对象
            depth_img: 深度图像数据（numpy数组）
            
        Returns:
            tuple: (cx, cy, cz_raw, cz_meters)
                - cx: 中心点x坐标（像素）
                - cy: 中心点y坐标（像素）
                - cz_raw: 原始深度值
                - cz_meters: 深度值（单位：米）
        """
        # 获取检测框坐标
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        
        # 计算中心点坐标（像素坐标）
        cx = float((x1 + x2) / 2)
        cy = float((y1 + y2) / 2)
        
        # 从深度图获取中心点的深度值
        cz_raw = self.get_depth_at_point(depth_img, int(cx), int(cy))
        
        # 深度值单位转换：假设原始单位是毫米，转换为米
        cz_meters = float(cz_raw / 1000.0)
        
        return cx, cy, cz_raw, cz_meters

    def pixel_to_3d_camera_coords(self, pixel_x, pixel_y, depth_z):
        """将图像平面上的像素坐标和深度值转换为相机坐标系下的3D坐标
        
        这是一个关键方法，用于从2D图像空间转换到3D相机坐标空间。
        
        原理：
        ------
        图像上的像素点 (pixel_x, pixel_y) 与相机坐标系下的3D点 (X, Y, Z) 的关系为：
        
        相机坐标系定义：
            - 原点：在相机镜头的光学中心
            - X轴：向右
            - Y轴：向下
            - Z轴：向前（沿着相机光轴方向）
        
        反投影公式（从像素坐标到相机3D坐标）：
            X = (pixel_x - cx) * Z / fx
            Y = (pixel_y - cy) * Z / fy
            Z = depth_z
        
        其中相机内参为：
            - fx, fy: 焦距（像素单位），从相机标定得到
            - cx, cy: 主点（图像中心坐标，像素），从相机标定得到
            - depth_z: 深度值（从深度相机获取，单位：米）
        
        这个转换允许我们：
        1. 从2D检测结果（图像坐标 + 深度值）得到3D点云信息
        2. 在机器人坐标系中使用这些3D点进行操作（如抓取规划）
        3. 进行3D空间中的几何计算
        
        Args:
            pixel_x (float): 图像中的像素x坐标（从左到右，0~图像宽度）
            pixel_y (float): 图像中的像素y坐标（从上到下，0~图像高度）
            depth_z (float): 深度值（单位：米，从深度相机获取）
        
        Returns:
            dict: 包含3D坐标的字典，键值为：
                - 'X': 相机坐标系下的X坐标（米），正方向向右
                - 'Y': 相机坐标系下的Y坐标（米），正方向向下
                - 'Z': 相机坐标系下的Z坐标（米），正方向向前（沿光轴）
                - 'pixel_x': 原始输入的像素x坐标
                - 'pixel_y': 原始输入的像素y坐标
                - 'depth_z': 原始输入的深度值
        
        Raises:
            RuntimeError: 如果相机内参未被初始化（camera_info尚未接收）
            ValueError: 如果输入参数无效
        
        Examples:
            >>> # 将检测到的物体中心点转换为3D坐标
            >>> coords_3d = self.pixel_to_3d_camera_coords(320, 240, 1.5)
            >>> print(f"3D坐标: X={coords_3d['X']:.3f}m, Y={coords_3d['Y']:.3f}m, Z={coords_3d['Z']:.3f}m")
        
        Notes:
            - 此方法假设已收到 camera_info 消息并成功提取了内参
            - 深度值应该是从对齐后的深度相机获取
            - 相机坐标系与相机的RGB帧对齐
            - 如果内参未初始化，会抛出异常
        """
        # 参数验证
        if depth_z is None or depth_z <= 0:
            raise ValueError(f"深度值无效：{depth_z}。深度值必须大于0（单位：米）")
        
        if pixel_x is None or pixel_y is None:
            raise ValueError(f"像素坐标无效：({pixel_x}, {pixel_y})")
        
        # 检查相机内参是否已初始化
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            raise RuntimeError(
                "相机内参未初始化！请确保已接收到 camera_info 消息。"
                "检查 /ob_camera_head/depth/camera_info 话题是否正常发布。"
            )
        
        # ============ 反投影计算 ============
        # 基于针孔相机模型的反投影公式
        # 从像素坐标 (pixel_x, pixel_y) 和深度 depth_z 计算相机坐标系下的3D点 (X, Y, Z)
        
        # 计算X坐标（相机坐标系中向右为正）
        # X = (pixel_x - cx) * Z / fx
        X = (pixel_x - self.cx) * depth_z / self.fx
        
        # 计算Y坐标（相机坐标系中向下为正）
        # Y = (pixel_y - cy) * Z / fy
        Y = (pixel_y - self.cy) * depth_z / self.fy
        
        # Z坐标就是深度值本身（相机坐标系中沿光轴方向）
        Z = depth_z
        
        # 返回结果字典，包含3D坐标和输入参数（便于追踪）
        result = {
            'X': X,  # 米
            'Y': Y,  # 米
            'Z': Z,  # 米
            'pixel_x': pixel_x,  # 原始像素坐标
            'pixel_y': pixel_y,  # 原始像素坐标
            'depth_z': depth_z,  # 原始深度值
            'camera_params': {
                'fx': self.fx,
                'fy': self.fy,
                'cx': self.cx,
                'cy': self.cy
            }
        }
        
        return result
    

    def draw_depth_info_on_depth_image(self, img, center_x, center_y, depth_meters):
        """在深度图上绘制中心点标记和深度值
        
        在深度图的指定位置绘制中心点标记（圆形+十字）和深度值文字
        
        Args:
            img: 深度图（可视化后的彩色图）
            center_x: 中心点x坐标
            center_y: 中心点y坐标
            depth_meters: 深度值（单位：米）
        """
        # 在深度图的对应位置绘制圆形标记
        cv2.circle(img, (center_x, center_y), 8, (255, 255, 255), 2)
        
        # 绘制十字叉标记
        cross_size = 12
        cv2.line(img, (center_x - cross_size, center_y), 
                (center_x + cross_size, center_y), (255, 255, 255), 2)
        cv2.line(img, (center_x, center_y - cross_size), 
                (center_x, center_y + cross_size), (255, 255, 255), 2)
        
        # 准备深度值文本
        depth_text = f"Depth: {depth_meters:.3f}m"
        
        # 获取深度文本尺寸
        (depth_text_width, depth_text_height), depth_baseline = cv2.getTextSize(
            depth_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
        )
        
        # 深度文本显示在中心点右侧
        depth_text_x = center_x + 15
        depth_text_y = center_y
        
        # 边界检查
        img_height, img_width = img.shape[:2]
        if depth_text_x + depth_text_width > img_width:
            depth_text_x = center_x - 15 - depth_text_width
        
        # 绘制深度文本背景
        cv2.rectangle(
            img,
            (depth_text_x - 3, depth_text_y - depth_text_height - 3),
            (depth_text_x + depth_text_width + 3, depth_text_y + depth_baseline + 3),
            (0, 0, 0),
            -1
        )
        
        # 绘制深度文字
        cv2.putText(
            img,
            depth_text,
            (depth_text_x, depth_text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),  # 青色文字
            2
        )

    def draw_depth_label_on_color_image(self, img, center_x, center_y, depth_meters):
        """在彩色图上绘制深度值标签
        
        在中心点下方绘制深度值标签
        
        Args:
            img: 彩色图
            center_x: 中心点x坐标
            center_y: 中心点y坐标
            depth_meters: 深度值（单位：米）
        """
        # 在中心点下方显示深度值
        depth_label = f"{depth_meters:.3f}m"
        (dl_width, dl_height), dl_baseline = cv2.getTextSize(
            depth_label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
        )
        
        # 深度标签显示在中心点下方
        dl_x = center_x - dl_width // 2
        dl_y = center_y + 20
        
        # 边界检查
        img_height, img_width = img.shape[:2]
        if dl_x < 0:
            dl_x = 0
        if dl_x + dl_width > img_width:
            dl_x = img_width - dl_width
        
        # 绘制深度标签背景
        cv2.rectangle(
            img,
            (dl_x - 3, dl_y - dl_height - 3),
            (dl_x + dl_width + 3, dl_y + dl_baseline + 3),
            (0, 0, 0),
            -1
        )
        
        # 绘制深度标签文字
        cv2.putText(
            img,
            depth_label,
            (dl_x, dl_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),  # 青色文字
            2
        )

    def synchronized_image_cb(self, color_msg, depth_msg):
        """同步处理彩色图和深度图的回调函数
        
        该回调函数确保处理的彩色图和深度图来自同一时刻的传感器数据
        这正是帧同步的核心作用
        
        Args:
            color_msg: 彩色图像消息 (sensor_msgs/msg/Image)
            depth_msg: 深度图像消息 (sensor_msgs/msg/Image)
        """
        self.get_logger().info("synchronized_image detected")

        # ============ 消息转换 ============
        # 将ROS消息转换为OpenCV格式
        # 彩色图：BGR格式（OpenCV标准格式）
        color_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        
        # 深度图：使用 "passthrough" 保持原始编码格式
        # Orbbec 摄像头的深度图 encoding 可能是 "16UC1" 而不是 "mono16"
        # "passthrough" 表示不进行编码转换，直接传递原始数据
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        
        # 生成时间戳（同一帧使用相同时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

        # ============ 保存原始图像 ============
        # 1. 保存彩色原始图片
        # self.save_image(color_img, timestamp, suffix="raw_color")
        
        # 2. 保存深度原始图片（深度图需要进行可视化处理才能正确显示）
        # 深度图的像素值通常在0-5000mm范围内，需要归一化到0-255以便显示
        depth_normalized = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_TURBO)
        # self.save_image(depth_colored, timestamp, suffix="raw_depth")

        # ============ YOLO物体识别 ============
        # 在彩色图上进行物体检测
        results = self.model.predict(
            source=color_img,
            device="cpu",
            conf=0.4,
            verbose=False
        )

        if len(results[0].boxes) == 0:
            self.get_logger().info("No objects detected")
            return

        # ============ 目标类别过滤 ============
        # 如果指定了 target_class_ids，则只保留该类别的检测框
        # 
        # 说明：
        #   - 只修改 results[0].boxes，不需要修改 results[0].names
        #   - results[0].names 是模型的类别映射（{0: 'person', 1: 'bicycle', ...}），是静态的
        #   - 每个 box 的 box.cls 保存类别 ID，通过 results[0].names[class_id] 可以获取类别名称
        #   - 所以过滤 boxes 后，names 仍然能正确地为剩余的 boxes 查询类别名称
        #
        if self.target_class_ids:
            filtered_boxes = []
            for box in results[0].boxes:
                cls = int(box.cls[0].cpu().numpy())
                # 只保留类别 ID 在目标列表中的检测框
                if cls in self.target_class_ids:
                    filtered_boxes.append(box)
            
            # 如果没有找到目标类别的检测框，则记录日志并返回
            if not filtered_boxes:
                target_class_names = ', '.join([self.class_names[cid] for cid in self.target_class_ids])
                self.get_logger().info(f"未检测到目标物体: {target_class_names}（检测到其他物体）")
                return
            
            # 使用过滤后的检测框更新结果
            # 这样后续的绘制和处理代码只会处理目标类别的物体
            results[0].boxes = filtered_boxes
        
        # ============ 创建标注图像 ============
        # 创建彩色图的副本用于绘制标注
        annotated_img = color_img.copy()
        
        # 创建深度图的副本用于绘制标注（可视化版本）
        depth_annotated = depth_colored.copy()

        # 定义颜色列表（BGR格式）
        colors = [
            (0, 255, 0),    # 绿色
            (255, 0, 0),    # 蓝色
            (0, 0, 255),    # 红色
            (255, 255, 0),  # 青色
            (255, 0, 255),  # 品红
            (0, 255, 255),  # 黄色
        ]

        # ============ 遍历所有检测框 ============
        # 遍历每个检测到的物体
        for i, box in enumerate(results[0].boxes):
            # 获取检测框坐标
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # 获取类别和置信度
            cls = int(box.cls[0].cpu().numpy())
            conf = float(box.conf[0].cpu().numpy())
            label = f"{results[0].names[cls]} {conf:.2f}"
            
            # 选择颜色
            color = colors[i % len(colors)]
            
            # ============ 在彩色图上绘制检测框和标签 ============
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), color, 2)
            self.draw_label_with_background(annotated_img, label, x1, y1, x2, y2, color)
            self.draw_center_point(annotated_img, x1, y1, x2, y2, color)
            
            # ============ 计算中心点和深度值 ============
            center_x, center_y, depth_value, depth_meters = self.calculate_center_point_with_depth(box, depth_img)
            center_x_int = int(center_x)
            center_y_int = int(center_y)
            
            # ============ 在深度图和彩色图上绘制深度信息 ============
            self.draw_depth_info_on_depth_image(depth_annotated, center_x_int, center_y_int, depth_meters)
            self.draw_depth_label_on_color_image(annotated_img, center_x_int, center_y_int, depth_meters)
            
            # 输出日志
            self.get_logger().info(
                f"Object {i+1}: {label} | Center: ({center_x_int}, {center_y_int}) | Depth raw={depth_value} | Depth={depth_meters:.3f}m"
            )

        # ============ 保存标注后的图像 ============
        # 保存带标注的彩色图
        self.save_image(annotated_img, timestamp, suffix="detected_color")
        
        # 保存带标注的深度图
        # self.save_image(depth_annotated, timestamp, suffix="detected_depth")

        # ============ 发布抓取点（使用第一个检测框） ============
        # 取第一个检测框计算抓取点
        box = results[0].boxes[0]
        
        # 计算抓取点的中心坐标和深度值（像素空间）
        cx, cy, cz_raw, cz = self.calculate_center_point_with_depth(box, depth_img)

        # ============ 转换到相机坐标系下的3D坐标 ============
        # 将像素坐标 (cx, cy) 和深度值 cz 转换为真实的相机坐标系下的3D坐标
        # 这一步是关键：从2D图像空间 + 深度值 → 3D相机坐标空间
        try:
            coords_3d = self.pixel_to_3d_camera_coords(cx, cy, cz)
            X_3d = coords_3d['X']  # 相机坐标系 X（向右）
            Y_3d = coords_3d['Y']  # 相机坐标系 Y（向下）
            Z_3d = coords_3d['Z']  # 相机坐标系 Z（沿光轴向前）
            
            # 发布 Pose（包含真实的3D相机坐标）
            pose = Pose()
            pose.position.x = X_3d     # 单位：米，相机坐标系中向右为正
            pose.position.y = Y_3d     # 单位：米，相机坐标系中向下为正
            pose.position.z = Z_3d     # 单位：米，相机坐标系中沿光轴向前为正
            
            pose.orientation.w = 1.0  # 无旋转（单位四元数）
            
            self.pub.publish(pose)
            
            self.get_logger().info(
                f"✓ Grasp point (3D Camera Coords): X={X_3d:.3f}m, Y={Y_3d:.3f}m, Z={Z_3d:.3f}m "
                f"| Image pixel: ({cx:.1f}, {cy:.1f}) | Detected {len(results[0].boxes)} objects"
            )
        except RuntimeError as e:
            # 如果相机内参未初始化，降级到使用像素坐标
            self.get_logger().warn(
                f"无法进行3D坐标转换：{str(e)}。使用像素坐标作为替代。"
            )
            pose = Pose()
            pose.position.x = cx       # 像素 x（0~图像宽度）
            pose.position.y = cy       # 像素 y（0~图像高度）
            pose.position.z = cz       # 深度（单位：米）
            pose.orientation.w = 1.0
            
            self.pub.publish(pose)
            
            self.get_logger().info(
                f"Primary grasp point (Pixel coords): ({cx:.1f}, {cy:.1f}, {cz:.3f}m) "
                f"[raw_depth={cz_raw}] | Detected {len(results[0].boxes)} objects"
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
