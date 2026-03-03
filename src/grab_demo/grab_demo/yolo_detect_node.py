import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import shutil
import sys
from datetime import datetime
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
from tf2_geometry_msgs import PointStamped as TF2PointStamped
from tf2_ros import StaticTransformBroadcaster
from bodyctrl_msgs.msg import (
    CmdSetMotorPosition, SetMotorPosition,
    CmdMotorCtrl, MotorCtrl,
    CmdSetMotorSpeed, SetMotorSpeed
)
from std_msgs.msg import Header, String
from grab_demo_msgs.msg import GraspCandidate

# from tf_transformations import quaternion_from_euler
# import math

# ros2 run grab_demo yolo_detect_node --target_classes apple
# 由于使用 ApproximateTimeSynchronizer 对彩色图和深度图进行帧同步处理，所以只推荐运行在41.2的orin板上（也就是头部相机所连接的板）。
# 在41.1的x86板上运行则会出现无法进行同步的问题，彩色图和深度图的传输会大量无效占用带宽，导致 synchronized_image_cb 回调长时间无法被调用。

# 控制参数定义
VELOCITY_LIMIT = 0.2  # 速度限制（弧度/秒）
CURRENT_LIMIT = 5.0  # 电流限制（安培）

prepare_pose = {
    # 准备删除 #
    # "left_arm": [
    #     {11: 0.5, 12: 0.15, 13: 0.1, 14: -0.9, 15: 0.2, 16: 0.0, 17: 0.0},
    #     {11: 0.5, 12: 0.4, 13: 0.1, 14: -2.0, 15: 0.2, 16: 0.0, 17: 0.0},
    # ],
    # "right_arm": [
    #     {21: 0.5, 22: -0.15, 23: -0.1, 24: -0.9, 25: -0.2, 26: 0.0, 27: 0.0},
    #     {21: 0.5, 22: -0.4, 23: -0.1, 24: -2.0, 25: -0.2, 26: 0.0, 27: 0.0},
    # ],
    "head": [
        {1: 0.0, 2: 0.3, 3: 0.0},
    ]
}

left_arm_grasp_q = [-0.187, 0.647, -0.562, 0.481]
left_arm_grasp_offset = [-0.10, 0.13, -0.05]  # 左手从识别到的物体点加上这个偏移量后才是左手末端真正要到的坐标（单位：米）
right_arm_grasp_offset = [-0.10, -0.13, -0.04]  # 右手从识别到的物体点加上这个偏移量后才是右手末端真正要到的坐标（单位：米）

class YoloDetectNode(Node):
    def __init__(self):
        super().__init__("yolo_detect_node")
        
        # ============ 按功能分类进行初始化 ============
        self._init_head_pose()               # 头部电机位置调整
        self._init_basic_components()        # 基础组件初始化
        self._init_yolo_model()              # YOLO 模型加载
        self._init_target_classes()          # 目标检测类别配置
        self._init_image_synchronizer()      # 图像同步器配置
        self._init_camera_parameters()       # 相机参数初始化
        self._init_tf_transforms()           # TF2 坐标系变换配置
        self._init_grasp_candidate_pub()     # 抢取候选点发布者

        self.get_logger().info(f"✓ YoloDetectNode 初始化完成，正在等待包含{self.target_classes}的图像数据...")

    def _init_head_pose(self):
        self.head_pos_cmd_publisher = self.create_publisher(CmdSetMotorPosition, '/head/cmd_pos', 10)
        self.get_logger().info("✓ 头部电机位置模式控制发布者已创建（话题：/head/cmd_pos）")
        time.sleep(0.5)  # 确保发布者初始化完成
        self.control_head_pos()

    def create_header_for_head_motor(self):
        """
        创建消息头
        
        返回值：
            Header: 包含时间戳和坐标系信息的消息头
            
        说明：
            所有命令消息都需要包含header信息，用于时间同步和坐标系标识
        """
        # 获取当前时间（秒和纳秒）
        now = self.get_clock().now()
        
        # 创建Header对象
        header = Header()
        header.stamp.sec = int(now.nanoseconds // 1_000_000_000)  # 转换为秒
        header.stamp.nanosec = int(now.nanoseconds % 1_000_000_000)  # 剩余纳秒
        header.frame_id = 'head'  # 坐标系ID
        
        return header

    def control_head_pos(self):
        """
        头部电机位置调整，以便让头部相机可顺利看到正前方的桌面上的物体
        """
        self.get_logger().info("")
        self.get_logger().info("=" * 50)
        self.get_logger().info("【头部电机位置调整】开始执行")
        self.get_logger().info("=" * 50)
        
        for pose in prepare_pose['head']:
            # 创建消息头
            header = self.create_header_for_head_motor()
            
            # 创建位置模式命令消息
            msg = CmdSetMotorPosition()
            msg.header = header
            
            # 为每个电机创建回零命令
            for motor_id, position in pose.items():
                # 创建单个电机的位置命令
                cmd = SetMotorPosition()
                cmd.name = motor_id  # 电机ID
                cmd.pos = position  # 目标位置
                cmd.spd = VELOCITY_LIMIT  # 速度限制（弧度/秒）
                cmd.cur = CURRENT_LIMIT  # 电流限制（安培）
                
                # 添加到消息数组
                msg.cmds.append(cmd)
                
                # self.get_logger().info(f"  电机 {motor_id}：运动到位置（{position} rad）")
            
            # 发送命令
            self.head_pos_cmd_publisher.publish(msg)
            self.get_logger().info("✓ 头部电机位置调整命令已发送")
            time.sleep(1.5)  # 给电机足够的时间运动到位

    def _init_basic_components(self):
        """初始化基础组件
        
        包括：
        - 发布静态 TF 变换
        - 初始化 CvBridge
        - 创建保存目录
        """
        # 发布静态TF变换
        # self.publish_static_transform()
        
        # 初始化 CvBridge，用于ROS图像消息与OpenCV格式的转换
        self.bridge = CvBridge()
        
        # 创建保存目录
        self.save_dir = "saved_data/grab_node"
        if os.path.exists(self.save_dir):
            shutil.rmtree(self.save_dir)
        os.makedirs(self.save_dir)

    def _init_yolo_model(self):
        """初始化 YOLO 模型和推理设置
        
        包括：
        - 选择推理设备（GPU 或 CPU）
        - 加载 YOLO 模型
        - GPU 预热
        
        性能对比（YOLOv8n，Jetson Orin）：
          - CPU: 150-300ms/frame (3-7 FPS)
          - GPU: 25-50ms/frame (20-40 FPS)
          - 加速比: 4-6倍
        """
        # 选择推理设备：GPU 或 CPU
        # 对于 Jetson Orin，强烈推荐使用 GPU（device=0），性能提升 4-6 倍
        # 设备选项：
        #   - device=0 或 device="cuda": 使用第一块GPU（Jetson Orin内置GPU）
        #   - device="cpu": 使用CPU（较慢）
        #   - device="cuda:0": 显式指定第一块GPU
        self.device = "cuda"  # 改为 "cpu" 可切换到CPU推理
        self.model = YOLO("yolo_models/yolov8n.pt")
        
        # GPU 预热：第一次GPU推理会有初始化开销（~500ms）
        # 预热可以避免第一帧检测时的长延迟
        if self.device == "cuda":
            try:
                dummy_input = np.zeros((480, 640, 3), dtype=np.uint8)
                self.model.predict(
                    source=dummy_input,
                    device=self.device,
                    conf=0.5,
                    verbose=False
                )
                self.get_logger().info("✓ GPU 预热完成")
            except Exception as e:
                self.get_logger().warn(f"GPU预热失败: {str(e)}，但不影响后续推理")

    def _init_target_classes(self):
        """初始化目标检测类别配置
        
        从命令行参数解析目标类别，建立类别名称到 ID 的映射
        
        使用方法：
            python3 -m yolo_detect_node --target_classes apple
            python3 -m yolo_detect_node --target_classes orange
            python3 -m yolo_detect_node  # 使用默认值 ['apple']
        
        可用的类别列表（YOLO v8n 支持的 COCO 数据集类别，共 80 类）：
          person, bicycle, car, motorbike, aeroplane, bus, train, truck,
          boat, traffic light, fire hydrant, stop sign, parking meter, bench,
          cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe, backpack,
          umbrella, handbag, tie, suitcase, frisbee, skis, snowboard, sports ball,
          kite, baseball bat, baseball glove, skateboard, surfboard, tennis racket,
          bottle, wine glass, cup, fork, knife, spoon, bowl, banana, apple, sandwich,
          orange, broccoli, carrot, hot dog, pizza, donut, cake, chair, couch,
          potted plant, bed, dining table, toilet, tv, laptop, mouse, remote,
          keyboard, microwave, oven, toaster, sink, refrigerator, book, clock,
          vase, scissors, teddy bear, hair drier, toothbrush
        """
        # 从命令行参数解析目标类别
        target_classes = self._parse_target_classes_from_args()
        
        # 获取模型的所有类别名称（字典：类别ID -> 类别名称）
        self.class_names = self.model.names
        
        # 构建类别名称到 ID 的映射（小写以支持大小写不敏感的搜索）
        self.class_id_map = {v.lower(): k for k, v in self.class_names.items()}
        
        # 将指定的目标类别名称转换为 YOLO 类别 ID
        self.target_class_ids = []
        if target_classes and target_classes != ['all']:
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
        
        # 日志输出过滤状态
        if self.target_class_ids:
            self.get_logger().info(f"类别过滤已启用，检测目标: {target_classes}")
            self.target_classes = target_classes
        else:
            self.get_logger().info("类别过滤已禁用，检测所有物体")

    def _init_image_synchronizer(self):
        """初始化图像同步器
        
        使用 message_filters 进行彩色图和深度图的帧同步
        ApproximateTimeSynchronizer: 近似时间同步器
        功能：同步来自不同话题的消息，允许时间戳有小偏差
        应用场景：不同传感器采样频率和发布时间不同，时间戳有轻微偏差
        """
        # 创建彩色图像订阅器
        color_sub = Subscriber(self, Image, "/ob_camera_head/color/image_raw")
        
        # 创建深度图像订阅器
        depth_sub = Subscriber(self, Image, "/ob_camera_head/depth/image_raw")
        
        # 创建同步器：queue_size=10表示缓冲区大小，slop=0.1表示允许100ms的时间差
        self.ts = ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=10,
            slop=0.1
        )
        
        # 注册同步回调函数
        self.ts.registerCallback(self.synchronized_image_cb)
        
        # 抓取点发布者
        self.grasp_pose_pub = self.create_publisher(PoseStamped, "/grasp_pose", 10)
        self.object_pose_pub = self.create_publisher(PoseStamped, "/object_pose", 10)
        
        # 创建 /gui/joint_command 发布者（用于向GUI发送关节命令）
        # 消息格式：JSON字符串，如 {"11": 0.5, "12": 0.15, ...}
        self.joint_command_pub = self.create_publisher(String, "/gui/joint_command", 10)
        self.get_logger().info("✓ GUI关节命令发布者已创建（话题：/gui/joint_command）")
        
        # 存储最新的深度数据（用于在回调中访问）
        self.latest_depth_img = None
        self.latest_color_timestamp = None

    def _init_camera_parameters(self):
        """初始化相机参数
        
        包括：
        - 订阅深度相机的 camera_info 话题
        - 初始化相机内参矩阵相关变量
        
        相机内参矩阵的结构：
          K = [[fx,  0, cx],
               [ 0, fy, cy],
               [ 0,  0,  1]]
        其中：
          - fx, fy: 焦距（像素单位）
          - cx, cy: 主点坐标（图像中心，像素坐标）
        """
        # 订阅深度相机的 camera_info 话题
        # camera_info 包含相机的内参矩阵、畸变系数等
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/ob_camera_head/depth/camera_info",
            self.depth_camera_info_cb,
            10
        )
        
        # 标志位：记录是否已获取过camera_info
        self.camera_info_received = False
        self.depth_unit = None  # 深度单位
        
        # 相机内参矩阵（用于像素坐标到3D坐标的反投影）
        self.camera_matrix_K = None  # 相机内参矩阵（3x3）
        self.fx = None  # 焦距 x
        self.fy = None  # 焦距 y
        self.cx = None  # 主点 x
        self.cy = None  # 主点 y

    def _init_tf_transforms(self):
        """初始化 TF2 坐标系变换配置
        
        用于将相机坐标系下的3D点变换到机器人基座坐标系
        """
        # TF2 坐标系变换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 相机光学坐标系（OpenCV和深度相机的标准坐标系）
        self.ob_camera_frame = "ob_camera_head_color_frame"
        
        # 目标基座坐标系（机器人抓取基准）
        # 用户可通过命令行参数 --target_frame 指定
        self.target_frame = self._parse_target_frame_from_args()
        
        self.get_logger().info(f"相机光学坐标系: {self.ob_camera_frame}")
        self.get_logger().info(f"目标基座坐标系: {self.target_frame}")
    
    def _init_grasp_candidate_pub(self):
        """初始化抓取候选点发布者，向执行节点发送检测结果"""
        self.grasp_candidate_pub = self.create_publisher(
            GraspCandidate, '/grasp_candidate', 10
        )
        # self.get_logger().info("✓ 抓取候选点发布者已创建（/grasp_candidate）")
   
    def _parse_target_classes_from_args(self):
        """
        从命令行参数解析目标类别
        
        使用方法：
            python3 -m yolo_detect_node --target_classes apple
            python3 -m yolo_detect_node --target_classes apple orange
            python3 -m yolo_detect_node --target_classes person car bicycle
            python3 -m yolo_detect_node  # 使用默认值 ['apple']
        
        Returns:
            list: 目标类别列表，如 ['apple', 'orange']，或默认值 ['apple']
        """
        target_classes = ['bottle']  # 默认值：检测瓶子
        
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
                    self.get_logger().info(f"[INFO] 从命令行解析到目标类别: {target_classes}")
                else:
                    self.get_logger().info(f"[WARN] --target_classes 后没有指定类别，使用默认值: {target_classes}")
            except Exception as e:
                self.get_logger().error(f"[ERROR] 解析命令行参数失败: {e}，使用默认值: {target_classes}")
        else:
            self.get_logger().info(f"[INFO] 未指定 --target_classes 参数，使用默认值: {target_classes}")
            self.get_logger().info(f"[INFO] 使用方法: python3 -m grab_demo.yolo_detect_node --target_classes apple orange")
        
        return target_classes

    def _parse_target_frame_from_args(self):
        """
        从命令行参数解析目标坐标系
        
        用于指定抓取操作的基准坐标系。对于天工是 pelvis
        
        使用方法：
            ros2 run grab_demo yolo_detect_node
            ros2 run grab_demo yolo_detect_node
        
        Returns:
            str: 目标基座坐标系名称，默认为 "pelvis"
        """
        target_frame = "pelvis"
        
        if '--target_frame' in sys.argv:
            try:
                idx = sys.argv.index('--target_frame')
                if idx + 1 < len(sys.argv):
                    frame = sys.argv[idx + 1]
                    if not frame.startswith('--'):
                        target_frame = frame
                        self.get_logger().info(f"[INFO] 从命令行解析到目标坐标系: {target_frame}")
            except Exception as e:
                self.get_logger().info(f"[ERROR] 解析目标坐标系参数失败: {e}，使用默认值: {target_frame}")
        else:
            self.get_logger().info(f"[INFO] 未指定 --target_frame 参数，使用默认值: {target_frame}")
            # print(f"[INFO] 使用方法: ros2 run grab_demo yolo_detect_node")
        
        return target_frame

    def _cleanup_old_files(self, max_files=20):
        """清理目录内的旧文件，保持文件数不超过指定数量
        
        当目录内文件数超过 max_files 时，删除最旧的文件
        
        Args:
            max_files (int): 目录内最多保留的文件数，默认20个
        """
        try:
            # 获取目录内所有文件
            files = [f for f in os.listdir(self.save_dir) 
                     if os.path.isfile(os.path.join(self.save_dir, f))]
            
            # 如果文件数超过限制，删除最老的文件
            if len(files) > max_files:
                # 按修改时间排序，最旧的在前面
                files_with_time = [(f, os.path.getmtime(os.path.join(self.save_dir, f))) 
                                   for f in files]
                files_with_time.sort(key=lambda x: x[1])  # 按时间升序排列
                
                # 需要删除的文件数
                num_to_delete = len(files) - max_files
                
                for i in range(num_to_delete):
                    old_file = files_with_time[i][0]
                    old_filepath = os.path.join(self.save_dir, old_file)
                    try:
                        os.remove(old_filepath)
                        self.get_logger().debug(f"Deleted old file: {old_file}")
                    except Exception as e:
                        self.get_logger().warn(f"Failed to delete {old_file}: {str(e)}")
                
                # self.get_logger().debug(f"Cleaned up {num_to_delete} old file(s), keeping latest {max_files} files")
        except Exception as e:
            self.get_logger().warn(f"Error during cleanup: {str(e)}")

    def save_image(self, img, timestamp, suffix="image"):
        """保存图片到指定目录
        
        在保存前检查目录文件数量，超过20个则删除最旧的文件
        """
        # 清理旧文件（保持不超过20个）
        self._cleanup_old_files(max_files=20)
        
        filename = f"{timestamp}_{suffix}.jpg"
        filepath = os.path.join(self.save_dir, filename)
        cv2.imwrite(filepath, img)
        # self.get_logger().info(f"Saved: {filename}")
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
        
        # self.get_logger().info("=" * 60)
        # self.get_logger().info("深度相机校准信息 (Depth Camera Info):")
        # self.get_logger().info("=" * 60)
        
        # # 打印基本信息
        # self.get_logger().info(f"相机名称 (Camera Name): {msg.header.frame_id}")
        # self.get_logger().info(f"图像分辨率 (Image Size): {msg.width} x {msg.height}")
        
        # # 打印内参矩阵（K矩阵）
        # self.get_logger().info("相机内参矩阵 (Camera Matrix K):")
        # K = np.array(msg.k).reshape(3, 3)
        # self.get_logger().info(f"\n{K}")
        
        # 打印畸变系数
        # self.get_logger().info(f"畸变系数 (Distortion Coefficients): {msg.d}")
        
        # ============ 提取并保存相机内参矩阵 ============
        # 从 camera_info 消息中提取内参矩阵 K
        # 这些参数用于将像素坐标反投影到相机坐标系下的3D坐标
        self.camera_matrix_K = np.array(msg.k).reshape(3, 3)
        self.fx = self.camera_matrix_K[0, 0]  # 焦距 x
        self.fy = self.camera_matrix_K[1, 1]  # 焦距 y
        self.cx = self.camera_matrix_K[0, 2]  # 主点 x
        self.cy = self.camera_matrix_K[1, 2]  # 主点 y
        
        # self.get_logger().info("\n" + "="*60)
        # self.get_logger().info("相机内参矩阵参数已保存：")
        # self.get_logger().info(f"  焦距 fx: {self.fx:.2f}")
        # self.get_logger().info(f"  焦距 fy: {self.fy:.2f}")
        # self.get_logger().info(f"  主点 cx: {self.cx:.2f}")
        # self.get_logger().info(f"  主点 cy: {self.cy:.2f}")
        # self.get_logger().info("这些参数用于像素坐标到3D相机坐标系的反投影转换")
        # self.get_logger().info("="*60 + "\n")
        
        # 检查是否有特殊标记表示深度单位
        # Orbbec SDK 某些版本在 camera_info 的其他字段中标记深度单位
        # 常见的有：
        # - binning_x 可能存储深度缩放因子
        # - binning_y 可能存储其他信息
        # self.get_logger().info(f"Binning X (可能包含深度缩放信息): {msg.binning_x}")
        # self.get_logger().info(f"Binning Y: {msg.binning_y}")
        
        # ROI (Region of Interest) 信息
        # self.get_logger().info(f"ROI: {msg.roi}")
        
        # self.get_logger().info("=" * 60)
        # self.get_logger().info("深度单位确认方法:")
        # self.get_logger().info("1. 查看 Orbbec SDK 文档或ROS驱动说明")
        # self.get_logger().info("2. 查看日志中实际深度值范围 (下面的YOLO回调会输出)")
        # self.get_logger().info("3. 通过对已知距离物体的测量来校准")
        # self.get_logger().info("=" * 60)

    def filter_detections_by_target_classes(self, results):
        """根据目标类别过滤检测框
        
        如果指定了 target_class_ids，则从 results[0].boxes 中过滤出该类别的检测框
        同时统计其他物体的数量并记录日志
        
        Args:
            results: YOLO 检测结果，包含 results[0].boxes 和 results[0].names
        
        Returns:
            bool: 是否成功保留了目标类别的检测框
                  - True: 保留了目标物体，results[0].boxes 已更新
                  - False: 未找到目标物体，results[0].boxes 未修改，应该忽略本帧
        
        Side Effects:
            - 直接修改 results[0].boxes，只保留目标类别的检测框
            - 记录日志信息
        """
        # 如果未指定目标类别，则不进行过滤，接受所有检测框
        if not self.target_class_ids:
            return True
        
        # ============ 遍历所有检测框，按类别分类 ============
        filtered_boxes = []  # 符合目标类别的检测框
        other_objects = {}   # 其他物体的统计信息 {类别名: 数量}
        
        for box in results[0].boxes:
            cls = int(box.cls[0].cpu().numpy())
            class_name = self.class_names[cls]
            
            # 按类别决定是否保留该检测框
            if cls in self.target_class_ids:
                filtered_boxes.append(box)
            else:
                # 统计未被过滤的物体及其数量
                other_objects[class_name] = other_objects.get(class_name, 0) + 1
        
        # ============ 检查是否找到目标类别 ============
        if not filtered_boxes:
            # 未找到目标类别，生成日志信息
            target_class_names = ', '.join([self.class_names[cid] for cid in self.target_class_ids])
            other_objects_str = ', '.join([f"{name}({count})" for name, count in other_objects.items()])
            
            # self.get_logger().info(f"未检测到目标物体: {target_class_names} | 检测到其他物体: {other_objects_str}")
            
            # 返回False，表示过滤失败，调用方应忽略本帧
            return False
        
        # ============ 更新检测框列表 ============
        # 直接修改 results[0].boxes，只保留过滤后的检测框
        # 注意：results[0].names 保持不变，仍是模型的完整类别映射
        results[0].boxes = filtered_boxes
        
        # 返回True，表示过滤成功，检测框列表已更新
        return True

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
        # if depth_value == 0:
        #     self.get_logger().warn(f"Invalid depth value at ({x}, {y})")
        
        return depth_value

    def calculate_center_point_with_depth(self, box, depth_img):
        """计算检测框的中心点坐标和深度值
        
        从检测框计算中心点的像素坐标，并从深度图获取对应的深度值
        如果中心点深度无效，则尝试采样周围区域求平均值
        
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
        
        # 如果中心点深度无效，尝试采样周围3x3区域求平均值
        if cz_raw == 0:
            self.get_logger().info(f"⚠ 中心点深度无效，尝试采样周围区域...")
            valid_depths = []
            
            # 采样3x3区域
            for dy in [-2, 0, 2]:
                for dx in [-2, 0, 2]:
                    sample_x = int(cx + dx)
                    sample_y = int(cy + dy)
                    depth = self.get_depth_at_point(depth_img, sample_x, sample_y)
                    if depth > 0:
                        valid_depths.append(depth)
            
            # 如果找到有效深度值，使用中位数
            if valid_depths:
                cz_raw = np.median(valid_depths)
                self.get_logger().info(f"✓ 使用周围{len(valid_depths)}个有效深度值的中位数: {cz_raw}")
            else:
                self.get_logger().warn(f"✗ 周围区域也没有有效深度值")
        
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
                - 'X': 相机坐标系下的X坐标（米），正方向向右（如果valid=True）
                - 'Y': 相机坐标系下的Y坐标（米），正方向向下（如果valid=True）
                - 'Z': 相机坐标系下的Z坐标（米），正方向向前（如果valid=True）
                - 'valid': 布尔值，表示数据是否有效（True=成功，False=失败）
                - 'error': 如果valid=False，包含错误信息；否则为None
                - 'pixel_x': 原始输入的像素x坐标
                - 'pixel_y': 原始输入的像素y坐标
                - 'depth_z': 原始输入的深度值
        
        Notes:
            - 深度值无效（≤0）时不抛异常，返回valid=False
            - 相机内参未初始化时不抛异常，返回valid=False
            - 调用方应检查返回值的'valid'字段，如果为False则忽略本次数据
        
        Notes:
            - 此方法假设已收到 camera_info 消息并成功提取了内参
            - 深度值应该是从对齐后的深度相机获取
            - 相机坐标系与相机的RGB帧对齐
            - 如果内参未初始化，会抛出异常
        """
        # 参数验证 - 返回标记而不是抛异常，让调用者决定是否继续
        if depth_z is None or depth_z <= 0:
            return {
                'X': None, 'Y': None, 'Z': None,
                'valid': False,
                'error': f"深度值无效：{depth_z}。深度值必须大于0（单位：米）",
                'pixel_x': pixel_x, 'pixel_y': pixel_y, 'depth_z': depth_z
            }
        
        if pixel_x is None or pixel_y is None:
            return {
                'X': None, 'Y': None, 'Z': None,
                'valid': False,
                'error': f"像素坐标无效：({pixel_x}, {pixel_y})",
                'pixel_x': pixel_x, 'pixel_y': pixel_y, 'depth_z': depth_z
            }
        
        # 检查相机内参是否已初始化
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return {
                'X': None, 'Y': None, 'Z': None,
                'valid': False,
                'error': "相机内参未初始化！请确保已接收到 camera_info 消息。检查 /ob_camera_head/depth/camera_info 话题是否正常发布。",
                'pixel_x': pixel_x, 'pixel_y': pixel_y, 'depth_z': depth_z
            }
        
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
            'valid': True,  # 标记数据有效
            'error': None,
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

    def transform_point_to_target_frame(self, point_camera, target_frame=None):
        """将相机坐标系下的3D点变换到目标坐标系
        
        这是一个关键方法，用于将相机光学坐标系下的3D坐标变换到机器人基座坐标系
        以便进行抓取规划和执行。
        
        坐标系变换流程：
        ----------------
        1. 输入：相机光学坐标系下的3D点 (X_cam, Y_cam, Z_cam)
           - X轴向右，Y轴向下，Z轴向前（OpenCV标准）
           - 原点在相机镜头的光学中心
        
        2. TF2变换：通过查询TF树获得相机坐标系到目标坐标系的变换矩阵
           - 从 ob_camera_head_color_frame 到 pelvis 的变换
           - 该变换包含相对位置和相对旋转
        
        3. 输出：目标坐标系下的3D点 (X_target, Y_target, Z_target)
           - 是 pelvis 坐标系下的坐标
               
        Args:
            point_camera (dict): 相机坐标系下的3D点
                包含键值：'X', 'Y', 'Z'（单位：米）
                通常来自 pixel_to_3d_camera_coords() 的返回值
            
            target_frame (str): 目标坐标系名称
                如果为None，则使用初始化时指定的 self.target_frame，值为 pelvis
        
        Returns:
            dict: 目标坐标系下的3D点，包含：
                - 'X', 'Y', 'Z': 目标坐标系下的坐标（米）
                - 'frame_id': 目标坐标系名称
                - 'camera_frame': 相机坐标系名称
                - 'success': 变换是否成功
                - 'error': 如果失败，包含错误信息
        
        Raises:
            ValueError: 如果输入点无效
            RuntimeError: 如果TF2变换查询失败
        
        Notes:
            - TF2需要维护完整的坐标系树，确保从相机坐标系到目标坐标系有变换路径
            - 如果变换不可用，会返回错误信息而不是抛出异常（便于调试）
            - 确保ROS2系统正确发布了 /tf 和 /tf_static 话题
        """
        # 参数验证
        if not isinstance(point_camera, dict) or 'X' not in point_camera or 'Y' not in point_camera or 'Z' not in point_camera:
            raise ValueError(f"输入点格式无效，必须包含 'X', 'Y', 'Z' 键")
        
        if target_frame is None:
            target_frame = self.target_frame
        
        try:
            # ============ 步骤1：创建相机坐标系下的点消息 ============
            # 将3D点转换为ROS PointStamped 消息格式
            # 这个消息包含坐标值和坐标系信息，是TF2变换的输入格式
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.ob_camera_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = float(point_camera['X'])
            point_stamped.point.y = float(point_camera['Y'])
            point_stamped.point.z = float(point_camera['Z'])
            
            # ============ 步骤2：查询TF变换 ============
            # 从TF缓冲区查询从相机坐标系到目标坐标系的变换
            # 使用较长的超时时间（8秒）以应对启动时的TF缓冲区延迟
            # 节点刚启动时，TF树需要时间来发布和缓冲变换数据
            # 通常几秒后就会正常，但新连接可能需要等待
            try:
                # 尝试查询最新的变换（超时8秒）
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    self.ob_camera_frame,
                    rclpy.time.Time(),  # 查询最新的变换
                    timeout=rclpy.duration.Duration(seconds=8.0)  # 等待
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
                # 如果最新时间戳失败，尝试查询最近可用的（通常在缓冲区内）
                # 这对于刚启动的节点很有效
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    self.ob_camera_frame,
                    rclpy.time.Time(seconds=0),  # 查询最近可用的任何时间戳
                    timeout=rclpy.duration.Duration(seconds=5.0)
                )
            
            # ============ 步骤3：执行坐标系变换 ============
            # 使用TF2库进行齐次变换
            # do_transform_point() 接收点和变换，返回变换后的点
            from tf2_geometry_msgs import do_transform_point
            point_transformed = do_transform_point(point_stamped, transform)
            
            # ============ 步骤4：提取变换后的坐标 ============
            result = {
                'X': float(point_transformed.point.x),
                'Y': float(point_transformed.point.y),
                'Z': float(point_transformed.point.z),
                'frame_id': target_frame,
                'camera_frame': self.ob_camera_frame,
                'success': True,
                'error': None,
                'original_camera_coords': {
                    'X': point_camera['X'],
                    'Y': point_camera['Y'],
                    'Z': point_camera['Z']
                }
            }
            
            return result
        
        except tf2_ros.LookupException as e:
            # 变换不存在（坐标系树中没有连接路径）
            self.get_logger().error(
                f"TF查询失败：无法找到从 {self.ob_camera_frame} 到 {target_frame} 的变换。"
                f"原因: {str(e)}"
            )
            return {
                'success': False,
                'error': f"LookupException: {str(e)}",
                'frame_id': target_frame,
                'camera_frame': self.ob_camera_frame,
                'X': None, 'Y': None, 'Z': None
            }
        
        except tf2_ros.ConnectivityException as e:
            # TF树连接问题
            self.get_logger().error(
                f"TF连接错误：无法连接到TF服务。"
                f"原因: {str(e)}"
            )
            return {
                'success': False,
                'error': f"ConnectivityException: {str(e)}",
                'frame_id': target_frame,
                'camera_frame': self.ob_camera_frame,
                'X': None, 'Y': None, 'Z': None
            }
        
        except tf2_ros.ExtrapolationException as e:
            # 时间戳超出有效范围
            self.get_logger().error(
                f"TF时间戳错误：请求的时间戳超出有效范围。"
                f"原因: {str(e)}"
            )
            return {
                'success': False,
                'error': f"ExtrapolationException: {str(e)}",
                'frame_id': target_frame,
                'camera_frame': self.ob_camera_frame,
                'X': None, 'Y': None, 'Z': None
            }
        
        except Exception as e:
            # 其他未预期的错误
            self.get_logger().error(f"坐标系变换发生未预期的错误: {str(e)}")
            return {
                'success': False,
                'error': f"UnexpectedException: {str(e)}",
                'frame_id': target_frame,
                'camera_frame': self.ob_camera_frame,
                'X': None, 'Y': None, 'Z': None
            }

    def get_axis_symmetric_quaternion(self, quaternion, axis: str = 'y'):
        """计算给定位姿四元数关于指定轴对称后的四元数。

        参数:
            quaternion: 输入四元数，支持以下形式：
                - [x, y, z, w] / (x, y, z, w)
                - {'x':..., 'y':..., 'z':..., 'w':...}
                - 具有 x/y/z/w 属性的对象
            axis: 对称轴，可选 'x'/'y'/'z'，默认 'y'

        返回:
            list[float]: 对称后的四元数 [x, y, z, w]
        """
        def _parse_quaternion(q):
            if isinstance(q, (list, tuple)) and len(q) == 4:
                qx, qy, qz, qw = map(float, q)
                return qx, qy, qz, qw
            if isinstance(q, dict) and all(k in q for k in ('x', 'y', 'z', 'w')):
                return float(q['x']), float(q['y']), float(q['z']), float(q['w'])
            if all(hasattr(q, k) for k in ('x', 'y', 'z', 'w')):
                return float(q.x), float(q.y), float(q.z), float(q.w)
            raise ValueError('quaternion 格式无效，应为 [x,y,z,w] / dict / 含 x,y,z,w 属性对象')

        def _quat_to_rot(qx, qy, qz, qw):
            norm = np.linalg.norm([qx, qy, qz, qw])
            if norm == 0:
                raise ValueError('四元数范数为0，无法计算')
            qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

            return np.array([
                [1 - 2 * (qy * qy + qz * qz),     2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
                [    2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),     2 * (qy * qz - qx * qw)],
                [    2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ], dtype=np.float64)

        def _rot_to_quat(rot):
            trace = float(np.trace(rot))
            if trace > 0.0:
                s = np.sqrt(trace + 1.0) * 2.0
                qw = 0.25 * s
                qx = (rot[2, 1] - rot[1, 2]) / s
                qy = (rot[0, 2] - rot[2, 0]) / s
                qz = (rot[1, 0] - rot[0, 1]) / s
            else:
                if rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
                    s = np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
                    qw = (rot[2, 1] - rot[1, 2]) / s
                    qx = 0.25 * s
                    qy = (rot[0, 1] + rot[1, 0]) / s
                    qz = (rot[0, 2] + rot[2, 0]) / s
                elif rot[1, 1] > rot[2, 2]:
                    s = np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
                    qw = (rot[0, 2] - rot[2, 0]) / s
                    qx = (rot[0, 1] + rot[1, 0]) / s
                    qy = 0.25 * s
                    qz = (rot[1, 2] + rot[2, 1]) / s
                else:
                    s = np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
                    qw = (rot[1, 0] - rot[0, 1]) / s
                    qx = (rot[0, 2] + rot[2, 0]) / s
                    qy = (rot[1, 2] + rot[2, 1]) / s
                    qz = 0.25 * s

            q = np.array([qx, qy, qz, qw], dtype=np.float64)
            q /= np.linalg.norm(q)
            return q.tolist()

        axis = axis.lower()
        symmetry_matrix_map = {
            'x': np.diag([1.0, -1.0, 1.0]),
            'y': np.diag([-1.0, 1.0, 1.0]),
            'z': np.diag([1.0, 1.0, -1.0]),
        }
        if axis not in symmetry_matrix_map:
            raise ValueError("axis 仅支持 'x'/'y'/'z'")

        qx, qy, qz, qw = _parse_quaternion(quaternion)
        rot = _quat_to_rot(qx, qy, qz, qw)
        symmetry = symmetry_matrix_map[axis]

        # 对称后的旋转矩阵：R' = S * R * S
        rot_sym = symmetry @ rot @ symmetry
        return _rot_to_quat(rot_sym)
    
    def publish_grasp_pose(self, coords_3d, pixel_coords, num_detections):
        """发布抓取点的位姿信息
        
        将相机坐标系下的3D点变换到目标基座坐标系并发布为PoseStamped消息
        如果变换失败则降级发布相机坐标系下的坐标
        
        Args:
            coords_3d (dict): 相机坐标系下的3D坐标，包含 'X', 'Y', 'Z' 键
            pixel_coords (tuple): 像素坐标和深度值 (cx, cy, cz_raw, cz_meters)
            num_detections (int): 检测到的物体数量
        """
        cx, cy, cz_raw, cz = pixel_coords
        X_cam, Y_cam, Z_cam = coords_3d['X'], coords_3d['Y'], coords_3d['Z']
        
        # self.get_logger().info(
        #     f"✓ 抓取点在相机坐标系的坐标: X={X_cam:.3f}m, Y={Y_cam:.3f}m, Z={Z_cam:.3f}m "
        #     f"| 像素坐标: ({cx:.1f}, {cy:.1f})"
        # )
        
        # 变换到目标基座坐标系
        coords_base = self.transform_point_to_target_frame(coords_3d)
        
        # 创建PoseStamped消息
        object_pose = PoseStamped()
        object_pose.header.stamp = self.get_clock().now().to_msg()
        # pose_stamped.pose.orientation.w = 1.0  # 无旋转
                
        
        if coords_base['success']:
            # 变换成功：使用基座坐标系
            object_pose.header.frame_id = coords_base['frame_id']
            # pose_stamped.pose.position.x = coords_base['X'] - 0.13 # 从上方抓取的左手需要的偏移
            # pose_stamped.pose.position.y = coords_base['Y'] + 0.08
            # pose_stamped.pose.position.z = coords_base['Z'] - 0.005
            object_pose.pose.position.x = coords_base['X'] # - 0.10 # 偏移在后面再加
            object_pose.pose.position.y = coords_base['Y'] # + 0.13
            object_pose.pose.position.z = coords_base['Z'] # - 0.05        
        else:
            # 变换失败：降级到相机坐标系
            object_pose.header.frame_id = self.ob_camera_frame
            object_pose.pose.position.x = X_cam
            object_pose.pose.position.y = Y_cam
            object_pose.pose.position.z = Z_cam
            
            self.get_logger().warn(
                f"⚠ 无法变换到 {self.target_frame}: {coords_base['error']}"
            )
        
        
        # 四元数（从tf2_echo读取）- 必须归一化
        # q = [-0.041, 0.712, -0.690, 0.121]
        # q = [-0.179, 0.708, -0.388, 0.562] # 从上方抓取的左手末端位姿

        # 从tf2_echo读取的四元数，针对从侧面抓取的末端位姿，如果物体在y>0则直接使用左手抓取时的末端位姿，否则进行y轴对称变换也就是右手抓取的末端位姿
        q = left_arm_grasp_q if object_pose.pose.position.y > 0 else self.get_axis_symmetric_quaternion(left_arm_grasp_q, 'x') # 从侧面抓取的左手末端位姿
        offset = left_arm_grasp_offset if object_pose.pose.position.y > 0 else right_arm_grasp_offset # 从侧面抓取的左手末端位姿的偏移
        q_norm = np.linalg.norm(q)
        q = [x / q_norm for x in q]  # 归一化
        object_pose.pose.orientation.x = q[0]
        object_pose.pose.orientation.y = q[1]
        object_pose.pose.orientation.z = q[2]
        object_pose.pose.orientation.w = q[3]

        # 发布原始物体位姿（无偏移）
        self.object_pose_pub.publish(object_pose)

        # 构造抓取位姿：与 object_pose 姿态完全一致，仅 position 添加偏移
        hand_base_pose = PoseStamped()
        hand_base_pose.header = object_pose.header
        hand_base_pose.pose.orientation = object_pose.pose.orientation
        hand_base_pose.pose.position.x = object_pose.pose.position.x + offset[0]  # 从侧面抓取的对应的手需要的偏移
        hand_base_pose.pose.position.y = object_pose.pose.position.y + offset[1]
        hand_base_pose.pose.position.z = object_pose.pose.position.z + offset[2]

        self.grasp_pose_pub.publish(hand_base_pose)
        return hand_base_pose
        # self.get_logger().info(
        #     f"✓ 已发布抓取点在 {object_pose.header.frame_id} 坐标系下的坐标 | "
        #     f"检测到 {num_detections} 个物体"
        # )
        # self.get_logger().info("-" * 50)

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
        """同步处理彩色图和深度图的回调函数 - 支持IK结果确认时的暂停
        
        该回调函数确保处理的彩色图和深度图来自同一时刻的传感器数据
        
        **暂停机制说明：**
        - 当IK解算成功时，该回调会被暂停，停止接收新的图像帧
        - 暂停期间等待用户在命令行确认是否执行抓取
        - 用户确认后，该回调恢复正常运行
        - 暂停状态由 self.pause_image_processing 标志控制
        
        Args:
            color_msg: 彩色图像消息 (sensor_msgs/msg/Image)
            depth_msg: 深度图像消息 (sensor_msgs/msg/Image)
        """
        # ============ 消息转换 ============
        # 将ROS消息转换为OpenCV格式
        # 彩色图：BGR格式（OpenCV标准格式）
        color_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        
        # 深度图：使用 "passthrough" 保持原始编码格式
        # Orbbec 摄像头的深度图 encoding 是 "16UC1" 而不是 "mono16"
        # "passthrough" 表示不进行编码转换，直接传递原始数据
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        
        # 生成时间戳（同一帧使用相同时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

        # ============ 保存原始图像 ============

        # 2. 保存深度原始图片（深度图需要进行可视化处理才能正确显示）
        # 深度图的像素值通常在0-5000mm范围内，需要归一化到0-255以便显示
        depth_normalized = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_TURBO)
        # self.save_image(depth_colored, timestamp, suffix="raw_depth")

        # ============ YOLO物体识别 ============
        # 在彩色图上进行物体检测
        # device 参数决定了推理运行的硬件：
        #   - device="cuda" 或 device=0: GPU推理（快速，4-6倍加速）
        #   - device="cpu": CPU推理（慢速，但稳定）
        results = self.model.predict(
            source=color_img,
            device=self.device,  # 使用节点初始化时指定的设备
            conf=0.4,
            verbose=False
        )

        if len(results[0].boxes) == 0:
            # self.get_logger().info("No objects detected")
            # self.get_logger().info("-" * 50)
            # 未检测到物体时保存彩色原始图片备查
            # self.save_image(color_img, timestamp, suffix="raw_color")
            # self.save_image(depth_colored, timestamp, suffix="raw_depth")
        
            return

        # ============ 目标类别过滤 ============
        # 调用过滤方法，按目标类别筛选检测框
        # 如果未找到目标类别，filter_detections_by_target_classes() 会返回 False 并记录日志
        if not self.filter_detections_by_target_classes(results):
            # 未检测到物体时保存彩色原始图片备查
            # self.save_image(color_img, timestamp, suffix="raw_color")
        
            return
        
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
            # self.get_logger().info(f"{i}-{label} | 中心点: ({center_x_int}, {center_y_int}) | Depth raw={depth_value} | Depth={depth_meters:.3f}m")

        # ============ 保存标注后的图像 ============
        # 保存带标注的彩色图
        # self.save_image(annotated_img, timestamp, suffix="detected_color")
        
        # 保存带标注的深度图
        # self.save_image(depth_annotated, timestamp, suffix="detected_depth")

        # ============ 发布抓取点（使用第一个检测框，假设只有一个要抓取的物体，例如只有一个苹果在视野内，如果有两个会随机选择第一个） ============
        # 取第一个检测框计算抓取点
        box = results[0].boxes[0]
        
        # 计算抓取点的中心坐标和深度值（像素空间）
        cx, cy, cz_raw, cz = self.calculate_center_point_with_depth(box, depth_img)

        # ============ 转换到相机坐标系下的3D坐标 ============
        # 将像素坐标 (cx, cy) 和深度值 cz 转换为真实的相机坐标系下的3D坐标
        # 这一步是关键：从2D图像空间 + 深度值 → 3D相机坐标空间
        coords_3d_camera_frame = self.pixel_to_3d_camera_coords(cx, cy, cz)
        
        self.save_image(annotated_img, timestamp, suffix="detected_color")
        self.save_image(depth_annotated, timestamp, suffix="detected_depth")
        # 检查坐标转换是否成功，如果失败则忽略本次数据直接返回
        if not coords_3d_camera_frame.get('valid', False):
            self.get_logger().warn(f"⚠ 3D坐标转换失败：{coords_3d_camera_frame.get('error', '未知错误')}。忽略本帧数据，进行下次检测。")
            self.get_logger().info("-" * 50)
            return
        
        # 发布抓取点位姿
        pose_stamped = None
        try:
            pose_stamped = self.publish_grasp_pose(coords_3d_camera_frame, (cx, cy, cz_raw, cz), len(results[0].boxes))
        except Exception as e:
            self.get_logger().error(f"发布抓取点失败: {str(e)}")
            self.get_logger().info("-" * 50)
        
        # ============ 发布抓取候选点给执行节点 ============
        if pose_stamped is not None:
            self._publish_grasp_candidate(pose_stamped, results[0].boxes[0])

    def _publish_grasp_candidate(self, pose_stamped: PoseStamped, box):
        """将检测结果打包为 GraspCandidate 消息发布给执行节点"""
        msg = GraspCandidate()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = pose_stamped.header.frame_id
        msg.pose            = pose_stamped
        msg.group_name      = "left_arm" if pose_stamped.pose.position.y > 0 else "right_arm"
        msg.confidence      = float(box.conf[0].cpu().numpy())
        cls                 = int(box.cls[0].cpu().numpy())
        msg.object_class    = self.class_names.get(cls, 'unknown')

        self.grasp_candidate_pub.publish(msg)
        self.get_logger().info(
            f'✓ 已发布抓取候选点 | 物体: {msg.object_class} '
            f'| 置信度: {msg.confidence:.2f} | 坐标系: {msg.header.frame_id}'
        )


def main():
    rclpy.init()
    node = YoloDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n用户中断程序")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
