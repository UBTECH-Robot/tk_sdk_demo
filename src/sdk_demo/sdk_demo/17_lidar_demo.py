#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Livox 雷达数据采集与保存节点

运行脚本：
    ros2 run sdk_demo lidar_demo
    
本脚本用于订阅和处理Livox雷达的两种数据类型：
1. IMU数据 (/livox/imu)
2. 点云数据 (/livox/lidar)

背景知识：
==========

1. Livox雷达内置IMU (Inertial Measurement Unit - 惯性测量单元)
   
   对于用户而言，通常不需要直接处理IMU数据。Livox雷达内置的IMU主要用于：
   
   - 运动畸变补偿：机器人移动时，IMU帮助修正点云畸变，输出更准确的点云数据
   - 姿态同步：IMU与雷达已在出厂时完成标定，自动提供精确的姿态信息
   - 多传感器融合：如需要高级SLAM算法，可选择性使用IMU数据提高定位精度
   
   简而言之：IMU在后台工作，确保点云质量。本脚本提供IMU数据订阅功能，
   但大多数应用场景中，用户只需关注点云数据即可。
   
   IMU数据格式（仅供参考）：
   - 角速度 (angular_velocity)：单位 rad/s，表示旋转速度
   - 线性加速度 (linear_acceleration)：单位 m/s²，表示加速度
   - 方向 (orientation)：四元数 (x, y, z, w)，表示姿态

2. 激光雷达类型对比：2D vs 机械式3D vs Livox固态3D
   
   ┌─────────────────┬─────────────────┬─────────────────┬─────────────────┐
   │ 特性            │ 2D雷达          │ 机械式3D雷达     │ Livox固态3D雷达  │
   ├─────────────────┼─────────────────┼─────────────────┼─────────────────┤
   │ 扫描方式        │ 平面旋转扫描    │ 多线束旋转扫描  │ 固态非重复扫描  │
   │ 覆盖范围        │ 360°平面        │ 360°立体空间    │ 70-100°半球     │
   │ 空间感知        │ 仅单一平面高度  │ 完整3D空间      │ 完整3D空间      │
   │ 数据格式        │ LaserScan(角度+距离) │ PointCloud2(3D点云) │ PointCloud2(3D点云) │
   │ 点数/帧         │ 几百个点        │ 数万个点        │ 数万个点        │
   │ 寿命            │ 5000-10000小时  │ 2000-5000小时   │ 理论10万小时    │
   │ 体积/重量       │ 小/轻           │ 大/重           │ 小/轻           │
   └─────────────────┴─────────────────┴─────────────────┴─────────────────┘
   
   2.1 2D激光雷达
   - 工作原理：在水平面内旋转扫描，只能检测该平面上的障碍物
   - 典型应用：室内导航、AGV避障、2D地图构建
   - 优势：成本低、算力需求小、算法成熟
   - 局限：无法检测悬空障碍、地面坑洼、楼梯台阶（缺少高度信息）
   
   2.2 传统机械式3D激光雷达
   - 工作原理：多线束（16/32/64线）激光发射器整体旋转，扫描360°立体空间
   - 扫描特点：重复性扫描（每次旋转激光打到的位置完全相同），线束间有固定间隙
   - 优势：360°全方位覆盖，技术成熟，SLAM算法生态完善
   - 劣势：成本极高（16线$4000+），机械旋转部件易磨损，体积大功耗高
   
   2.3 Livox固态3D激光雷达
   - 工作原理：微机电系统（MEMS）扫描，无宏观旋转部件
   - 扫描特点：非重复扫描（每次扫描路径略有不同），随时间累积覆盖整个视场
       * 类比：机械雷达像"固定梳子刷墙"（梳齿间有间隙），Livox像"喷枪喷涂"（逐步均匀覆盖）
       * 0.5秒内实现接近100%视场覆盖，能检测到细小障碍物（电线、树枝）
   - 优势：
       * 寿命长：固态设计，理论10万小时（机械式仅2000-5000小时）
       * 体积小巧：适合小型机器人和无人机
       * 均匀覆盖：非重复扫描消除固定盲区
   - 劣势：
       * 视场角有限（70-100°），需机器人转向或多雷达组合覆盖全方位
       * 需要支持非重复扫描的SLAM算法（传统算法需适配）

3. PointCloud2 点云数据说明
   
   3.1 什么是点云？
   - 点云是三维空间中大量点的集合，每个点记录(x, y, z)坐标
   - 类比：照片是"2D像素集合"（记录颜色），点云是"3D点集合"（记录空间位置）
   - 一帧Livox点云包含50,000-100,000个点，数据量约800KB-2MB
   
   3.2 为什么PointCloud2数据人类无法直接阅读？
   - 数据量巨大：5万个点×16字节=800KB，即使转成文本也有数万行数字
   - 二进制编码：为节省空间使用紧凑编码，打开是乱码
   - 无序散点：这些3D点没有"从左到右"的阅读顺序，无法像表格那样理解
   
   3.3 如何"看懂"点云数据？
   - 可视化工具：在装备了激光雷达的设备上，直接使用RViz查看点云数据
   - 本脚本方案：将PointCloud2转换为PCD文件（ASCII格式），可用文本编辑器打开
   - 统计分析：查看点云中心、密度、范围等统计信息，而非逐点查看
   
   3.4 PointCloud2消息结构（技术参考）
   - header: 时间戳和坐标系
   - width × height: 点云组织（无序点云height=1，width=点数）
   - fields: 每个点的数据字段（x, y, z, intensity, timestamp等）
   - data: 实际点云数据（二进制格式）
   - is_dense: 所有点是否有效（无NaN或Inf值）

数据保存策略：
============
- IMU数据保存为CSV文件：便于数据分析和可视化
- 点云数据保存为PCD文件：标准的点云存储格式
- 自动管理文件数量：超过10个文件时自动删除最旧的文件
- 文件命名包含时间戳：便于识别和管理

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os
import csv
from datetime import datetime
from pathlib import Path
import struct


class LivoxDataRecorder(Node):
    """
    Livox雷达数据记录节点
    
    功能：
    1. 订阅IMU和点云数据
    2. 实时显示数据统计信息
    3. 保存数据到本地文件系统
    4. 自动管理文件数量（最多保存10个）
    """
    
    def __init__(self):
        super().__init__('livox_data_recorder')
        
        # 创建数据保存目录
        self.script_dir = self._find_source_directory()
        self.data_dir = self.script_dir / 'livox_data'
        self.imu_dir = self.data_dir / 'imu'
        self.pointcloud_dir = self.data_dir / 'pointcloud'
        
        # 确保目录存在
        self.imu_dir.mkdir(parents=True, exist_ok=True)
        self.pointcloud_dir.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info(f'数据保存目录: {self.data_dir}')
        
        # 文件保存上限
        self.max_files = 50
        
        # 数据统计计数器
        self.imu_count = 0
        self.pointcloud_count = 0
        self.imu_save_count = 0
        self.pointcloud_save_count = 0
        
        # 订阅IMU话题
        self.imu_subscription = self.create_subscription(
            Imu,
            '/livox/imu',
            self.imu_callback,
            10
        )
        self.get_logger().info('已订阅话题: /livox/imu')
        
        # 订阅点云话题
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            10
        )
        self.get_logger().info('已订阅话题: /livox/lidar')
        
        # 创建定时器，每5秒显示一次统计信息
        self.timer = self.create_timer(5.0, self.display_statistics)
        
    def _find_source_directory(self):
        """智能定位源代码目录
        
        通过分析当前文件路径，自动定位到ROS2工作空间的src目录。
        适用于从install目录运行或直接从src目录运行的情况。
        
        Returns:
            Path: 源代码目录路径
        """
        current_file = Path(__file__).resolve()
        
        # 情况1: 如果当前在install目录运行
        # 路径类似: /path/to/workspace/install/pkg/lib/.../file.py
        # 需要找到workspace根目录，然后定位到src/pkg/pkg/
        if 'install' in current_file.parts:
            # 向上查找直到找到包含install目录的父目录（工作空间根目录）
            path = current_file
            while path.parent != path:
                # 检查当前目录是否同时包含install和src目录（工作空间根目录特征）
                if (path / 'install').exists() and (path / 'src').exists():
                    # 找到工作空间根目录
                    workspace_root = path
                    # 构造源代码路径: workspace/src/sdk_demo/sdk_demo/
                    source_dir = workspace_root / 'src' / 'sdk_demo' / 'sdk_demo'
                    if source_dir.exists():
                        return source_dir
                    break
                # 继续向上查找父目录
                path = path.parent
        
        # 情况2: 如果当前已经在src目录运行（开发时直接运行）
        # 路径类似: /path/to/workspace/src/pkg/pkg/file.py
        if 'src' in current_file.parts:
            # 当前文件的父目录就是我们要的目录（src/pkg/pkg/）
            return current_file.parent
        
        # 备用方案: 返回当前文件所在目录
        return current_file.parent
    
    def imu_callback(self, msg):
        """
        IMU数据回调函数
        
        IMU消息 (sensor_msgs/msg/Imu) 包含：
        - header: 时间戳和坐标系信息
        - orientation: 四元数表示的方向 (x, y, z, w)
        - angular_velocity: 角速度 (x, y, z) 单位: rad/s
        - linear_acceleration: 线性加速度 (x, y, z) 单位: m/s²
        - covariance: 协方差矩阵，表示测量的不确定性
        """
        self.imu_count += 1
        
        # 每接收100条IMU数据保存一次（IMU频率通常较高，避免频繁保存）
        if self.imu_count % 100 == 0:
            self.save_imu_data(msg)
            
        # 每收到10条数据显示一次（避免输出过多）
        if self.imu_count % 10 == 0:
            self.get_logger().info(
                f'IMU数据 #{self.imu_count} - '
                f'角速度: ({msg.angular_velocity.x:.3f}, '
                f'{msg.angular_velocity.y:.3f}, '
                f'{msg.angular_velocity.z:.3f}) rad/s, '
                f'加速度: ({msg.linear_acceleration.x:.3f}, '
                f'{msg.linear_acceleration.y:.3f}, '
                f'{msg.linear_acceleration.z:.3f}) m/s²'
            )
    
    def pointcloud_callback(self, msg):
        """
        点云数据回调函数
        
        PointCloud2消息 (sensor_msgs/msg/PointCloud2) 包含：
        - header: 时间戳和坐标系信息
        - height, width: 点云的组织结构（无序点云height=1）
        - fields: 每个点的字段定义（x, y, z, intensity等）
        - is_bigendian: 字节序
        - point_step: 单个点的字节数
        - row_step: 一行的字节数
        - data: 实际的点云数据（二进制格式）
        - is_dense: 是否所有点都有效（无NaN或Inf）
        """
        self.pointcloud_count += 1
        
        # 计算点云中的点数
        point_count = msg.width * msg.height
        
        # 每接收10帧点云保存一次（点云数据量大，不宜频繁保存）
        if self.pointcloud_count % 10 == 0:
            self.save_pointcloud_data(msg)
        
        # 显示点云信息
        self.get_logger().info(
            f'点云数据 #{self.pointcloud_count} - '
            f'点数: {point_count}, '
            f'尺寸: {msg.width}x{msg.height}, '
            f'坐标系: {msg.header.frame_id}'
        )
    
    def save_imu_data(self, msg):
        """
        保存IMU数据到CSV文件
        
        CSV格式便于在Excel或Python中进行数据分析
        每行包含一次IMU测量的所有信息
        """
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        filename = self.imu_dir / f'imu_{timestamp_str}.csv'
        
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # 写入表头
                writer.writerow([
                    'timestamp_sec', 'timestamp_nanosec',
                    'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                    'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                    'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'
                ])
                
                # 写入数据
                writer.writerow([
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ])
            
            self.imu_save_count += 1
            self.get_logger().info(f'IMU数据已保存: {filename.name}')
            
            # 管理文件数量
            self.manage_file_limit(self.imu_dir, '.csv')
            
        except Exception as e:
            self.get_logger().error(f'保存IMU数据失败: {str(e)}')
    
    def save_pointcloud_data(self, msg):
        """
        保存点云数据到PCD文件
        
        PCD (Point Cloud Data) 是PCL库的标准点云格式
        支持ASCII和二进制两种编码方式，这里使用ASCII便于查看
        """
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        filename = self.pointcloud_dir / f'pointcloud_{timestamp_str}.pcd'
        
        try:
            # 从PointCloud2消息中提取点数据
            points = []
            for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
                points.append(point)
            
            if len(points) == 0:
                self.get_logger().warn('点云为空，跳过保存')
                return
            
            # 写入PCD文件（ASCII格式）
            with open(filename, 'w') as f:
                # PCD文件头
                f.write('# .PCD v0.7 - Point Cloud Data file format\n')
                f.write('VERSION 0.7\n')
                f.write('FIELDS x y z\n')
                f.write('SIZE 4 4 4\n')
                f.write('TYPE F F F\n')
                f.write('COUNT 1 1 1\n')
                f.write(f'WIDTH {len(points)}\n')
                f.write('HEIGHT 1\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                f.write(f'POINTS {len(points)}\n')
                f.write('DATA ascii\n')
                
                # 写入点数据
                for point in points:
                    f.write(f'{point[0]} {point[1]} {point[2]}\n')
            
            self.pointcloud_save_count += 1
            self.get_logger().info(
                f'点云数据已保存: {filename.name} ({len(points)} 个点)'
            )
            
            # 管理文件数量
            self.manage_file_limit(self.pointcloud_dir, '.pcd')
            
        except Exception as e:
            self.get_logger().error(f'保存点云数据失败: {str(e)}')
    
    def manage_file_limit(self, directory, extension):
        """
        管理目录中的文件数量，确保不超过设定的上限
        
        策略：按文件修改时间排序，删除最旧的文件
        这样可以保证磁盘空间不会被无限占用
        
        参数：
            directory: 目录路径
            extension: 文件扩展名（如 '.csv' 或 '.pcd'）
        """
        try:
            # 获取所有指定类型的文件
            files = sorted(
                directory.glob(f'*{extension}'),
                key=lambda x: x.stat().st_mtime
            )
            
            # 如果文件数量超过上限，删除最旧的文件
            while len(files) > self.max_files:
                oldest_file = files.pop(0)
                oldest_file.unlink()
                self.get_logger().info(f'已删除旧文件: {oldest_file.name}')
                
        except Exception as e:
            self.get_logger().error(f'管理文件数量失败: {str(e)}')
    
    def display_statistics(self):
        """
        定时显示数据接收和保存的统计信息
        
        帮助用户了解节点的运行状态
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('数据统计信息:')
        self.get_logger().info(f'  IMU数据接收: {self.imu_count} 条')
        self.get_logger().info(f'  IMU数据保存: {self.imu_save_count} 个文件')
        self.get_logger().info(f'  点云数据接收: {self.pointcloud_count} 帧')
        self.get_logger().info(f'  点云数据保存: {self.pointcloud_save_count} 个文件')
        self.get_logger().info(f'  数据目录: {self.data_dir}')
        self.get_logger().info('=' * 60)


def main(args=None):
    """
    主函数：初始化ROS2节点并启动数据记录
    
    使用方法：
    1. 确保Livox雷达已连接并正常工作
    2. 运行此脚本: python3 17_lidar_demo.py
    3. 数据将自动保存到脚本所在目录的 livox_data 文件夹中
    4. 按 Ctrl+C 停止记录
    
    数据文件说明：
    - livox_data/imu/*.csv: IMU数据文件
    - livox_data/pointcloud/*.pcd: 点云数据文件
    - 文件名包含时间戳，便于识别
    - 自动保持最新的10个文件
    """
    rclpy.init(args=args)
    
    recorder = LivoxDataRecorder()
    
    try:
        recorder.get_logger().info('Livox数据记录节点已启动')
        recorder.get_logger().info('正在等待数据...')
        recorder.get_logger().info('按 Ctrl+C 停止记录')
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('用户中断，正在停止...')
    finally:
        # 显示最终统计
        recorder.get_logger().info('=' * 60)
        recorder.get_logger().info('最终统计:')
        recorder.get_logger().info(f'  总共接收IMU数据: {recorder.imu_count} 条')
        recorder.get_logger().info(f'  总共保存IMU文件: {recorder.imu_save_count} 个')
        recorder.get_logger().info(f'  总共接收点云数据: {recorder.pointcloud_count} 帧')
        recorder.get_logger().info(f'  总共保存点云文件: {recorder.pointcloud_save_count} 个')
        recorder.get_logger().info('=' * 60)
        
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
