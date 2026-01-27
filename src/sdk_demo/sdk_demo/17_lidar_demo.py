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

1. Livox雷达专用IMU (Inertial Measurement Unit - 惯性测量单元)
   
   1.1 为什么激光雷达需要配备IMU？
   - 运动畸变补偿：激光雷达扫描需要时间，在这期间如果雷达本身在运动（如机器人移动、
     转向），会导致点云发生畸变。IMU可以实时测量这些运动，用于后续的畸变校正
   - 姿态估计：提供雷达在空间中的精确姿态信息，这对于点云数据的坐标变换至关重要
   - 时间同步：IMU数据频率高（通常100-200Hz），可以为点云提供高频的姿态插值
   - 多传感器融合：IMU数据可以与点云融合，提高定位精度和鲁棒性
   
   1.2 Livox雷达IMU的特点
   - 硬件时间同步：IMU和激光雷达共享同一时钟源，确保数据时间戳精确对齐
   - 工厂标定：IMU与雷达的相对位置和姿态在出厂时已经标定，无需用户再次标定
   - 高采样率：通常为100Hz或200Hz，能够捕捉快速运动
   - 低延迟：IMU数据与点云数据的时间差通常在毫秒级
   
   1.3 IMU传感器组件
   - 三轴陀螺仪：测量角速度(绕X、Y、Z轴的旋转速度)，单位 rad/s
   - 三轴加速度计：测量线性加速度(沿X、Y、Z轴的加速度)，单位 m/s²
   - 磁力计（部分型号）：测量磁场方向，辅助航向估计
   
   1.4 在激光SLAM中的应用
   - 运动补偿：校正扫描过程中的点云畸变，提高点云质量
   - 位姿预测：在点云匹配前提供初始位姿估计，加速配准过程
   - 回环检测：利用IMU累积的位姿变化辅助判断是否回到原点
   - 退化场景：在激光雷达失效的场景（如长走廊、空旷环境）提供短期定位
   
   1.5 数据格式与单位
   - 角速度：rad/s (弧度/秒)，表示旋转的快慢
   - 线性加速度：m/s² (米/秒²)，表示速度变化率
   - 方向(四元数)：(x, y, z, w)，无量纲，表示3D空间中的旋转姿态
   - 时间戳：与雷达点云精确同步，误差通常<1ms

2. PointCloud2 (点云数据) - 通俗易懂的理解
   
   2.1 什么是点云？用生活中的例子来理解
   - 想象你在黑暗中用激光笔扫描一个房间，每次激光打到物体表面，你就记录下这个点的位置
   - 扫描成千上万次后，这些点连在一起，就能"画"出房间的3D形状
   - 这些密密麻麻的点，就是"点云"
   
   类比理解：
   * 如果把照片比作"2D像素的集合"（每个像素记录颜色）
   * 那么点云就是"3D点的集合"（每个点记录空间位置）
   * 照片告诉你"那里有什么颜色"，点云告诉你"那里有什么物体，距离多远"
   
   2.2 PointCloud2数据为什么人类无法直接阅读？
   
   原因1：二进制编码存储
   - PointCloud2使用紧凑的二进制格式存储，不是文本格式
   - 就像你打开一个.jpg图片的原始数据，看到的是乱码
   - 例如：一个点的坐标(1.234, 5.678, 9.012)被编码成12个字节的二进制数据
   
   原因2：数据量巨大
   - 一帧点云可能包含50,000到100,000个点
   - 每个点有多个属性：x, y, z坐标、强度、时间戳等
   - 总数据量：50,000点 × 每点16字节 = 800KB（单帧！）
   - 即使转成文本，也是上万行数字，人眼无法理解
   
   原因3：缺乏直观的组织结构
   - 这些点是无序的3D空间散点，没有"从左到右、从上到下"的阅读顺序
   - 不像CSV表格那样有行列结构，可以逐行阅读
   - 就像把一个雕塑打散成沙粒，你无法从沙粒直接"看出"原来的雕塑形状
   
   2.3 通俗理解PointCloud2的数据结构
   
   可以把PointCloud2想象成一个"特殊的Excel表格"：
   
   | 点编号 | X坐标(m) | Y坐标(m) | Z坐标(m) | 强度 | 时间戳(ns) |
   |-------|---------|---------|---------|-----|-----------|
   | 1     | 1.234   | 2.456   | 0.123   | 100 | 123456789 |
   | 2     | 1.567   | 2.890   | 0.145   | 95  | 123456790 |
   | ...   | ...     | ...     | ...     | ... | ...       |
   | 50000 | 5.678   | 8.901   | 1.234   | 87  | 123789012 |
   
   但是：
   - 这个"表格"有5万到10万行
   - 数据以二进制紧密打包，不是文本
   - 坐标是相对雷达的，需要转换才能理解空间关系
   
   2.4 如何"看懂"点云数据？
   
   方法1：可视化工具（推荐）
   - 使用PCL Viewer、CloudCompare等工具
   - 将数万个点绘制成3D图像，人眼可以直观看到形状
   - 就像把散落的沙粒重新组装成雕塑
   
   方法2：转换为可读格式
   - 本脚本将PointCloud2转换为PCD文件（ASCII格式）
   - PCD文件虽然也很大，但至少可以用文本编辑器打开查看
   - 可以看到每个点的具体坐标值（尽管仍然难以理解整体形状）
   
   方法3：统计分析
   - 不直接看每个点，而是看统计信息
   - 如：点云中心位置、密度、范围、最近障碍物距离等
   - 就像不数人群中的每个人，而是看"大约有多少人"
   
   2.5 Livox雷达特点
   - 固态激光雷达，采用非重复扫描模式（覆盖更均匀）
   - 高分辨率和高精度的3D测距（误差通常<2cm）
   - 视场角广（通常70°-100°），适合机器人导航和环境感知
   
   2.6 在机器人中的应用
   - 3D环境建模：构建周围环境的三维模型（如建筑物、树木）
   - 障碍物检测：识别前方障碍物的位置和形状（如行人、车辆）
   - 地形分析：评估地面的可通行性（如楼梯、坡道、坑洼）
   - SLAM：同时定位与地图构建（机器人知道自己在哪里）
   
   2.7 PointCloud2消息的技术细节
   - header: 时间戳和坐标系信息
   - width × height: 点云的组织结构（无序点云height=1，width=点数）
   - fields: 每个点包含哪些数据（x,y,z,intensity等）
   - point_step: 一个点占用多少字节（如16字节）
   - row_step: 一行数据占用多少字节
   - data: 实际的点云数据（二进制格式，这就是人类无法阅读的部分）
   - is_dense: 所有点是否都有效（无NaN或Inf值）

数据保存策略：
============
- IMU数据保存为CSV文件：便于数据分析和可视化
- 点云数据保存为PCD文件：标准的点云存储格式
- 自动管理文件数量：超过10个文件时自动删除最旧的文件
- 文件命名包含时间戳：便于识别和管理

作者：SDK Demo
日期：2026-01-27
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
