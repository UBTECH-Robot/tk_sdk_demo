#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 使用方式：
# 1. 确保已安装 matplotlib 库：pip install matplotlib
# 2. 运行此节点：
#       ros2 run sdk_demo save_pointcloud_images

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import time
import matplotlib.pyplot as plt
from pathlib import Path

class PointCloudXZPlotter(Node):
    def __init__(self):
        super().__init__('livox_pointcloud_xz_plotter')

        # 使用 _find_source_directory 方法智能定位源代码目录
        self.script_dir = self._find_source_directory()
        self.save_dir = self.script_dir / 'livox_data' / 'pointcloud_imgs'
        self.prepare_directory()

        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            10
        )

        self.image_count = 0
        self.max_images = 100
        self.last_save_time = time.time()

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
    
    def prepare_directory(self):
        if not self.save_dir.exists():
            self.save_dir.mkdir(parents=True, exist_ok=True)
        else:
            png_files = list(self.save_dir.glob('*.png'))
            for f in png_files:
                f.unlink()
            self.get_logger().info(f'清空旧图片，准备保存至目录：{self.save_dir.resolve()}')

    def pointcloud_callback(self, msg):
        now = time.time()
        if now - self.last_save_time < 0.2:  # 每 0.2 秒保存一次
            return
        self.last_save_time = now

        if self.image_count >= self.max_images:
            self.get_logger().info('已保存 100 张图片，退出程序。')
            rclpy.shutdown()
            return

        points = np.array([
            [x, y, z]
            for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])

        if len(points) == 0:
            self.get_logger().warn('收到的点云为空，跳过本次保存。')
            return

        # 提取 X 和 Z，用 Z 作为颜色映射
        x = points[:, 0]
        z = points[:, 2]
        c = z  # Z 值用于颜色映射

        # 绘制 X-Z 图
        plt.figure(figsize=(8, 6))
        scatter = plt.scatter(x, z, c=c, cmap='viridis', s=0.5)  # viridis 是蓝-绿-黄渐变
        plt.colorbar(scatter, label='Z 高度 [m]')
        plt.title(f'PointCloud X-Z View #{self.image_count}')
        plt.xlabel('X [m]')
        plt.ylabel('Z [m]')
        plt.axis('equal')
        plt.grid(True)

        # 保存图像
        image_path = self.save_dir / f'pointcloud_{self.image_count}.png'
        plt.savefig(str(image_path), dpi=150)
        plt.close()
        self.get_logger().info(f'保存图像: {image_path.name}')
        self.image_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudXZPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
