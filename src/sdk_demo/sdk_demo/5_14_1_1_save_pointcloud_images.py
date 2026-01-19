import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import os
import time
import glob
import matplotlib.pyplot as plt

class PointCloudXZPlotter(Node):
    def __init__(self):
        super().__init__('livox_pointcloud_xz_plotter')

        self.save_dir = 'pointcloud_imgs'
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

    def prepare_directory(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        else:
            png_files = glob.glob(os.path.join(self.save_dir, '*.png'))
            for f in png_files:
                os.remove(f)
            self.get_logger().info(f'清空旧图片，准备保存至目录：{self.save_dir}')

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
        image_path = os.path.join(self.save_dir, f'pointcloud_{self.image_count}.png')
        plt.savefig(image_path, dpi=150)
        plt.close()
        self.get_logger().info(f'保存图像: {image_path}')
        self.image_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudXZPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
