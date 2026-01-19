import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import os
import numpy as np
import time
import glob
import shutil
import open3d as o3d
import tarfile

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_pcd_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            10
        )

        self.save_dir = 'pointcloud_pcds'
        self.tar_file = 'pointcloud_pcds.tar.gz'
        self.image_count = 0
        self.max_images = 100
        self.last_save_time = time.time()

        self.prepare_directory()

    def prepare_directory(self):
        if os.path.exists(self.save_dir):
            shutil.rmtree(self.save_dir)
        os.makedirs(self.save_dir)

        if os.path.exists(self.tar_file):
            os.remove(self.tar_file)

        self.get_logger().info(f'清空旧数据，准备保存至目录：{self.save_dir}')

    def pointcloud_callback(self, msg):
        now = time.time()
        if now - self.last_save_time < 0.5:  # 每 0.5 秒保存一次
            return
        self.last_save_time = now

        if self.image_count >= self.max_images:
            self.get_logger().info(f'已保存 {self.max_images} 张 .pcd 文件，打包并退出...')
            self.create_tarfile()
            rclpy.shutdown()
            return

        # 转换为 numpy 数组
        points = np.array([
            [x, y, z]
            for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])

        if len(points) == 0:
            self.get_logger().warn("收到的点云为空，跳过本次保存。")
            return

        # 创建 Open3D 点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 保存为 .pcd 文件
        filename = os.path.join(self.save_dir, f'pointcloud_{self.image_count}.pcd')
        o3d.io.write_point_cloud(filename, pcd, write_ascii=True)
        self.get_logger().info(f'保存：{filename}')
        self.image_count += 1

    def create_tarfile(self):
        with tarfile.open(self.tar_file, "w:gz") as tar:
            tar.add(self.save_dir, arcname=os.path.basename(self.save_dir))
        self.get_logger().info(f'打包完成: {self.tar_file}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
