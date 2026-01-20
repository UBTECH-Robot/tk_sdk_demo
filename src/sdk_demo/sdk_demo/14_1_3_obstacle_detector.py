import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d

# cd livox_ros_driver2
# source install/setup.bash
# ros2 launch livox_ros_driver2 msg_MID360_launch.py
class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('livox_obstacle_detector')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.callback,
            10
        )

    def callback(self, msg):
        # 读取点云
        points = np.array([
            [x, y, z]
            for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])
        if len(points) == 0:
            self.get_logger().info("没有检测到点云数据")
            return

        # 创建 Open3D 点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 体素下采样（降分辨率加速处理）
        pcd = pcd.voxel_down_sample(voxel_size=0.1)

        # 去除地面（Z 轴高度滤波）
        points = np.asarray(pcd.points)
        filtered_points = points[points[:, 2] > -0.5]  # 可根据实际情况调整
        pcd.points = o3d.utility.Vector3dVector(filtered_points)

        # 聚类：欧几里得距离
        labels = np.array(
            pcd.cluster_dbscan(eps=0.5, min_points=10, print_progress=False)
        )

        max_label = labels.max()
        self.get_logger().info(f'检测到 {max_label + 1} 个障碍物')

        # 提取每个障碍物的中心位置
        for i in range(max_label + 1):
            cluster = np.asarray(pcd.points)[labels == i]
            center = cluster.mean(axis=0)
            self.get_logger().info(f'障碍物 #{i} 中心位置: x={center[0]:.2f}, y={center[1]:.2f}, z={center[2]:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
