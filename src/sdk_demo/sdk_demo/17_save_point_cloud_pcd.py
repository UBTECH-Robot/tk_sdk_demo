# 使用方式：
# 1. 确保已安装 open3d 库：pip install open3d
# 2. 运行此节点：
#       ros2 run sdk_demo save_point_cloud_pcd
# 3. 如果没有数据，请检查：
#    - 雷达是否正常工作
#    - 话题名称是否正确：ros2 topic list
#    - 话题是否有数据发布：ros2 topic hz /livox/lidar

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
        
        # 保存目录和文件配置
        self.save_dir = 'pointcloud_pcds'
        self.tar_file = 'pointcloud_pcds.tar.gz'
        self.image_count = 0
        self.max_images = 100
        self.last_save_time = time.time()
        
        # 用于监控是否收到数据
        self.received_first_msg = False
        self.last_msg_time = None
        self.total_callbacks = 0
        
        # 准备保存目录
        self.prepare_directory()
        
        # 订阅点云话题
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            10
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('点云保存节点已启动')
        self.get_logger().info(f'订阅话题: /livox/lidar')
        self.get_logger().info(f'保存目录: {os.path.abspath(self.save_dir)}')
        self.get_logger().info(f'目标数量: {self.max_images} 个PCD文件')
        self.get_logger().info(f'保存间隔: 0.5秒')
        self.get_logger().info('正在等待点云数据...')
        self.get_logger().info('=' * 60)
        
        # 创建定时器，每5秒输出一次状态
        self.status_timer = self.create_timer(5.0, self.print_status)

    def prepare_directory(self):
        """准备保存目录"""
        if os.path.exists(self.save_dir):
            shutil.rmtree(self.save_dir)
        os.makedirs(self.save_dir)

        if os.path.exists(self.tar_file):
            os.remove(self.tar_file)

        self.get_logger().info(f'已清空旧数据，准备保存至目录：{os.path.abspath(self.save_dir)}')
    
    def print_status(self):
        """定期打印节点状态"""
        if not self.received_first_msg:
            self.get_logger().warn('⚠️  还未收到任何点云数据！')
            self.get_logger().warn('   请检查：')
            self.get_logger().warn('   1. 雷达是否已启动')
            self.get_logger().warn('   2. 运行命令检查话题：ros2 topic list | grep livox')
            self.get_logger().warn('   3. 检查话题数据：ros2 topic hz /livox/lidar')
        else:
            elapsed = time.time() - self.last_msg_time if self.last_msg_time else 0
            self.get_logger().info(f'📊 状态: 已保存 {self.image_count}/{self.max_images} 个文件, '
                                 f'总回调次数: {self.total_callbacks}, '
                                 f'上次接收: {elapsed:.1f}秒前')

    def pointcloud_callback(self, msg):
        """点云数据回调函数"""
        # 标记已收到第一条消息
        if not self.received_first_msg:
            self.received_first_msg = True
            self.get_logger().info('✅ 成功接收到第一帧点云数据！')
        
        self.total_callbacks += 1
        self.last_msg_time = time.time()
        
        # 时间间隔控制
        now = time.time()
        if now - self.last_save_time < 0.5:  # 每 0.5 秒保存一次
            return
        self.last_save_time = now

        # 达到最大保存数量
        if self.image_count >= self.max_images:
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'✅ 已保存 {self.max_images} 个 .pcd 文件，正在打包...')
            self.create_tarfile()
            self.get_logger().info('✅ 所有任务完成，节点即将退出')
            self.get_logger().info('=' * 60)
            rclpy.shutdown()
            return

        try:
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
            filename = os.path.join(self.save_dir, f'pointcloud_{self.image_count:04d}.pcd')
            o3d.io.write_point_cloud(filename, pcd, write_ascii=True)
            
            progress = (self.image_count + 1) / self.max_images * 100
            self.get_logger().info(f'💾 [{self.image_count + 1}/{self.max_images}] ({progress:.1f}%) '
                                 f'保存: {filename} ({len(points)} 个点)')
            self.image_count += 1
            
        except Exception as e:
            self.get_logger().error(f'❌ 保存点云时出错: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def create_tarfile(self):
        """打包PCD文件为tar.gz"""
        try:
            with tarfile.open(self.tar_file, "w:gz") as tar:
                tar.add(self.save_dir, arcname=os.path.basename(self.save_dir))
            file_size = os.path.getsize(self.tar_file) / (1024 * 1024)  # MB
            self.get_logger().info(f'📦 打包完成: {self.tar_file} ({file_size:.2f} MB)')
        except Exception as e:
            self.get_logger().error(f'❌ 打包失败: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n⚠️  用户中断（Ctrl+C），正在退出...')
        if node.image_count > 0:
            node.get_logger().info(f'已保存 {node.image_count} 个文件，正在打包...')
            node.create_tarfile()
    except Exception as e:
        node.get_logger().error(f'❌ 运行时错误: {str(e)}')
        import traceback
        node.get_logger().error(traceback.format_exc())
    finally:
        node.get_logger().info('节点正在关闭...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
