#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Livox 雷达点云数据批量保存为PCD文件

功能说明：
=========
本脚本用于从Livox雷达订阅点云数据，并将其保存为标准的PCD（Point Cloud Data）文件格式。
PCD是PCL（Point Cloud Library）库的标准格式，支持多种点云处理工具读取。

主要功能：
1. 订阅 /livox/lidar 话题的 PointCloud2 消息
2. 每0.5秒保存一帧点云为单独的PCD文件（ASCII格式）
3. 达到指定数量（默认100个）后自动停止
4. 将所有PCD文件打包为 pointcloud_pcds.tar.gz 便于传输和存储
5. 实时监控数据接收状态，帮助排查问题

使用方式：
=========
1. 安装依赖库：
   pip install open3d
   或
   conda install -c open3d-admin open3d

2. 确保Livox雷达已连接并正常工作：
   - 检查话题列表：ros2 topic list | grep livox
   - 检查话题数据：ros2 topic hz /livox/lidar
   - 查看点云内容：ros2 topic echo /livox/lidar --no-arr

3. 运行此节点：
   ros2 run sdk_demo save_point_cloud_pcd

4. 等待采集完成，脚本会自动退出并生成：
   - pointcloud_pcds/ 目录（包含所有PCD文件）
   - pointcloud_pcds.tar.gz 打包文件

5. 查看PCD文件（任选其一）：
   - CloudCompare: 专业点云查看器
   - PCL Viewer: pcl_viewer pointcloud_0001.pcd
   - Python + Open3D: o3d.io.read_point_cloud("xxx.pcd")
   - RViz2: 先转换回PointCloud2再可视化

配置参数：
=========
可在 __init__ 方法中修改以下参数：
- save_dir: 保存目录名称（默认 'pointcloud_pcds'）
- max_images: 最大保存文件数（默认 100）
- 时间间隔: 在 pointcloud_callback 中修改（默认 0.5秒）

注意事项：
=========
1. 磁盘空间：每个PCD文件约3-10MB（取决于点数），100个文件需300MB-1GB空间
2. 保存速度：ASCII格式便于阅读但文件较大，如需高速保存可改用binary格式
3. 数据完整性：脚本会跳过空点云，确保每个文件都有有效数据
4. Ctrl+C中断：会自动打包已保存的文件，不会丢失数据

故障排查：
=========
如果脚本运行后没有保存数据，依次检查：
1. 雷达连接：检查硬件连接和驱动程序
2. ROS话题：ros2 topic list 确认 /livox/lidar 存在
3. 数据发布：ros2 topic hz /livox/lidar 确认有数据流
4. 权限问题：确保当前目录有写入权限
5. 依赖库：确认 open3d 已正确安装

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import time
import glob
import shutil
import open3d as o3d
import tarfile

class PointCloudSaver(Node):
    """点云数据保存节点
    
    从Livox雷达订阅点云数据，并将其批量保存为PCD文件格式。
    支持自动打包、进度监控和错误处理。
    """
    
    def __init__(self):
        """初始化点云保存节点
        
        设置保存目录、订阅话题、创建定时器等初始化操作。
        """
        # 调用父类Node的初始化方法，设置节点名称为'pointcloud_pcd_saver'
        super().__init__('pointcloud_pcd_saver')
        
        # ========== 智能定位源代码目录 ==========
        # 使用 _find_source_directory 方法自动定位到源代码目录
        # 适用于从install目录运行或直接从src目录运行的情况
        self.script_dir = self._find_source_directory()
        
        # ========== 保存目录和文件配置 ==========
        # 保存PCD文件的目录路径（使用Path对象）
        self.save_dir = self.script_dir / 'livox_data' / 'pointcloud_pcds'
        # 打包后的tar.gz文件路径（使用Path对象）
        self.tar_file = self.script_dir / 'livox_data' / 'pointcloud_pcds.tar.gz'
        # 已保存的文件计数器
        self.image_count = 0
        # 最大保存文件数量，达到此数量后自动停止
        self.max_images = 100
        # 上次保存时间戳，用于控制保存间隔
        self.last_save_time = time.time()
        
        # ========== 数据接收监控变量 ==========
        # 标记是否已收到第一条消息，用于诊断数据接收问题
        self.received_first_msg = False
        # 最后一条消息的时间戳，用于计算数据接收间隔
        self.last_msg_time = None
        # 总回调次数统计，用于监控数据接收频率
        self.total_callbacks = 0
        
        # ========== 准备保存目录 ==========
        # 清理旧数据并创建新的保存目录
        self.prepare_directory()
        
        # ========== 订阅点云话题 ==========
        # 创建订阅者，订阅Livox雷达的点云数据话题
        # 参数说明：
        #   - PointCloud2: 消息类型
        #   - '/livox/lidar': 话题名称
        #   - self.pointcloud_callback: 回调函数
        #   - 10: 队列大小
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            10
        )
        
        # ========== 输出启动信息 ==========
        self.get_logger().info('=' * 60)
        self.get_logger().info('点云保存节点已启动')
        self.get_logger().info(f'订阅话题: /livox/lidar')
        self.get_logger().info(f'保存目录: {self.save_dir.resolve()}')
        self.get_logger().info(f'目标数量: {self.max_images} 个PCD文件')
        self.get_logger().info(f'保存间隔: 0.5秒')
        self.get_logger().info('正在等待点云数据...')
        self.get_logger().info('=' * 60)
        
        # ========== 创建状态监控定时器 ==========
        # 每5秒输出一次节点状态，帮助用户了解运行情况
        # 参数说明：
        #   - 5.0: 定时器间隔（秒）
        #   - self.print_status: 定时器回调函数
        self.status_timer = self.create_timer(5.0, self.print_status)
    
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
        """准备保存目录
        
        清理旧的点云数据和打包文件，创建新的保存目录。
        确保每次运行都从干净的状态开始。
        """
        # 检查保存目录是否存在
        if self.save_dir.exists():
            # 如果存在，删除整个目录及其所有内容
            # 使用shutil.rmtree可以递归删除非空目录
            shutil.rmtree(self.save_dir)
        
        # 创建新的保存目录
        # Path.mkdir会创建所有必要的父目录（如果不存在）
        self.save_dir.mkdir(parents=True, exist_ok=True)

        # 检查打包文件是否存在
        if self.tar_file.exists():
            # 如果存在，删除旧的打包文件
            self.tar_file.unlink()

        # 输出目录准备完成的信息
        # Path.resolve()获取绝对路径，便于用户定位文件
        self.get_logger().info(f'已清空旧数据，准备保存至目录：{self.save_dir.resolve()}')
    
    def print_status(self):
        """定期打印节点状态
        
        每5秒由定时器调用，输出当前运行状态。
        根据是否收到数据，输出不同的诊断信息。
        """
        # 检查是否已收到第一条消息
        if not self.received_first_msg:
            # 未收到数据，输出警告信息和排查建议
            self.get_logger().warn('⚠️  还未收到任何点云数据！')
            self.get_logger().warn('   请检查：')
            self.get_logger().warn('   1. 雷达是否已启动')
            # 提供检查话题列表的命令
            self.get_logger().warn('   2. 运行命令检查话题：ros2 topic list | grep livox')
            # 提供检查话题数据频率的命令
            self.get_logger().warn('   3. 检查话题数据：ros2 topic hz /livox/lidar')
        else:
            # 已收到数据，输出当前进度统计
            # 计算距离上次接收消息的时间（秒）
            elapsed = time.time() - self.last_msg_time if self.last_msg_time else 0
            # 输出已保存文件数、总回调次数、距离上次接收的时间
            self.get_logger().info(f'📊 状态: 已保存 {self.image_count}/{self.max_images} 个文件, '
                                 f'总回调次数: {self.total_callbacks}, '
                                 f'上次接收: {elapsed:.1f}秒前')

    def pointcloud_callback(self, msg):
        """点云数据回调函数
        
        每次收到点云消息时调用，负责处理和保存点云数据。
        实现了时间间隔控制、数量限制、数据转换和文件保存等功能。
        
        Args:
            msg: sensor_msgs.msg.PointCloud2类型的点云消息
        """
        # ========== 标记数据接收状态 ==========
        # 检查是否是第一次收到消息
        if not self.received_first_msg:
            self.received_first_msg = True
            self.get_logger().info('✅ 成功接收到第一帧点云数据！')
        
        # 更新统计信息
        self.total_callbacks += 1
        self.last_msg_time = time.time()
        
        # ========== 时间间隔控制 ==========
        # 获取当前时间
        now = time.time()
        # 检查距离上次保存的时间是否小于0.5秒
        # 这样可以避免过于频繁地保存文件，控制数据采集速度
        if now - self.last_save_time < 0.5:
            # 如果间隔不足，直接返回，不处理本次消息
            return
        # 更新上次保存时间
        self.last_save_time = now

        # ========== 检查是否达到最大保存数量 ==========
        if self.image_count >= self.max_images:
            # 已达到目标数量，输出完成信息
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'✅ 已保存 {self.max_images} 个 .pcd 文件，正在打包...')
            # 调用打包函数，将所有PCD文件打包为tar.gz
            self.create_tarfile()
            self.get_logger().info('✅ 所有任务完成，节点即将退出')
            self.get_logger().info('=' * 60)
            # 关闭ROS2节点，退出程序
            rclpy.shutdown()
            return

        # ========== 点云数据处理和保存 ==========
        try:
            # 将ROS2 PointCloud2消息转换为numpy数组
            # pc2.read_points是sensor_msgs_py提供的工具函数
            # 参数说明：
            #   - msg: PointCloud2消息
            #   - field_names: 指定要提取的字段（x, y, z坐标）
            #   - skip_nans: 跳过NaN（非数字）值，避免无效点
            points = np.array([
                [x, y, z]
                for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            ])

            # 检查点云是否为空
            if len(points) == 0:
                # 如果没有有效点，输出警告并跳过本次保存
                self.get_logger().warn("收到的点云为空，跳过本次保存。")
                return

            # ========== 创建Open3D点云对象 ==========
            # 创建Open3D的点云对象
            pcd = o3d.geometry.PointCloud()
            # 设置点云的坐标数据
            # o3d.utility.Vector3dVector将numpy数组转换为Open3D的向量格式
            pcd.points = o3d.utility.Vector3dVector(points)

            # ========== 保存为PCD文件 ==========
            # 生成文件名：pointcloud_0001.pcd, pointcloud_0002.pcd, ...
            # {self.image_count:04d} 表示用4位数字，不足前面补0
            filename = self.save_dir / f'pointcloud_{self.image_count:04d}.pcd'
            # 保存点云为PCD文件
            # write_ascii=True表示使用ASCII格式保存（可读性好但文件较大）
            # 如需更快的保存速度，可以改为write_ascii=False（二进制格式）
            o3d.io.write_point_cloud(str(filename), pcd, write_ascii=True)
            
            # ========== 输出进度信息 ==========
            # 计算完成百分比
            progress = (self.image_count + 1) / self.max_images * 100
            # 输出保存成功的日志，包含：序号、进度百分比、文件名、点数
            self.get_logger().info(f'💾 [{self.image_count + 1}/{self.max_images}] ({progress:.1f}%) '
                                 f'保存: {filename} ({len(points)} 个点)')
            # 更新文件计数器
            self.image_count += 1
            
        except Exception as e:
            # 捕获并处理保存过程中的异常
            self.get_logger().error(f'❌ 保存点云时出错: {str(e)}')
            # 输出完整的异常堆栈信息，便于调试
            import traceback
            self.get_logger().error(traceback.format_exc())

    def create_tarfile(self):
        """打包PCD文件为tar.gz
        
        将保存目录中的所有PCD文件打包为一个tar.gz压缩文件。
        便于传输和存储，减少文件数量。
        使用gzip压缩算法，可以显著减小文件大小。
        """
        try:
            # 打开tar.gz文件进行写入
            # "w:gz"表示以写入模式打开，使用gzip压缩
            with tarfile.open(self.tar_file, "w:gz") as tar:
                # 将整个保存目录添加到tar文件中
                # arcname参数指定在tar文件中的路径（不包含完整路径）
                # Path.name提取目录名称，使tar文件结构更简洁
                tar.add(self.save_dir, arcname=self.save_dir.name)
            
            # 计算打包文件的大小（MB）
            # Path.stat().st_size获取文件字节数
            # 除以1024*1024转换为MB
            file_size = self.tar_file.stat().st_size / (1024 * 1024)
            
            # 输出打包完成的信息，包含文件名和大小
            self.get_logger().info(f'📦 打包完成: {self.tar_file.name} ({file_size:.2f} MB)')
        except Exception as e:
            # 捕获并处理打包过程中的异常
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
