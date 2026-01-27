#!/usr/bin/env python3
"""
深度相机保存节点
============================

功能说明:
1. 订阅头部/腰部/背部深度相机的彩色图和深度图
2. 每秒自动保存彩色图和深度图的左右拼接图
3. 自动检测相机服务是否启动

使用方法:
---------
1. 确保深度相机服务已启动

2. 运行节点:
   查看头部相机（默认）:
     ros2 run sdk_demo depth_camera
     ros2 run sdk_demo depth_camera --camera head
   
   查看腰部相机:
     ros2 run sdk_demo depth_camera --camera waist
   
   查看背部相机:
     ros2 run sdk_demo depth_camera --camera back

3. 图片保存位置:
   保存在源代码目录下的 10_depth_camera_imgs/ 文件夹中

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from PIL import Image as PILImage
from io import BytesIO
import time
from datetime import datetime
from pathlib import Path
import subprocess


class DepthCameraNode(Node):
    """深度相机节点类"""
    
    # 相机配置
    CAMERA_CONFIGS = {
        'head': {
            'name': '头部相机',
            'start_tips': '可在41.2上使用 sudo systemctl start orbbec_head.service 启动头部相机服务',
            'color_topics': [
                '/ob_camera_head/color/image_raw/compressed',
                '/ob_camera_head/color/image_raw'
            ],
            'depth_topics': [
                '/ob_camera_head/depth/image_raw',
                '/ob_camera_head/depth/image_raw/compressedDepth',
                '/ob_camera_head/depth/image_raw/compressed',
            ],
            'check_topics': [
                '/ob_camera_head/color/image_raw',
                '/ob_camera_head/depth/image_raw'
            ],
            'enabled': True,
            'selected_color_topic': None,
            'selected_depth_topic': None,
            'color_is_compressed': False,
            'depth_is_compressed': False
        },
        'waist': {
            'name': '腰部相机',
            'start_tips': '无界可在41.2，无疆可在41.3，使用 sudo systemctl start orbbec_waist.service 启动腰部相机服务',
            'color_topics': [
                '/ob_camera_waist/color/image_raw/compressed',
                '/ob_camera_waist/color/image_raw'
            ],
            'depth_topics': [
                '/ob_camera_waist/depth/image_raw',
                '/ob_camera_waist/depth/image_raw/compressedDepth',
                '/ob_camera_waist/depth/image_raw/compressed',
            ],
            'check_topics': [
                '/ob_camera_waist/color/image_raw',
                '/ob_camera_waist/depth/image_raw'
            ],
            'enabled': False,
            'selected_color_topic': None,
            'selected_depth_topic': None,
            'color_is_compressed': False,
            'depth_is_compressed': False
        },
        'back': {
            'name': '背部相机',
            'start_tips': '无界可在41.2，无疆可在41.3，使用 sudo systemctl start orbbec_back.service 启动背部相机服务',
            'color_topics': [
                '/ob_camera_back/color/image_raw/compressed',
                '/ob_camera_back/color/image_raw'
            ],
            'depth_topics': [
                '/ob_camera_back/depth/image_raw',
                '/ob_camera_back/depth/image_raw/compressedDepth',
                '/ob_camera_back/depth/image_raw/compressed',
            ],
            'check_topics': [
                '/ob_camera_back/color/image_raw',
                '/ob_camera_back/depth/image_raw'
            ],
            'enabled': False,
            'selected_color_topic': None,
            'selected_depth_topic': None,
            'color_is_compressed': False,
            'depth_is_compressed': False
        }
    }
    
    def __init__(self, camera_name='head'):
        super().__init__('depth_camera_node')
        
        for cam_id in self.CAMERA_CONFIGS:
            self.CAMERA_CONFIGS[cam_id]['enabled'] = (cam_id == camera_name)
        
        self.color_image = None
        self.depth_image = None
        self.save_interval = 1.0
        self.current_camera = camera_name
        self.current_camera_display_name = self.CAMERA_CONFIGS[camera_name]['name']
        
        # 智能定位源代码目录
        self.script_dir = self._find_source_directory()
        self.save_dir = self.script_dir / '10_depth_camera_imgs'
        self.save_dir.mkdir(exist_ok=True)
        self.get_logger().info(f'图片保存目录: {self.save_dir}')
        
        # 初始化 rqt_process 属性
        self.rqt_process = None
        
        if not self.check_camera_services():
            raise RuntimeError('相机服务未启动')
        
        # 启动 rqt_image_view 窗口
        try:
            self.rqt_process = subprocess.Popen(
                ['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info('已启动 rqt_image_view 窗口')
        except Exception as e:
            self.get_logger().warning(f'启动 rqt_image_view 失败: {e}')
        
        self.create_subscribers()
        self.timer = self.create_timer(self.save_interval, self.timer_callback)
        
        self.get_logger().info(f'{self.current_camera_display_name}节点启动成功')
        self.get_logger().info(f'图像将每{self.save_interval}秒自动保存')
    
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
    
    def check_camera_services(self):
        """检查相机服务是否启动
        
        通过检查ROS2话题是否存在来判断相机服务是否正常运行。
        如果服务未启动，会输出错误信息并提示启动命令。
        
        Returns:
            bool: 如果所有必需的话题都存在返回True，否则返回False
        """
        self.get_logger().info('正在检查相机服务...')
        
        # 获取当前ROS2系统中所有可用的话题列表
        topic_names_and_types = self.get_topic_names_and_types()
        available_topics = [name for name, _ in topic_names_and_types]
        
        # 遍历所有相机配置
        for camera_id, config in self.CAMERA_CONFIGS.items():
            # 跳过未启用的相机
            if not config['enabled']:
                continue
            
            camera_name = config['name']
            
            # 检查必需的话题是否存在
            # check_topics 包含彩色图和深度图的基本话题
            missing = [t for t in config['check_topics'] if t not in available_topics]
            
            # 如果有缺失的话题，说明相机服务未启动
            if missing:
                self.get_logger().error(f'{camera_name}服务未启动，缺少: {missing}，{config["start_tips"]}')
                return False
            
            # 遍历彩色图话题候选列表，找到第一个可用的话题
            # 优先使用压缩格式的话题以节省带宽
            for color_topic in config['color_topics']:
                if color_topic in available_topics:
                    config['selected_color_topic'] = color_topic
                    # 根据话题名称判断是否为压缩格式
                    config['color_is_compressed'] = 'compressed' in color_topic
                    self.get_logger().info(f'{camera_name} 彩色图: {color_topic}')
                    break
            
            # 遍历深度图话题候选列表，找到第一个可用的话题
            # 优先使用压缩格式的话题以节省带宽
            for depth_topic in config['depth_topics']:
                if depth_topic in available_topics:
                    config['selected_depth_topic'] = depth_topic
                    # 根据话题名称判断是否为压缩格式
                    config['depth_is_compressed'] = 'compressed' in depth_topic
                    self.get_logger().info(f'{camera_name} 深度图: {depth_topic}')
                    break
        
        return True
    
    def create_subscribers(self):
        """创建ROS2订阅者
        
        为启用的相机创建彩色图和深度图的订阅者。
        根据话题类型（压缩或未压缩）选择不同的消息类型和回调函数。
        使用lambda函数捕获camera_id参数，以便在回调中识别是哪个相机的消息。
        """
        # 遍历所有相机配置
        for camera_id, config in self.CAMERA_CONFIGS.items():
            # 跳过未启用的相机
            if not config['enabled']:
                continue
            
            # 创建彩色图订阅者
            # 根据话题是否为压缩格式，选择订阅CompressedImage或Image消息
            if config['color_is_compressed']:
                # 压缩格式：订阅CompressedImage消息
                self.create_subscription(
                    CompressedImage, 
                    config['selected_color_topic'],
                    # 使用lambda捕获camera_id，确保回调知道是哪个相机的消息
                    lambda msg, cid=camera_id: self.color_callback(msg, cid), 
                    10  # 队列大小
                )
            else:
                # 未压缩格式：订阅Image消息
                self.create_subscription(
                    Image, 
                    config['selected_color_topic'],
                    lambda msg, cid=camera_id: self.color_callback(msg, cid), 
                    10
                )
            
            # 创建深度图订阅者
            # 根据话题是否为压缩格式，选择订阅CompressedImage或Image消息
            if config['depth_is_compressed']:
                # 压缩格式：订阅CompressedImage消息
                self.create_subscription(
                    CompressedImage, 
                    config['selected_depth_topic'],
                    lambda msg, cid=camera_id: self.depth_callback(msg, cid), 
                    10
                )
            else:
                # 未压缩格式：订阅Image消息
                self.create_subscription(
                    Image, 
                    config['selected_depth_topic'],
                    lambda msg, cid=camera_id: self.depth_callback(msg, cid), 
                    10
                )
    
    def color_callback(self, msg, camera_id):
        """彩色图回调函数
        
        处理接收到的彩色图消息，将其转换为PIL Image对象并保存。
        
        Args:
            msg: ROS2图像消息（Image或CompressedImage）
            camera_id: 相机ID，用于获取对应的相机配置
        """
        try:
            config = self.CAMERA_CONFIGS[camera_id]
            
            # 根据消息类型进行不同的解码处理
            if config['color_is_compressed']:
                # 压缩格式：使用PIL直接解码JPEG/PNG等压缩图像
                # BytesIO将字节数据转换为文件流对象
                self.color_image = PILImage.open(BytesIO(msg.data))
                # 确保图像是RGB格式，如果不是则转换
                if self.color_image.mode != 'RGB':
                    self.color_image = self.color_image.convert('RGB')
            else:
                # 未压缩格式：从ROS2 Image消息转换
                self.color_image = self._ros_image_to_pil(msg)
        except Exception as e:
            self.get_logger().error(f'彩色图解码失败: {e}')
    
    def depth_callback(self, msg, camera_id):
        """深度图回调函数
        
        处理接收到的深度图消息，将其转换为彩色热力图并保存。
        
        Args:
            msg: ROS2图像消息（Image或CompressedImage）
            camera_id: 相机ID，用于获取对应的相机配置
        """
        try:
            config = self.CAMERA_CONFIGS[camera_id]
            
            # 根据消息类型进行不同的解码处理
            if config['depth_is_compressed']:
                # 压缩格式：使用PIL解码
                depth_pil = PILImage.open(BytesIO(msg.data))
                # 将深度图转换为彩色热力图以便可视化
                self.depth_image = self._depth_to_colormap(depth_pil)
            else:
                # 未压缩格式：从ROS2 Image消息转换
                depth_pil = self._ros_depth_to_pil(msg)
                # 将深度图转换为彩色热力图以便可视化
                self.depth_image = self._depth_to_colormap(depth_pil)
        except Exception as e:
            self.get_logger().error(f'深度图转换失败: {e}')
    
    def _ros_image_to_pil(self, ros_image):
        """将ROS2彩色图消息转换为PIL Image对象
        
        支持常见的彩色图像编码格式，如bgr8、rgb8等。
        
        Args:
            ros_image: ROS2 Image消息，包含彩色图像数据
            
        Returns:
            PIL.Image: RGB格式的PIL Image对象
        """
        # BGR8格式：OpenCV常用的格式，需要转换为RGB
        if ros_image.encoding == 'bgr8':
            # frombytes创建图像，'raw'表示原始数据，'BGR'指定字节顺序
            image = PILImage.frombytes('RGB', (ros_image.width, ros_image.height), ros_image.data, 'raw', 'BGR')
            return image
        # RGB8格式：已经是RGB格式，直接转换
        elif ros_image.encoding == 'rgb8':
            return PILImage.frombytes('RGB', (ros_image.width, ros_image.height), ros_image.data)
        # 其他格式：默认按RGB处理
        else:
            return PILImage.frombytes('RGB', (ros_image.width, ros_image.height), ros_image.data)
    
    def _ros_depth_to_pil(self, ros_image):
        """将ROS2深度图消息转换为PIL Image对象
        
        支持常见的深度图像编码格式，如16UC1（16位无符号整数）、mono16等。
        
        Args:
            ros_image: ROS2 Image消息，包含深度图像数据
            
        Returns:
            PIL.Image: PIL Image对象，模式为'I;16'（16位灰度）或'L'（8位灰度）
        """
        width, height = ros_image.width, ros_image.height
        
        # 16UC1或mono16：16位深度值，每个像素2字节
        if ros_image.encoding in ['16UC1', 'mono16']:
            return PILImage.frombytes('I;16', (width, height), ros_image.data)
        # 其他格式：默认按8位灰度处理
        else:
            return PILImage.frombytes('L', (width, height), ros_image.data)
    
    def _depth_to_colormap(self, depth_image):
        """将深度图转换为彩色热力图
        
        使用Jet颜色映射将深度值映射为彩色图像，便于可视化深度信息。
        深度值越小显示蓝色，越大显示红色，中间过渡为青色、黄色等。
        
        Args:
            depth_image: PIL Image对象，包含深度数据
            
        Returns:
            PIL.Image: RGB格式的彩色热力图
        """
        # 处理16位深度图
        if depth_image.mode == 'I;16':
            # 获取所有像素的深度值
            depth_array = list(depth_image.getdata())
            
            # 过滤掉无效的深度值（0通常表示无效或超出范围）
            valid = [v for v in depth_array if v > 0]
            
            # 如果没有有效深度值，返回全黑图像
            if not valid:
                return PILImage.new('RGB', depth_image.size, (0, 0, 0))
            
            # 计算深度值的范围，用于归一化
            min_val, max_val = min(valid), max(valid)
            
            # 如果所有深度值相同，归一化后都为0
            if max_val == min_val:
                normalized = [0] * len(depth_array)
            else:
                # 将深度值归一化到0-255范围
                # 公式：normalized = 255 * (value - min) / (max - min)
                # 无效值（0）保持为0
                normalized = [int(255 * (v - min_val) / (max_val - min_val)) if v > 0 else 0 
                            for v in depth_array]
            
            # 创建8位灰度图像
            gray_image = PILImage.new('L', depth_image.size)
            gray_image.putdata(normalized)
        else:
            # 非16位深度图，直接转换为灰度图
            gray_image = depth_image.convert('L')
        
        # 创建RGB图像用于存储彩色热力图
        rgb_image = PILImage.new('RGB', gray_image.size)
        
        # 获取所有像素的灰度值
        pixels = list(gray_image.getdata())
        
        # 使用Jet颜色映射将每个灰度值转换为RGB颜色
        colored = [self._jet_colormap(p) for p in pixels]
        
        # 将彩色像素数据写入图像
        rgb_image.putdata(colored)
        return rgb_image
    
    def _jet_colormap(self, value):
        """Jet颜色映射函数
        
        将0-255的灰度值映射到Jet颜色空间的RGB值。
        Jet颜色映射的特点：
        - 0-63: 蓝色到青色（蓝色逐渐减弱，绿色逐渐增强）
        - 64-127: 青色到黄色（绿色保持，蓝色逐渐减弱，红色逐渐增强）
        - 128-191: 黄色到红色（绿色逐渐减弱，红色保持）
        - 192-255: 红色到深红（红色保持，绿色逐渐减弱）
        
        Args:
            value: 灰度值，范围0-255
            
        Returns:
            tuple: RGB颜色值，格式为(R, G, B)，每个分量范围0-255
        """
        # 蓝色到青色区间（0-63）
        if value < 64:
            # 蓝色从128递增到255，绿色和红色保持为0
            return (0, 0, 128 + value * 2)
        # 青色到黄色区间（64-127）
        elif value < 128:
            # 绿色从0递增到255，蓝色保持255，红色从0递增到255
            return (0, (value - 64) * 4, 255)
        # 黄色到红色区间（128-191）
        elif value < 192:
            # 红色从0递增到255，绿色保持255，蓝色从255递减到0
            return ((value - 128) * 4, 255, 255 - (value - 128) * 4)
        # 红色到深红区间（192-255）
        else:
            # 红色保持255，绿色从255递减到0，蓝色保持0
            return (255, 255 - (value - 192) * 4, 0)
    
    def timer_callback(self):
        if self.color_image and self.depth_image:
            self.save_images()
        else:
            self.get_logger().info(f'等待{self.current_camera_display_name}数据...')
    
    def _cleanup_old_images(self, max_images=100):
        """清理旧图片，保持最多max_images张
        
        当保存的图片数量超过限制时，删除最旧的图片以节省磁盘空间。
        按文件的修改时间排序，删除最旧的文件。
        
        Args:
            max_images: 保留的最大图片数量，默认100张
        """
        try:
            # 获取当前相机的所有图片文件
            # 文件名格式：{camera_name}_{timestamp}.jpg
            image_files = sorted(
                [f for f in self.save_dir.glob(f'{self.current_camera}_*.jpg')],
                key=lambda x: x.stat().st_mtime  # 按修改时间排序
            )
            
            # 如果超过最大数量，删除最旧的图片
            if len(image_files) >= max_images:
                # 计算需要删除的文件数量
                files_to_delete = image_files[:len(image_files) - max_images + 1]
                for f in files_to_delete:
                    # 删除文件
                    f.unlink()
                    self.get_logger().info(f'已删除旧图片: {f.name}')
        except Exception as e:
            self.get_logger().error(f'清理旧图片失败: {e}')
    
    def save_images(self):
        """保存彩色图和深度图的拼接图
        
        将彩色图和深度图水平拼接成一张图片并保存。
        深度图会调整大小以匹配彩色图的尺寸。
        保存前会清理旧图片以控制磁盘使用。
        """
        try:
            # 清理旧图片，保持最多100张
            self._cleanup_old_images(max_images=100)
            
            # 获取彩色图的尺寸
            color_w, color_h = self.color_image.size
            
            # 将深度图调整到与彩色图相同的尺寸
            # 使用LANCZOS重采样算法，质量较好
            depth_resized = self.depth_image.resize((color_w, color_h), PILImage.LANCZOS)
            
            # 创建拼接图像，宽度是两张图之和，高度与原图相同
            combined = PILImage.new('RGB', (color_w * 2, color_h))
            
            # 将彩色图粘贴到左侧
            combined.paste(self.color_image, (0, 0))
            
            # 将深度图粘贴到右侧
            combined.paste(depth_resized, (color_w, 0))
            
            # 生成文件名：相机名_时间戳.jpg
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'{self.current_camera}_{timestamp}.jpg'
            filepath = self.save_dir / filename
            
            # 保存为JPEG格式，质量95%
            combined.save(str(filepath), 'JPEG', quality=95)
            self.get_logger().info(f'已保存: {filename}')
        except Exception as e:
            self.get_logger().error(f'保存失败: {e}')
    
    def destroy_node(self):
        """销毁节点
        
        在节点销毁时清理资源，包括关闭rqt_image_view窗口。
        必须在调用父类destroy_node之前清理子进程。
        """
        # 关闭 rqt_image_view 窗口
        self._cleanup_rqt_process()
        # 调用父类的destroy_node方法
        super().destroy_node()
    
    def _cleanup_rqt_process(self):
        """清理 rqt_image_view 进程
        
        优雅地关闭rqt_image_view窗口，如果进程无法正常终止则强制杀死。
        使用两层保护机制：先尝试terminate（发送SIGTERM），如果失败则kill（发送SIGKILL）。
        """
        # 检查是否存在rqt_process属性且进程不为空
        if hasattr(self, 'rqt_process') and self.rqt_process:
            try:
                # 第一步：尝试优雅终止进程（发送SIGTERM信号）
                self.rqt_process.terminate()
                # 等待最多2秒让进程正常退出
                self.rqt_process.wait(timeout=2)
            except:
                # 如果terminate失败或超时，尝试强制杀死进程
                try:
                    # 第二步：强制杀死进程（发送SIGKILL信号）
                    self.rqt_process.kill()
                except:
                    # 如果kill也失败，忽略错误（进程可能已经退出）
                    pass
            finally:
                # 无论成功与否，都将进程引用设为None
                self.rqt_process = None


def main(args=None):
    """主函数
    
    解析命令行参数，创建并运行深度相机节点。
    处理各种异常情况，确保资源正确释放。
    
    Args:
        args: 命令行参数，通常为None（由ROS2自动处理）
    """
    import argparse
    import sys
    
    # 创建参数解析器
    parser = argparse.ArgumentParser(description='深度相机保存节点')
    # 添加--camera参数，可选值为head、waist、back，默认为head
    parser.add_argument('-c', '--camera', choices=['head', 'waist', 'back'], 
                       default='head', help='选择相机')
    
    # 过滤掉ROS2内部参数（以__开头的参数）
    # 这些参数由ROS2系统自动添加，不需要传递给argparse
    filtered_args = [arg for arg in sys.argv[1:] if not arg.startswith('__')]
    parsed_args = parser.parse_args(filtered_args)
    
    # 初始化ROS2
    rclpy.init(args=args)
    node = None
    
    try:
        # 创建深度相机节点实例
        node = DepthCameraNode(camera_name=parsed_args.camera)
        # 进入ROS2事件循环，持续运行节点
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获Ctrl+C中断信号，静默处理（正常退出）
        pass
    except Exception as e:
        # 捕获其他异常，打印错误信息
        print(f'错误: {e}')
        # 即使初始化失败，也要尝试清理 rqt_image_view
        if node is not None and hasattr(node, '_cleanup_rqt_process'):
            try:
                node._cleanup_rqt_process()
            except:
                pass
    finally:
        # 清理阶段：确保所有资源正确释放
        # 先销毁节点（会自动清理 rqt_image_view），再关闭rclpy
        if node is not None:
            try:
                node.destroy_node()
            except:
                # 如果 destroy_node 失败，直接清理 rqt_image_view
                try:
                    if hasattr(node, '_cleanup_rqt_process'):
                        node._cleanup_rqt_process()
                except:
                    pass
        
        # 关闭ROS2系统
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass


if __name__ == '__main__':
    main()
