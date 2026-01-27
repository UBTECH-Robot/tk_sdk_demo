#!/usr/bin/env python3
"""
深度相机保存节点（无GUI版本）
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
    """深度相机节点类 - 无GUI版本"""
    
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
                if (path / 'install').exists() and (path / 'src').exists():
                    # 找到工作空间根目录
                    workspace_root = path
                    # 构造源代码路径: workspace/src/sdk_demo/sdk_demo/
                    source_dir = workspace_root / 'src' / 'sdk_demo' / 'sdk_demo'
                    if source_dir.exists():
                        return source_dir
                    break
                path = path.parent
        
        # 情况2: 如果当前已经在src目录运行（开发时直接运行）
        # 路径类似: /path/to/workspace/src/pkg/pkg/file.py
        if 'src' in current_file.parts:
            # 当前文件的父目录就是我们要的目录
            return current_file.parent
        
        # 备用方案: 返回当前文件所在目录
        return current_file.parent
    
    def check_camera_services(self):
        self.get_logger().info('正在检查相机服务...')
        topic_names_and_types = self.get_topic_names_and_types()
        available_topics = [name for name, _ in topic_names_and_types]
        
        for camera_id, config in self.CAMERA_CONFIGS.items():
            if not config['enabled']:
                continue
            
            camera_name = config['name']
            missing = [t for t in config['check_topics'] if t not in available_topics]
            
            if missing:
                self.get_logger().error(f'{camera_name}服务未启动，缺少: {missing}，{config["start_tips"]}')
                return False
            
            for color_topic in config['color_topics']:
                if color_topic in available_topics:
                    config['selected_color_topic'] = color_topic
                    config['color_is_compressed'] = 'compressed' in color_topic
                    self.get_logger().info(f'{camera_name} 彩色图: {color_topic}')
                    break
            
            for depth_topic in config['depth_topics']:
                if depth_topic in available_topics:
                    config['selected_depth_topic'] = depth_topic
                    config['depth_is_compressed'] = 'compressed' in depth_topic
                    self.get_logger().info(f'{camera_name} 深度图: {depth_topic}')
                    break
        
        return True
    
    def create_subscribers(self):
        for camera_id, config in self.CAMERA_CONFIGS.items():
            if not config['enabled']:
                continue
            
            if config['color_is_compressed']:
                self.create_subscription(CompressedImage, config['selected_color_topic'],
                                       lambda msg, cid=camera_id: self.color_callback(msg, cid), 10)
            else:
                self.create_subscription(Image, config['selected_color_topic'],
                                       lambda msg, cid=camera_id: self.color_callback(msg, cid), 10)
            
            if config['depth_is_compressed']:
                self.create_subscription(CompressedImage, config['selected_depth_topic'],
                                       lambda msg, cid=camera_id: self.depth_callback(msg, cid), 10)
            else:
                self.create_subscription(Image, config['selected_depth_topic'],
                                       lambda msg, cid=camera_id: self.depth_callback(msg, cid), 10)
    
    def color_callback(self, msg, camera_id):
        try:
            config = self.CAMERA_CONFIGS[camera_id]
            if config['color_is_compressed']:
                self.color_image = PILImage.open(BytesIO(msg.data))
                if self.color_image.mode != 'RGB':
                    self.color_image = self.color_image.convert('RGB')
            else:
                self.color_image = self._ros_image_to_pil(msg)
        except Exception as e:
            self.get_logger().error(f'彩色图解码失败: {e}')
    
    def depth_callback(self, msg, camera_id):
        try:
            config = self.CAMERA_CONFIGS[camera_id]
            if config['depth_is_compressed']:
                depth_pil = PILImage.open(BytesIO(msg.data))
                self.depth_image = self._depth_to_colormap(depth_pil)
            else:
                depth_pil = self._ros_depth_to_pil(msg)
                self.depth_image = self._depth_to_colormap(depth_pil)
        except Exception as e:
            self.get_logger().error(f'深度图转换失败: {e}')
    
    def _ros_image_to_pil(self, ros_image):
        if ros_image.encoding == 'bgr8':
            image = PILImage.frombytes('RGB', (ros_image.width, ros_image.height), ros_image.data, 'raw', 'BGR')
            return image
        elif ros_image.encoding == 'rgb8':
            return PILImage.frombytes('RGB', (ros_image.width, ros_image.height), ros_image.data)
        else:
            return PILImage.frombytes('RGB', (ros_image.width, ros_image.height), ros_image.data)
    
    def _ros_depth_to_pil(self, ros_image):
        width, height = ros_image.width, ros_image.height
        if ros_image.encoding in ['16UC1', 'mono16']:
            return PILImage.frombytes('I;16', (width, height), ros_image.data)
        else:
            return PILImage.frombytes('L', (width, height), ros_image.data)
    
    def _depth_to_colormap(self, depth_image):
        if depth_image.mode == 'I;16':
            depth_array = list(depth_image.getdata())
            valid = [v for v in depth_array if v > 0]
            if not valid:
                return PILImage.new('RGB', depth_image.size, (0, 0, 0))
            
            min_val, max_val = min(valid), max(valid)
            if max_val == min_val:
                normalized = [0] * len(depth_array)
            else:
                normalized = [int(255 * (v - min_val) / (max_val - min_val)) if v > 0 else 0 
                            for v in depth_array]
            
            gray_image = PILImage.new('L', depth_image.size)
            gray_image.putdata(normalized)
        else:
            gray_image = depth_image.convert('L')
        
        rgb_image = PILImage.new('RGB', gray_image.size)
        pixels = list(gray_image.getdata())
        colored = [self._jet_colormap(p) for p in pixels]
        rgb_image.putdata(colored)
        return rgb_image
    
    def _jet_colormap(self, value):
        if value < 64:
            return (0, 0, 128 + value * 2)
        elif value < 128:
            return (0, (value - 64) * 4, 255)
        elif value < 192:
            return ((value - 128) * 4, 255, 255 - (value - 128) * 4)
        else:
            return (255, 255 - (value - 192) * 4, 0)
    
    def timer_callback(self):
        if self.color_image and self.depth_image:
            self.save_images()
        else:
            self.get_logger().info(f'等待{self.current_camera_display_name}数据...')
    
    def _cleanup_old_images(self, max_images=100):
        """清理旧图片，保持最多max_images张
        
        Args:
            max_images: 保留的最大图片数量，默认100张
        """
        try:
            # 获取所有图片文件，按修改时间排序
            image_files = sorted(
                [f for f in self.save_dir.glob(f'{self.current_camera}_*.jpg')],
                key=lambda x: x.stat().st_mtime
            )
            
            # 如果超过最大数量，删除最旧的
            if len(image_files) >= max_images:
                files_to_delete = image_files[:len(image_files) - max_images + 1]
                for f in files_to_delete:
                    f.unlink()
                    self.get_logger().info(f'已删除旧图片: {f.name}')
        except Exception as e:
            self.get_logger().error(f'清理旧图片失败: {e}')
    
    def save_images(self):
        try:
            # 清理旧图片，保持最多100张
            self._cleanup_old_images(max_images=100)
            
            color_w, color_h = self.color_image.size
            depth_resized = self.depth_image.resize((color_w, color_h), PILImage.LANCZOS)
            
            combined = PILImage.new('RGB', (color_w * 2, color_h))
            combined.paste(self.color_image, (0, 0))
            combined.paste(depth_resized, (color_w, 0))
            
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'{self.current_camera}_{timestamp}.jpg'
            filepath = self.save_dir / filename
            
            combined.save(str(filepath), 'JPEG', quality=95)
            self.get_logger().info(f'已保存: {filename}')
        except Exception as e:
            self.get_logger().error(f'保存失败: {e}')
    
    def destroy_node(self):
        # 关闭 rqt_image_view 窗口
        self._cleanup_rqt_process()
        super().destroy_node()
    
    def _cleanup_rqt_process(self):
        """清理 rqt_image_view 进程"""
        if hasattr(self, 'rqt_process') and self.rqt_process:
            try:
                self.rqt_process.terminate()
                self.rqt_process.wait(timeout=2)
            except:
                try:
                    self.rqt_process.kill()
                except:
                    pass
            finally:
                self.rqt_process = None


def main(args=None):
    import argparse
    import sys
    
    parser = argparse.ArgumentParser(description='深度相机保存节点')
    parser.add_argument('-c', '--camera', choices=['head', 'waist', 'back'], 
                       default='head', help='选择相机')
    
    filtered_args = [arg for arg in sys.argv[1:] if not arg.startswith('__')]
    parsed_args = parser.parse_args(filtered_args)
    
    rclpy.init(args=args)
    node = None
    
    try:
        node = DepthCameraNode(camera_name=parsed_args.camera)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # 静默处理 Ctrl+C
    except Exception as e:
        print(f'错误: {e}')
        # 即使初始化失败，也要尝试清理 rqt_image_view
        if node is not None and hasattr(node, '_cleanup_rqt_process'):
            try:
                node._cleanup_rqt_process()
            except:
                pass
    finally:
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
        
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass


if __name__ == '__main__':
    main()
