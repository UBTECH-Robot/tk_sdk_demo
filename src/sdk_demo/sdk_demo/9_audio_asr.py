#!/usr/bin/env python3
"""
TTS 文本转语音节点 (Text-to-Speech Node)
==========================================

此模块提供 TTS 文本转语音功能，通过调用 lyre 服务将文本转换为语音并播放。

主要功能:
    1. 调用远程 TTS 服务 (/audio_play/play_text)
    2. 支持流式播放和打断播放
    3. 服务可用性检查
    4. 同步和异步调用模式

服务依赖:
    - 服务名: /audio_play/play_text
    - 服务提供者: lyre (运行在 192.168.41.2)
    - 环境配置: 需要先执行 `. ~/lyre_ros2/install/setup.bash`

使用示例:
    # 配置环境变量
    # 如果在天工的orin板上运行本脚本，用这个命令设置 lyre_ros2 环境变量
    source ~/lyre_ros2/install/setup.bash

    # 如果在天工的x86板上运行本脚本，用这个命令设置 lyre_ros2 环境变量
    source ~/ros2ws/install/setup.bash
    
    # 启动节点
    ros2 run sdk_demo audio_asr

注意事项:
    - 需要先确保192.168.41.2上的 lyre 服务已启动
        cd /home/nvidia/lyre_ros2 && . install/setup.bash && ros2 launch lyre audio.launch.py

"""

import rclpy
from rclpy.node import Node
from rclpy.client import Client
import sys
import time
import uuid

try:
    from lyre_msgs.srv import PlayText
except ImportError:
    print("=" * 70)
    print("错误: 无法导入 lyre_msgs.srv.PlayText")
    print("=" * 70)
    print("请先配置 lyre_ros2 环境变量:")
    print("  source ~/lyre_ros2/install/setup.bash # 天工 orin 板")
    print("")
    print("  source ~/ros2ws/install/setup.bash # 天工 x86 板")
    print("")
    print("然后重新运行此节点:")
    print("  ros2 run sdk_demo audio_asr")
    print("=" * 70)
    sys.exit(1)


class TTSNode(Node):
    """
    TTS 文本转语音节点
    
    通过调用远程 lyre 服务实现文本转语音功能。
    支持流式播放、强制打断等高级特性。
    
    属性:
        client: 服务客户端
        service_name: TTS 服务名称
        service_timeout: 服务等待超时时间（秒）
    
    方法:
        play_text: 播放文本（同步）
        play_text_async: 播放文本（异步）
        check_service: 检查服务是否可用
    """
    
    def __init__(self):
        """
        初始化 TTS 节点
        
        初始化流程:
            1. 创建节点
            2. 创建服务客户端
            3. 检查服务是否可用
            4. 如果服务不可用，提示并退出
        
        退出条件:
            - 服务在超时时间内未发现
            - 提示用户检查 lyre 服务状态
        """
        super().__init__('tts_node')
        
        # 服务配置
        self.service_name = '/audio_play/play_text'
        self.service_timeout = 10.0  # 等待服务的超时时间（秒）
        
        self.get_logger().info('正在初始化 TTS 节点...')
        
        # 创建服务客户端
        self.client = self.create_client(PlayText, self.service_name)
        self.get_logger().info(f'已创建服务客户端: {self.service_name}')
        
        # 检查服务是否存在
        if not self.check_service():
            self.get_logger().error('=' * 60)
            self.get_logger().error(f'未发现服务: {self.service_name}')
            self.get_logger().error('请确认 192.168.41.2 上的 lyre 服务是否启动')
            self.get_logger().error('检查项:')
            self.get_logger().error('  1. lyre 服务是否正在运行')
            self.get_logger().error('  2. 网络连接是否正常 (ping 192.168.41.2)')
            self.get_logger().error('=' * 60)
            sys.exit(1)
        
        self.get_logger().info('TTS 服务已就绪，节点初始化完成')
    
    def check_service(self) -> bool:
        """
        检查 TTS 服务是否可用
        
        等待服务在超时时间内上线。
        
        返回:
            bool: True 表示服务可用，False 表示超时未发现
        
        超时时间:
            由 self.service_timeout 设置（默认 10 秒）
        
        日志:
            - 每秒输出等待信息
            - 服务发现后输出确认信息
        """
        self.get_logger().info(f'正在等待服务: {self.service_name}')
        self.get_logger().info(f'超时时间: {self.service_timeout} 秒')
        
        # 等待服务可用
        service_available = self.client.wait_for_service(timeout_sec=self.service_timeout)
        
        if service_available:
            self.get_logger().info(f'服务已发现: {self.service_name}')
            return True
        else:
            self.get_logger().warning(f'服务等待超时 ({self.service_timeout} 秒)')
            return False
    
    def play_text(
        self, 
        text: str, 
        force: bool = False, 
        sid: str = None,
        seq: int = 0,
        last: bool = True,
        token: str = "",
        output: str = "",
        timeout: float = 30.0
    ) -> tuple:
        """
        播放文本（同步调用）
        
        将文本发送到 TTS 服务进行语音合成和播放。
        此方法会阻塞直到服务响应或超时。
        
        参数:
            text (str): 要转换为语音的文本内容
            force (bool): 是否强制播放（打断当前播放），默认 False
            sid (str): 流标识符，如果为 None 则自动生成 UUID
            seq (int): 序列号（用于流式传输），默认 0
            last (bool): 是否为最后一个包，默认 True
            token (str): 系统 API 令牌（应用不可用），默认空
            output (str): 输出配置（应用不可用），默认空
            timeout (float): 服务调用超时时间（秒），默认 30.0
        
        返回:
            tuple: (success: bool, sid: str, message: str)
                - success: 是否成功
                - sid: 播放流 ID
                - message: 状态消息
        """
        # 生成流 ID（如果未提供）
        if sid is None:
            sid = str(uuid.uuid4())
        
        self.get_logger().info(f'正在播放文本: "{text[:50]}..." (force={force}, sid={sid})')
        
        # 创建请求
        request = PlayText.Request()
        request.sid = sid
        request.seq = seq
        request.last = last
        request.force = force
        request.text = text
        request.token = token
        request.output = output
        
        try:
            # 同步调用服务
            future = self.client.call_async(request)
            
            # 等待响应
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
                
                # 检查响应状态码
                if response.code == 0:  # CODE_OK
                    self.get_logger().info(f'播放成功: {response.message} (sid={response.sid})')
                    return (True, response.sid, response.message)
                elif response.code == 1:  # CODE_INVALID_PARAMS
                    self.get_logger().error(f'参数无效: {response.message}')
                    return (False, response.sid, response.message)
                else:  # CODE_FAILED
                    self.get_logger().error(f'播放失败: {response.message}')
                    return (False, response.sid, response.message)
            else:
                msg = f'服务调用超时 ({timeout} 秒)'
                self.get_logger().error(msg)
                return (False, sid, msg)
                
        except Exception as e:
            msg = f'服务调用异常: {e}'
            self.get_logger().error(msg)
            import traceback
            self.get_logger().error(traceback.format_exc())
            return (False, sid, msg)
    
    def play_text_async(
        self,
        text: str,
        callback=None,
        force: bool = False,
        sid: str = None,
        **kwargs
    ):
        """
        播放文本（异步调用）
        
        将文本发送到 TTS 服务，不阻塞等待响应。
        可以提供回调函数处理响应结果。
        
        参数:
            text (str): 要转换为语音的文本内容
            callback (callable): 响应回调函数，签名为 callback(future)
            force (bool): 是否强制播放，默认 False
            sid (str): 流标识符，如果为 None 则自动生成
            **kwargs: 其他参数（seq, last, token, output）
        """
        # 生成流 ID（如果未提供）
        if sid is None:
            sid = str(uuid.uuid4())
        
        self.get_logger().info(f'异步播放文本: "{text[:50]}..." (sid={sid})')
        
        # 创建请求
        request = PlayText.Request()
        request.sid = sid
        request.seq = kwargs.get('seq', 0)
        request.last = kwargs.get('last', True)
        request.force = force
        request.text = text
        request.token = kwargs.get('token', '')
        request.output = kwargs.get('output', '')
        
        # 异步调用服务
        future = self.client.call_async(request)
        
        # 如果提供了回调函数，添加回调
        if callback is not None:
            future.add_done_callback(callback)
        
        return future


def main(args=None):
    """
    TTS 节点主函数
    
    初始化 ROS2 上下文，创建 TTS 节点，并提供交互式测试。
    
    使用方法:
        ros2 run sdk_demo audio_asr
    
    交互模式:
        - 输入文本后按回车播放
        - 输入 'exit' 或 'quit' 退出
        - 输入 '!' 开头的文本进行强制打断播放
    
    退出:
        按 Ctrl+C 或输入 exit/quit
    """
    rclpy.init(args=args)
    
    try:
        # 创建节点
        node = TTSNode()
        
        # 交互式测试模式
        print("\n" + "=" * 60)
        print("TTS 文本转语音节点已启动")
        print("=" * 60)
        print("使用说明:")
        print("  - 输入文本后按回车进行播放")
        print("  - 输入 '!' 开头的文本强制打断当前播放 (例如: !紧急消息)")
        print("  - 输入 'exit' 或 'quit' 退出")
        print("=" * 60 + "\n")
        
        # 创建单独的线程处理 ROS2 回调
        import threading
        spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
        spin_thread.start()
        
        # 交互式输入循环
        while rclpy.ok():
            try:
                user_input = input("请输入要播放的文本: ").strip()
                
                if not user_input:
                    continue
                
                # 退出命令
                if user_input.lower() in ['exit', 'quit', 'q']:
                    print("正在退出...")
                    break
                
                # 强制打断播放（以 '!' 开头）
                force = False
                if user_input.startswith('!'):
                    force = True
                    user_input = user_input[1:].strip()
                    if not user_input:
                        print("警告: 强制标记后没有文本内容")
                        continue
                
                # 调用 TTS 播放
                success, sid, message = node.play_text(user_input, force=force)
                
                if success:
                    print(f"✓ 播放成功 (SID: {sid})")
                else:
                    print(f"✗ 播放失败: {message}")
                    
            except EOFError:
                # Ctrl+D
                print("\n检测到 EOF，正在退出...")
                break
            except KeyboardInterrupt:
                # Ctrl+C
                print("\n接收到键盘中断信号")
                break
        
    except SystemExit:
        # 节点初始化失败（服务不可用）
        pass
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
