
#!/usr/bin/env python3
"""
音频保存示例

此示例通过socket连接接收来自RK3588s板的原生音频数据，
并将其保存为PCM和WAV格式文件。

使用方法：
    1. 基础启动（不保存音频）：
       ros2 run sdk_demo audio_saver
    
    2. 启用音频保存功能：
       ros2 run sdk_demo audio_saver --ros-args -p save_audio:=true
    
    3. 编译命令：
       colcon build
       source install/setup.bash

    4. 播放保存的PCM文件（Linux系统）：
       aplay -f S16_LE -r 16000 -c 1 audio_134758915.pcm

    5. 调整音量：
       # 音量调整到100%
       amixer set Speaker 100%
       
       # 音量调整到50%
       amixer set Speaker 50%

详细音频协议文档：https://aiui-doc.xf-yun.com/project-1/doc-392/

依赖项：
    - rclpy: ROS2 Python客户端库
    - wave: 音频文件处理
    - socket: 网络通信
"""

import os
import threading
import wave
from queue import Queue, Empty
import rclpy
from rclpy.node import Node
import signal
from pathlib import Path
from datetime import datetime
import struct
from socket import *
import logging
from logging import StreamHandler, Formatter

LOG_LEVEL = logging.INFO
# 日志级别选项：DEBUG（详细调试信息）/ INFO（一般信息）/ WARNING（警告）/ ERROR（错误）/ CRITICAL（严重错误）

def setup_logger(name=None):
    """
    创建并配置一个logger对象
    
    功能：
        - 创建具有统一格式和日志级别的logger
        - 支持多个logger的创建，通过name区分
        - 避免重复添加handler
    
    参数：
        name (str, optional): 日志记录器名称，None表示使用root logger
    
    返回值：
        logging.Logger: 配置好的logger对象
    
    示例：
        logger = setup_logger(__name__)
        logger.info("这是一条信息")
        logger.debug("这是调试信息")
    """
    logger = logging.getLogger(name)
    logger.setLevel(LOG_LEVEL)

    # 检查是否已有handler，避免重复添加
    if not logger.hasHandlers():
        handler = StreamHandler()
        # 格式：时间 [日志级别] logger名称: 消息
        formatter = Formatter(
            fmt="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
            datefmt="%m-%d %H:%M:%S"
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    return logger

logger = setup_logger(__name__)

class SocketAudioSaver(Node):
    """
    ROS2节点：音频保存器
    
    此节点通过socket接收来自AI服务的音频流，并将其保存为文件。
    
    功能特性：
        - 通过socket连接接收实时音频数据
        - 支持语音活动检测(VAD)来判断说话状态
        - 自动创建目录和清理旧文件
        - 支持多线程并发接收和保存
        - 音频保存为PCM和WAV两种格式
    
    参数：
        save_audio (bool, default=False): 是否启用音频保存功能
                                         通过ROS参数设置：-p save_audio:=true
    
    属性：
        audio_provider (SocketAudioProvider): socket音频提供者
        audio_queue (Queue): 存储待保存音频的队列
        audio_buffer (bytearray): 临时音频缓冲区
        sample_rate (int): 采样率，默认16000Hz
        channels (int): 声道数，默认为单声道(1)
        bit_depth (int): 位深度，默认16位
    """
    
    def __init__(self):        
        super().__init__('audio_saver')
        
        # 声明ROS参数：save_audio
        self.declare_parameter('save_audio', False)
        self.save_audio = self.get_parameter('save_audio').get_parameter_value().bool_value

        # 初始化socket音频提供者（连接到本地服务）
        self.audio_provider = SocketAudioProvider('10.42.0.127', 9080)
        
        # 停止事件标志，用于优雅关闭线程
        self.stop_event = threading.Event()

        # 设置音频文件保存目录
        self.audio_files_dir_in_path = Path("saved_data") / "audio_files"
        self.audio_files_dir = (self.audio_files_dir_in_path).resolve()

        # 当前处理的PCM和WAV文件路径
        self.pcm_file = None
        self.wav_file = None

        self.audio = None
        self.stream = None
        
        # 音频参数配置
        self.sample_rate = 16000  # 采样率：16kHz
        self.channels = 1          # 单声道
        self.bit_depth = 16        # 16位音频
        
        # 音频缓冲区：用于临时存储VAD状态为1、2、3的音频数据
        self.audio_buffer = bytearray()
        
        if self.save_audio:
            # 启用音频保存：创建队列和保存线程
            self.audio_queue = Queue()
            # 清理旧的音频文件
            self.clear_old_files()
            # 启动音频保存线程
            self.saving_thread = threading.Thread(target=self.keep_saving_wav_pcm_file)
            self.saving_thread.start()
            self.get_logger().info(f"音频保存功能已启用，音频文件将保存在{self.audio_files_dir}目录下")
        else:
            # 未启用保存功能
            self.audio_queue = None
            self.saving_thread = None
            self.get_logger().info(f"音频保存功能未启用，使用 ros2 run sdk_demo audio_saver --ros-args -p save_audio:=true 命令可启用音频保存功能")

        # 启动音频接收线程（总是运行）
        self.receive_pub_thread = threading.Thread(target=self.keep_receiving_publish_audio)
        self.receive_pub_thread.start()    
        self.get_logger().info("AudioSaver 节点成功启动")

    def ensure_directories(self):
        """
        确保音频文件保存目录存在
        
        功能：
            - 如果目录不存在，则创建它（包括所有父目录）
            - 使用exist_ok=True避免目录已存在时的错误
        """
        self.audio_files_dir_in_path.mkdir(parents=True, exist_ok=True)

    def clear_old_files(self):
        """
        清理旧的音频文件
        
        功能：
            - 每次启动时清空音频文件目录
            - 防止旧数据积累
            - 异常时继续执行，不中断启动流程
        """
        self.ensure_directories()

        for file in os.listdir(self.audio_files_dir):
            try:
                os.remove(os.path.join(self.audio_files_dir, file))
            except Exception as e:
                self.get_logger().info(f"删除文件失败: {e}")

    def get_new_name(self, dir_name):
        """
        生成新的音频文件名
        
        功能：
            - 基于当前时间戳生成唯一的文件名
            - 同时生成PCM和WAV格式的文件路径
        
        参数：
            dir_name (str): 保存音频文件的目录路径
        
        说明：
            文件名格式：audio_HHMMSSMMM（时分秒+3位毫秒）
            例如：audio_143022456.pcm
        """
        # 生成时分秒+毫秒的时间戳
        timestamp = datetime.now().strftime('%H%M%S%f')[:-3]  # 时分秒+毫秒（保留3位）
        filename = os.path.join(dir_name, f"audio_{timestamp}")
        self.pcm_file = f"{filename}.pcm"  # PCM格式：原始音频数据
        self.wav_file = f"{filename}.wav"  # WAV格式：带音频信息头的标准格式
    
    def save_wav_file(self, audio_data):
        """
        保存音频数据为WAV格式文件
        
        WAV格式是一种标准的音频文件格式，包含音频的元数据（采样率、位深度等）。
        
        参数：
            audio_data (bytes): 原始PCM音频数据
        
        功能：
            - 生成新的文件名
            - 使用wave模块写入音频数据
            - 设置正确的音频参数
            - 包含完整的错误处理
        
        异常：
            - 若audio_data为空，记录警告并返回
            - 文件写入失败时记录错误
        """
        if not audio_data:
            self.get_logger().info("没有音频数据可保存")
            return
        
        self.get_new_name(self.audio_files_dir)
        
        try:
            with wave.open(self.wav_file, 'wb') as wf:
                wf.setnchannels(self.channels)                    # 设置声道数：1（单声道）
                wf.setsampwidth(self.bit_depth // 8)              # 设置采样宽度：2字节（16bit/8）
                wf.setframerate(self.sample_rate)                 # 设置帧率：16000Hz
                wf.writeframes(audio_data)                        # 写入音频数据
            self.get_logger().info(f"已转换为WAV格式: {self.wav_file}")
        except Exception as e:
            self.get_logger().info(f"保存WAV文件失败: {e}")

    def save_pcm_file(self, audio_data):
        """
        保存音频数据为PCM格式文件
        
        PCM(Pulse Code Modulation)是原始的音频格式，不包含任何头部信息，
        只是原始的音频样本数据。
        
        参数：
            audio_data (bytes): 原始音频数据
        
        功能：
            - 使用追加模式写入文件（支持多次调用累积）
            - 简单直接，无需处理音频格式信息
            - 包含完整的错误处理
        
        异常：
            - 若audio_data为空，记录警告并返回
            - 文件写入失败时记录错误
        """
        if not audio_data:
            self.get_logger().info("没有音频数据可保存")
            return        
        
        try:
            with open(self.pcm_file, 'ab') as pcm_file:  # 'ab'：追加二进制模式
                pcm_file.write(audio_data)
            self.get_logger().info(f"已保存PCM文件: {self.pcm_file}")
        except Exception as e:
            self.get_logger().info(f"保存PCM文件失败: {e}")

    def keep_saving_wav_pcm_file(self):
        """
        持续保存音频文件的线程函数
        
        功能：
            - 在独立线程中运行
            - 从audio_queue队列持续获取音频数据
            - 同时保存为WAV和PCM两种格式
            - 优雅处理线程停止信号
        
        流程：
            1. 等待队列中的音频数据（超时2秒）
            2. 获取数据后确保目录存在
            3. 同时保存为WAV和PCM格式
            4. 标记队列任务完成
            5. 超时或异常时继续循环
        
        说明：
            - 此方法在save_audio参数为True时由__init__启动
            - 通过stop_event.set()可优雅停止此线程
        """
        while not self.stop_event.is_set():
            try:
                # 等待队列中的数据，超时2秒
                audio_data = self.audio_queue.get(timeout=2)
                try:
                    if audio_data is None:
                        continue
                    self.ensure_directories()

                    # 同时保存WAV和PCM格式
                    self.save_wav_file(audio_data)
                    self.save_pcm_file(audio_data)
                except Exception as e:
                    self.get_logger().info(f"保存音频文件时发生错误: {e}")
                finally:
                    # 标记队列任务完成
                    self.audio_queue.task_done()
            except Empty:
                # 队列为空，超时返回，继续循环
                continue
            except Exception as e:
                self.get_logger().info(f'keep_saving_wav_file循环错误: {e}')

    def close(self):
        """
        优雅关闭节点
        
        功能：
            - 设置停止事件，通知所有线程停止
            - 关闭socket连接
            - 等待线程安全退出
        
        说明：
            - 应在signal handler或finally块中调用
            - 确保资源的正确释放
        """
        self.stop_event.set()
        self.audio_provider.close()
        if self.saving_thread and self.saving_thread.is_alive():
            self.saving_thread.join()

    def keep_receiving_publish_audio(self):
        """
        持续接收并处理音频数据的线程函数
        
        功能：
            - 在独立线程中运行
            - 从socket实时接收音频数据
            - 根据VAD(语音活动检测)状态处理音频缓冲
            - 当说话结束时将完整的句子提交保存
        
        VAD状态说明（关键参数vad）：
            - vad=0: 静音，无需处理
            - vad=1: 开始说话，清空缓冲并保存当前音频数据
            - vad=2: 持续说话，继续缓冲音频数据
            - vad=3: 结束说话，将完整句子数据提交队列保存
        
        流程：
            1. 调用audio_provider.read()从socket接收音频
            2. 检查VAD状态
            3. 根据VAD状态更新audio_buffer
            4. 当VAD=3时，将完整的句子音频提交保存队列
            5. 若save_audio为False，仅接收不保存
        
        说明：
            - 此线程在节点初始化时始终启动
            - 通过stop_event.set()可停止此线程
        """
        while not self.stop_event.is_set():
            audio_res = self.audio_provider.read()
            if audio_res is None:
                continue

            audio_data, vad = audio_res
            if vad == 1:
                # 开始说话：清空缓存，保存新数据
                self.get_logger().debug("开始说话，先清空缓存，然后缓存音频数据")
                self.audio_buffer.clear()
                self.audio_buffer.extend(audio_data)
            elif vad == 2:
                # 持续说话：继续缓存
                self.get_logger().debug("持续说话，继续缓存音频数据")
                self.audio_buffer.extend(audio_data)
            elif vad == 3:
                # 结束说话：添加最后数据并提交保存
                self.audio_buffer.extend(audio_data)
                sentence_audio_data = bytes(self.audio_buffer)
                if self.save_audio and self.audio_queue is not None:
                    # 将完整的句子音频提交到保存队列
                    self.audio_queue.put(sentence_audio_data)
                self.get_logger().debug("结束说话")


def main(args=None):
    """
    ROS2节点的主入口函数
    
    功能：
        - 初始化ROS2系统
        - 创建AudioSaver节点
        - 设置信号处理器用于优雅关闭
        - 启动节点循环
        - 处理异常并确保清理资源
    
    使用方法：
        ros2 run sdk_demo audio_saver --ros-args -p save_audio:=true
    
    优雅关闭：
        - 按Ctrl+C可安全停止节点
        - 收到SIGTERM信号时会自动清理
    """
    rclpy.init(args=args)
    audio_saver = SocketAudioSaver()

    stop_called = False

    def stop_handle():
        """
        处理停止信号的内部函数
        
        功能：
            - 防止重复处理
            - 关闭音频保存器
            - 销毁ROS节点
            - 优雅关闭ROS系统
        """
        nonlocal stop_called
        if stop_called:
            return
        stop_called = True

        print("接收到终止信号，准备终止程序...")

        audio_saver.close()
        audio_saver.destroy_node()
        print("节点已销毁，正在关闭 rclpy...")
        if rclpy.ok():
            rclpy.shutdown()

    # 注册SIGTERM信号处理器
    signal.signal(signal.SIGTERM, lambda *args: stop_handle())

    try:
        # 启动节点事件循环
        rclpy.spin(audio_saver)
    except KeyboardInterrupt:
        # 处理Ctrl+C信号
        print("接收到 Ctrl+C，准备退出...")
    finally:
        # 确保清理（即使异常也会执行）
        stop_handle()

if __name__ == '__main__':
    main()

class SocketConnector:
    """
    Socket连接器基类
    
    功能：
        - 建立和管理TCP socket连接
        - 处理连接异常
        - 实现完整的数据接收逻辑
    
    属性：
        client_socket (socket): TCP socket对象
        server_ip_port (tuple): 服务器地址和端口 (ip, port)
        run (bool): 连接状态标志
    
    说明：
        - 创建实例时自动尝试连接
        - 连接失败时run=False，程序继续运行
        - 接收超时时间为3秒
    """
    
    def __init__(self, ip: str, port: int):
        """
        初始化socket连接器并尝试连接到服务器
        
        参数：
            ip (str): 服务器IP地址
            port (int): 服务器端口号
        
        异常处理：
            - ConnectionRefusedError: 连接被拒绝
            - timeout: 连接超时
            - 其他异常: 记录错误但不中断程序
        """
        self.client_socket = socket(AF_INET, SOCK_STREAM)
        self.server_ip_port = (ip, port)
        self.run = False  # 默认未连接

        try:
            # 尝试连接到服务器
            self.client_socket.connect(self.server_ip_port)
            # 设置接收超时为3秒
            self.client_socket.settimeout(3.0)
            self.run = True
            logger.info(f"已成功连接到服务器 {ip}:{port}")
        except ConnectionRefusedError:
            logger.error(f"连接被拒绝: {ip}:{port}")
        except timeout:
            logger.error(f"连接超时: {ip}:{port}")
        except Exception as e:
            logger.error(f"连接服务器失败: {ip}:{port}, 错误: {e}")

    def close(self):
        """
        关闭socket连接
        
        功能：
            - 设置run标志为False
            - 关闭socket
            - 异常时继续执行，不抛出异常
        """
        self.run = False
        try:
            self.client_socket.close()
        except Exception as e:
            logger.warning(f"关闭socket时出错: {e}")
        logger.info("资源已释放")

    def receive_full_data(self, expected_length):
        """
        接收指定长度的完整数据
        
        功能：
            - 循环接收数据直到达到指定长度
            - 处理socket分包情况
            - 检测连接断开
        
        参数：
            expected_length (int): 期望接收的字节数
        
        返回值：
            bytes: 接收到的完整数据
            None: 连接断开或超时
        
        说明：
            - 每次接收最多4096字节
            - 若收到空数据说明连接已断开
            - 超时或错误时返回None
        """
        received_data = bytearray()
        while len(received_data) < expected_length and self.run:
            try:
                # 计算本次应接收的字节数
                recv_size = min(4096, expected_length - len(received_data))
                chunk = self.client_socket.recv(recv_size)
                if not chunk:
                    # 收到空数据表示连接已被关闭
                    logger.info("服务器关闭连接")
                    return None
                received_data.extend(chunk)
            except timeout:
                # 超时返回None
                return None
            except Exception as e:
                logger.error(f"接收错误: {e}")
                return None
        return bytes(received_data)

class SocketAudioProvider(SocketConnector):
    """
    Socket音频提供者
    
    从socket接收音频数据并解析协议，返回音频数据和VAD状态。
    
    协议格式参考：https://aiui-doc.xf-yun.com/project-1/doc-392/
    
    通信协议说明：
        头部(9字节)：
            - Byte 0: 同步头 0xa5
            - Byte 1: 用户ID 0x01
            - Byte 2: 消息类型
            - Byte 3-4: 消息长度(小端序)
            - Byte 5-8: 消息ID(小端序)
        
        消息体(msg_length+1字节)：
            - Byte 0: VAD状态 (0=静音, 1=开始说话, 2=持续说话, 3=结束说话)
            - Byte 1: 声道信息
            - Byte 2-3: 保留
            - Byte 4-7: 帧ID
            - Byte 8-end-1: 音频数据
            - 最后1字节: 校验码
    """
    
    def __init__(self, ip: str, port: int):
        """
        初始化音频提供者
        
        参数：
            ip (str): 音频服务器IP
            port (int): 音频服务器端口
        """
        super().__init__(ip, port)

    def read(self):
        """
        从socket读取音频数据并解析
        
        返回值：
            tuple: (audio_data, vad) - 音频数据和VAD状态
                   audio_data (bytes): 音频PCM数据
                   vad (int): VAD状态
                       0 = 静音
                       1 = 开始说话
                       2 = 持续说话
                       3 = 结束说话
            None: 连接断开、超时或协议错误
        
        流程：
            1. 接收9字节头部
            2. 验证头部同步码和用户ID
            3. 解析头部获取消息长度
            4. 接收消息体
            5. 解析消息体获取VAD、声道和音频数据
            6. 验证数据完整性
        
        说明：
            - 若头部校验失败，记录错误但继续接收
            - 仅返回单声道(channel=0)的音频
            - 自动处理分包和数据校验
        """
        try:
            # 接收9字节的协议头部
            header = self.receive_full_data(9)
            if not header:
                return None

            try:
                # 解析头部：同步头、用户ID、消息类型、消息长度、消息ID
                # '<'表示小端序，'BBBIH'表示(1字节+1字节+1字节+2字节+4字节)
                sync_head, user_id, msg_type, msg_length, msg_id = struct.unpack('<BBBIH', header)
            except struct.error as e:
                logger.error(f"解析头部出错: {e}")
                return None

            # 验证头部校验码
            if sync_head != 0xa5 or user_id != 0x01:
                logger.info(f"头部校验失败: sync={sync_head:02x}, user={user_id:02x}")
                return None

            # 接收消息体（消息长度 + 1字节校验码）
            body = self.receive_full_data(msg_length + 1)
            if not body:
                return None

            try:
                # 解析消息体
                vad = body[0]                                  # VAD状态
                channel = body[1]                              # 声道信息
                frame_id = struct.unpack('<I', body[4:8])[0]  # 帧ID
                audio_data = body[8:-1]                        # 提取音频数据（去掉最后1字节校验码）
            except Exception as e:
                logger.error(f"解析body出错: {e}, body长度={len(body)}")
                return None

            # 仅返回单声道数据
            if channel == 0:
                return audio_data, vad
            return None

        except Exception as e:
            logger.error(f"处理异常: {e}")
            return None
        
