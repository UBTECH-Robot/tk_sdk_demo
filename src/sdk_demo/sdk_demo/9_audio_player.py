#!/usr/bin/env python3
"""
音频播放示例
====================================

提供了一个音频播放节点和内置的简单音频播放器，支持实时音频流播放和文件播放。
针对文件播放，仅支持播放脚本所在前系统的音频文件，不支持播放其他设备的音频，例如当前脚本在41.2，则要播放的音频文件也必须在41.2上。

主要功能:
    1. 实时音频流播放：接收音频字节流并实时播放
    2. 文件播放：支持 WAV、PCM、MP3 格式的音频文件播放
    3. 播放状态监控：可查询当前是否正在播放

核心类:
    AudioPlayer: 主要的音频播放器类
    AudioPlayerNode: ROS2 节点，内部持有 AudioPlayer 的实例

工作流程:
    1. 音频播放节点 AudioPlayerNode 接收到音频路径消息
    2. 停止当前播放并清空队列
    3. 设置新的音频id（使用时间戳区分）
    4. 立即播放新音频

使用示例:
    # 先启动节点
    ros2 run sdk_demo audio_player
    
    # 使用ros命令发送音频路径（另一个终端），最好是绝对路径，如果是相对路径，则需要是相对于执行 ros2 run sdk_demo audio_player 命令的目录的相对路径
    # 对应的 .wav 和 .pcm 文件，可以先使用 ros2 run sdk_demo audio_saver 录制一些语音数据
    ros2 topic pub /audio_file_path std_msgs/String "data: '/home/nvidia/sdk_demo/saved_data/audio_files/audio_113806860.wav'" --once
    ros2 topic pub /audio_file_path std_msgs/String "data: '/home/nvidia/sdk_demo/saved_data/audio_files/audio_134758915.pcm'" --once

注意:
    - 新消息会立即打断当前播放
    - 需要确保音频文件路径有效且可访问
    - 节点退出时会自动释放音频资源
    - 播放格式: float32, 22050Hz, 单声道
    - 所有输入会自动转换为此格式
    
    
依赖项:
    - pyaudio: 音频播放库
    - numpy: 音频数据处理
    - wave: WAV 文件处理
    - pydub (可选): MP3 格式支持
    - ffmpeg (可选): MP3 解码支持

安装依赖:
    pip install pyaudio numpy
    pip install pydub  # 可选，用于 MP3 支持
    sudo apt install ffmpeg  # 可选，用于 MP3 解码
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time
import traceback
import pyaudio
from queue import Queue, Empty, Full
import wave
import time
import logging
from logging import StreamHandler, Formatter
import numpy as np
from pathlib import Path

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
        logger.Logger: 配置好的logger对象
    
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

def wait_for_audio_ready(max_wait=5):
    """
    等待音频设备就绪
    
    在某些系统中，音频设备可能需要初始化时间。此函数会循环检查
    音频设备是否可用，直到设备就绪或超时。
    
    参数:
        max_wait (int, optional): 最大等待时间（秒），默认为5秒
    
    功能:
        - 每0.5秒检查一次音频设备
        - 检查默认输出设备是否可用
        - 检查设备是否有有效的采样率
        - 超时后仍会继续执行，但记录警告
    
    注意:
        此函数主要用于解决某些系统启动时音频设备未就绪的问题
    """
    for i in range(max_wait * 10):
        try:
            p = pyaudio.PyAudio()
            info = p.get_default_output_device_info()
            if info and info.get('defaultSampleRate'):
                logger.info(f"音频设备已就绪，默认采样率: {info['defaultSampleRate']}")
                p.terminate()
                return
        except Exception as e:
            pass
        time.sleep(0.5)
    logger.info("警告：音频设备可能未准备就绪，继续执行...")

class AudioPlayer:
    """
    多线程音频播放器
    
    此类提供了一个完整的音频播放解决方案，支持实时流播放和文件播放。
    使用独立的后台线程处理音频播放，不会阻塞主线程。
    
    核心特性:
        - 自动音频格式转换（统一为 float32, 22050Hz, 单声道）
        - 多问题场景支持（可按音频id管理不同的音频队列）
        - 线程安全的音频队列管理
        - 实时播放状态监控
        - 支持多种音频格式（WAV、PCM、MP3）
    
    内部架构:
        - playing_thread: 后台播放线程，持续从队列获取音频并播放
        - audioid_audio_map: 按音频id组织的音频队列字典
        - playing_stream: PyAudio 音频输出流
        - 多个锁保证线程安全
    
    音频规格:
        格式: pyaudio.paFloat32 (32位浮点)
        采样率: 22050 Hz
        声道: 1 (单声道)
        缓冲区大小: 1024 帧
    
    注意事项:
        - 必须在使用完毕后调用 close() 释放资源
        - MP3 支持需要额外安装 pydub 和 ffmpeg
        - 所有音频会自动转换为统一格式，可能产生轻微延迟
    """
    
    def __init__(self):
        """
        初始化音频播放器
        
        初始化流程:
            1. 等待音频设备就绪
            2. 创建 PyAudio 实例
            3. 获取默认输出设备信息
            4. 初始化各种锁和事件
            5. 打开音频输出流
            6. 启动后台播放线程
        
        创建的资源:
            - self.audio: PyAudio 实例
            - self.playing_stream: 音频输出流
            - self.playing_thread: 后台播放线程
            - self.audioid_audio_map: 音频队列字典
            - 各种线程同步锁和事件
        
        异常:
            - OSError: 如果音频设备不可用或配置错误
        """
        # 等待音频设备准备就绪
        wait_for_audio_ready()
        
        # 初始化 PyAudio
        self.audio = pyaudio.PyAudio()
        self.device_info = self.audio.get_default_output_device_info()
        
        # 音频id管理
        self.audioid_lock = threading.Lock()
        self.audioid_audio_map_lock = threading.Lock()
        self.audioid = ""  # 当前音频的惟一标识即可，可用时间戳
        self.audioid_audio_map = {}  # 音频id -> 音频队列的映射

        # 音频流管理
        self.stream_lock = threading.Lock()
        self.playing_stream = self.open_stream()
        self.chunk_size = 1024

        # 线程控制
        self.stop_event = threading.Event()  # 停止信号
        self.is_speaking_event = threading.Event()  # 播放状态标志

        # 启动后台播放线程
        self.playing_thread = threading.Thread(target=self.keep_playing_audio, daemon=True)
        self.playing_thread.start()
        logger.info(f"音频播放线程已启用，音频数据将自动按顺序播放")

    def is_speaking(self) -> bool:
        """
        检查当前是否正在播放音频
        
        此方法是线程安全的，可以从任何线程调用。
        
        返回:
            bool: True 表示正在播放，False 表示空闲
        """
        return self.is_speaking_event.is_set()
    
    def set_audioid(self, text: str):
        """
        设置当前音频id并创建对应的音频队列
        
        在多问题场景中，通过音频id区分不同的音频队列。
        相同音频id的音频会按顺序播放，不同问题的音频互不干扰。
        
        参数:
            text (str): 音频id，作为音频队列的标识
        
        功能:
            1. 设置当前音频id
            2. 为该问题创建新的音频队列（如果不存在）
        """
        with self.audioid_lock:
            self.audioid = text
        with self.audioid_audio_map_lock:
            self.audioid_audio_map[text] = Queue()

    def get_audioid(self) -> str:
        """
        获取当前音频id
        
        返回:
            str: 当前设置的音频id
        
        线程安全: 使用锁保护，可从多线程调用
        """
        with self.audioid_lock:
            return self.audioid
        
    def open_stream(self):
        """
        打开音频输出流（带重试机制）
        
        创建一个新的 PyAudio 音频流，配置为标准格式。
        如果遇到采样率错误（-9997），会自动重试3次。
        
        音频配置:
            - 格式: float32 (32位浮点，范围 -1.0 到 1.0)
            - 声道: 单声道
            - 采样率: 22050 Hz
            - 缓冲区: 1024 帧
            - 输出设备: 系统默认输出设备
        
        重试机制:
            - 最多重试3次
            - 每次重试间隔3秒
            - 只针对采样率错误（errno -9997）重试
            - 其他错误直接抛出
        
        返回:
            pyaudio.Stream: 配置好的音频输出流
        
        异常:
            - OSError: 如果无法打开音频设备或设备配置错误
        
        注意:
            - 此方法通常由构造函数调用，不需要手动调用
            - 如果流被关闭，需要重新调用此方法
            - 使用锁保护，确保线程安全
        """
        with self.stream_lock:
            last_exc = None
            for _ in range(3):
                try:
                    device_index = self.device_info['index']
                    logger.info(f'使用的音频输出设备索引: {device_index}, 设备名称: {self.device_info["name"]}')
                    stream = self.audio.open(
                        format=pyaudio.paFloat32,
                        channels=1,
                        rate=22050,
                        output=True,
                        output_device_index=device_index,
                        frames_per_buffer=1024
                    )
                    return stream
                except OSError as e:
                    logger.info(f'PyAudio open_stream报错了：{e}')
                    traceback.print_exc()
                    if e.errno == -9997:  # Invalid sample rate
                        last_exc = e
                        time.sleep(3)  # 等待一会再试
                    else:
                        raise  # 不是采样率的问题，直接抛出
            # 三次都失败，抛出最后一次的异常
        raise last_exc
    
    def stop_other_audio_and_clear_queue(self):
        """
        停止其他问题的音频队列，保留当前问题的队列
        
        此方法会：
            1. 获取当前音频id
            2. 清空除当前问题外的所有音频队列
            3. 清除播放状态标志
        
        使用场景:
            - 切换到新问题时，清理旧问题的音频
            - 保持当前问题的播放队列不受影响
        
        线程安全: 使用锁保护队列操作
        
        注意:
            - 只删除其他问题的队列，不影响当前问题
            - 会清除 is_speaking 标志
        """
        audioid = self.get_audioid()
        with self.audioid_audio_map_lock:
            for q_text in list(self.audioid_audio_map.keys()):
                if q_text == audioid:
                    continue
                try:
                    del self.audioid_audio_map[q_text]
                except KeyError:
                    pass

        self.is_speaking_event.clear()

    def close(self):
        """
        关闭音频播放器并释放所有资源
        
        清理流程:
            1. 设置停止事件，通知后台线程退出
            2. 等待播放线程结束（最多2秒）
            3. 停止并关闭音频流
            4. 终止 PyAudio 实例
        
        资源释放:
            - 后台播放线程
            - PyAudio 音频流
            - PyAudio 实例
        
        注意:
            - 必须在程序结束前调用，否则可能导致资源泄漏
            - 关闭后的播放器实例不可再使用
            - 建议使用 with 语句或 try-finally 确保调用
        """
        self.stop_event.set()
        self.playing_thread.join(timeout=2)

        with self.stream_lock:
            self.playing_stream.stop_stream()
            self.playing_stream.close()

        self.audio.terminate()

    def try_put(self, audioid: str, audio_data: bytes):
        """
        尝试将音频数据放入指定问题的队列
        
        功能:
            1. 如果队列不存在，自动创建（线程安全的双重检查）
            2. 尝试在1秒内将音频数据放入队列
            3. 如果队列满了，删除最旧的音频，再放入新音频
        
        参数:
            audioid (str): 音频id，作为队列标识
            audio_data (bytes): 音频数据（float32 格式）
        
        队列管理:
            - 使用双重检查锁（Double-Checked Locking）模式
            - 队列满时采用 FIFO 策略（先进先出）
            - 超时时间: 1秒
        
        线程安全: 使用锁保护队列创建过程
        
        注意:
            - 此方法为内部方法，通常不直接调用
            - 请使用 play() 方法进行播放
        """
        if audioid not in self.audioid_audio_map:
            with self.audioid_audio_map_lock:
                if audioid not in self.audioid_audio_map:
                    self.audioid_audio_map[audioid] = Queue()
        queue = self.audioid_audio_map[audioid]
        try:
            queue.put(audio_data, timeout=1)
        except Full:
            queue.get_nowait()  # 弹出最旧的一条
            queue.put(audio_data)

    def play(self, audio_data: bytes):
        """
        播放音频数据（流式播放）
        
        将音频数据添加到当前问题的播放队列中，后台线程会自动按顺序播放。
        
        参数:
            audio_data (bytes): 音频数据，应为 float32 格式
                               (22050Hz 采样率，单声道)
        
        工作流程:
            1. 获取当前音频id
            2. 将音频数据放入该问题的队列
            3. 后台播放线程自动从队列取出并播放
        
        注意:
            - 音频会按添加顺序播放（FIFO）
            - 不会阻塞调用线程
            - 音频格式必须正确，否则播放会失败
        """
        self.try_put(self.get_audioid(), audio_data)

    def play_by_path(self, file_path: str):
        """
        通过文件路径播放音频
        
        功能：
            - 支持绝对路径和相对路径
            - 支持 .wav、.pcm、.mp3 格式的音频文件
            - 自动转换为播放器所需的格式（float32, 22050Hz, 单声道）
            - 内部调用 play 方法进行实际播放
        
        参数：
            file_path (str): 音频文件路径，支持绝对路径和相对路径
        
        支持的格式：
            - .wav: 标准WAV格式，自动重采样到22050Hz
            - .pcm: 原始PCM格式（假设为16位、16000Hz采样率）
            - .mp3: MP3格式（需要安装pydub和ffmpeg）
        
        异常：
            - FileNotFoundError: 文件不存在
            - ValueError: 不支持的文件格式
            - Exception: 音频处理或播放错误
        """
        # 转换为Path对象，支持相对路径和绝对路径
        audio_path = Path(file_path)
        
        # 如果是相对路径，转换为绝对路径
        if not audio_path.is_absolute():
            audio_path = audio_path.resolve()
        
        # 检查文件是否存在
        if not audio_path.exists():
            raise FileNotFoundError(f"音频文件不存在: {audio_path}")
        
        # 获取文件扩展名
        file_ext = audio_path.suffix.lower()
        
        logger.info(f"正在加载音频文件: {audio_path}")
        
        try:
            if file_ext == '.wav':
                audio_data = self._load_wav(audio_path)
            elif file_ext == '.pcm':
                audio_data = self._load_pcm(audio_path)
            elif file_ext == '.mp3':
                audio_data = self._load_mp3(audio_path)
            else:
                raise ValueError(f"不支持的音频格式: {file_ext}，支持的格式: .wav, .pcm, .mp3")
            
            # 调用 play 方法播放音频数据
            self.play(audio_data)
            logger.info(f"音频文件已加入播放队列: {audio_path.name}")
            
        except Exception as e:
            logger.error(f"播放音频文件失败 {audio_path}: {e}")
            raise
    
    def _load_wav(self, file_path: Path) -> bytes:
        """
        加载 WAV 格式音频文件
        
        功能:
            1. 读取 WAV 文件的所有参数（声道、采样率、位深等）
            2. 自动处理多声道（转换为单声道）
            3. 支持 8位、16位、32位采样
            4. 归一化到 [-1.0, 1.0] 范围
            5. 自动重采样到 22050Hz
        
        参数:
            file_path (Path): WAV 文件路径
        
        返回:
            bytes: float32 格式的音频数据，22050Hz 采样率，单声道
        
        支持的格式:
            - 位深: 8位 (uint8), 16位 (int16), 32位 (int32)
            - 声道: 单声道、立体声、多声道
            - 采样率: 任意（自动重采样到 22050Hz）
        
        处理流程:
            1. 打开 WAV 文件并读取参数
            2. 读取所有音频帧
            3. 根据位深转换为 numpy 数组
            4. 多声道转单声道（取平均）
            5. 归一化到 [-1.0, 1.0]
            6. 重采样到 22050Hz
            7. 转换为 float32 bytes
        
        异常:
            ValueError: 不支持的采样宽度
        """
        with wave.open(str(file_path), 'rb') as wf:
            # 获取音频参数
            channels = wf.getnchannels()
            sample_width = wf.getsampwidth()
            framerate = wf.getframerate()
            n_frames = wf.getnframes()
            
            logger.debug(f"WAV参数: {channels}声道, {sample_width*8}位, {framerate}Hz, {n_frames}帧")
            
            # 读取所有音频数据
            audio_data = wf.readframes(n_frames)
            
            # 转换为numpy数组
            if sample_width == 2:  # 16位
                audio_array = np.frombuffer(audio_data, dtype=np.int16)
            elif sample_width == 1:  # 8位
                audio_array = np.frombuffer(audio_data, dtype=np.uint8)
                audio_array = (audio_array.astype(np.float32) - 128) / 128.0
            elif sample_width == 4:  # 32位
                audio_array = np.frombuffer(audio_data, dtype=np.int32)
            else:
                raise ValueError(f"不支持的采样宽度: {sample_width}字节")
            
            # 如果是立体声，转换为单声道（取平均）
            if channels == 2:
                audio_array = audio_array.reshape(-1, 2).mean(axis=1)
            elif channels > 2:
                audio_array = audio_array.reshape(-1, channels).mean(axis=1)
            
            # 归一化到 [-1.0, 1.0]
            if sample_width == 2:
                audio_array = audio_array.astype(np.float32) / 32768.0
            elif sample_width == 4:
                audio_array = audio_array.astype(np.float32) / 2147483648.0
            
            # 重采样到 22050Hz（如果需要）
            if framerate != 22050:
                audio_array = self._resample(audio_array, framerate, 22050)
            
            # 转换为bytes
            return audio_array.astype(np.float32).tobytes()
    
    def _load_pcm(self, file_path: Path) -> bytes:
        """
        加载 PCM 格式音频文件
        
        默认假设:
            - 格式: 16位有符号整数 (int16)
            - 采样率: 16000 Hz
            - 声道: 单声道
        
        如果您的 PCM 文件格式不同，请修改此函数中的参数。
        
        参数:
            file_path (Path): PCM 文件路径
        
        返回:
            bytes: float32 格式的音频数据，22050Hz 采样率，单声道
        
        处理流程:
            1. 读取原始 PCM 字节数据
            2. 转换为 int16 numpy 数组
            3. 归一化到 [-1.0, 1.0] 范围
            4. 重采样从 16000Hz 到 22050Hz
            5. 转换为 float32 bytes
        
        注意:
            - PCM 文件没有头信息，需要预先知道格式
            - 如果格式不同，可能需要修改代码
        """
        # 读取原始PCM数据
        with open(file_path, 'rb') as f:
            pcm_data = f.read()
        
        # 转换为16位整数数组
        audio_array = np.frombuffer(pcm_data, dtype=np.int16)
        
        # 归一化到 [-1.0, 1.0]
        audio_array = audio_array.astype(np.float32) / 32768.0
        
        # 重采样从 16000Hz 到 22050Hz
        audio_array = self._resample(audio_array, 16000, 22050)
        
        # 转换为bytes
        return audio_array.astype(np.float32).tobytes()
    
    def _load_mp3(self, file_path: Path) -> bytes:
        """
        加载 MP3 格式音频文件
        
        依赖:
            - pydub (>=0.25.1): Python 音频处理库
            - ffmpeg (>=4.0): 音频解码器
        
        安装:
            pip install pydub
            sudo apt install ffmpeg  # Ubuntu/Debian
            brew install ffmpeg      # macOS
        
        参数:
            file_path (Path): MP3 文件路径
        
        返回:
            bytes: float32 格式的音频数据，22050Hz 采样率，单声道
        
        处理流程:
            1. 使用 pydub 加载 MP3 文件
            2. 转换为单声道（如果是多声道）
            3. 重采样到 22050Hz
            4. 转换为 numpy 数组
            5. 根据位深归一化到 [-1.0, 1.0]
            6. 转换为 float32 bytes
        
        异常:
            ImportError: pydub 未安装
            ValueError: 不支持的采样宽度
        
        注意:
            - MP3 解码需要 ffmpeg 支持
            - 确保系统中 ffmpeg 可用
        """
        try:
            from pydub import AudioSegment
        except ImportError:
            raise ImportError("播放MP3需要安装pydub库: pip install pydub")
        
        # 加载MP3文件
        audio = AudioSegment.from_mp3(str(file_path))
        
        # 转换为单声道
        if audio.channels > 1:
            audio = audio.set_channels(1)
        
        # 转换采样率到 22050Hz
        if audio.frame_rate != 22050:
            audio = audio.set_frame_rate(22050)
        
        # 获取原始音频数据
        audio_data = audio.raw_data
        
        # 转换为numpy数组
        if audio.sample_width == 2:  # 16位
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            audio_array = audio_array.astype(np.float32) / 32768.0
        elif audio.sample_width == 4:  # 32位
            audio_array = np.frombuffer(audio_data, dtype=np.int32)
            audio_array = audio_array.astype(np.float32) / 2147483648.0
        else:
            raise ValueError(f"不支持的采样宽度: {audio.sample_width}字节")
        
        # 转换为bytes
        return audio_array.astype(np.float32).tobytes()
    
    def _resample(self, audio_array: np.ndarray, orig_sr: int, target_sr: int) -> np.ndarray:
        """
        音频重采样（线性插值）
        
        使用线性插值法将音频从一个采样率转换到另一个采样率。
        这是一个简单的重采样实现，适合大多数场景。
        
        参数:
            audio_array (np.ndarray): 原始音频数据（1维数组）
            orig_sr (int): 原始采样率（Hz）
            target_sr (int): 目标采样率（Hz）
        
        返回:
            np.ndarray: 重采样后的音频数据（float32）
        
        算法:
            1. 计算音频时长 = 样本数 / 采样率
            2. 计算目标样本数 = 时长 * 目标采样率
            3. 生成均匀分布的插值索引
            4. 使用线性插值计算新样本值
        
        性能:
            - 时间复杂度: O(n)，n 为目标样本数
            - 空间复杂度: O(n)
        
        注意:
            - 如果采样率相同，直接返回原数组
            - 线性插值可能不是最优质量，如需更高质量可使用 librosa
            - 适合语音类音频，对音乐质量要求高的场景建议使用专业库
        """
        if orig_sr == target_sr:
            return audio_array
        
        # 计算重采样比率
        duration = len(audio_array) / orig_sr
        target_length = int(duration * target_sr)
        
        # 使用线性插值
        indices = np.linspace(0, len(audio_array) - 1, target_length)
        resampled = np.interp(indices, np.arange(len(audio_array)), audio_array)
        
        return resampled.astype(np.float32)

    def keep_playing_audio(self):
        """
        后台播放线程主循环
        
        此方法在独立线程中运行，持续从音频队列中取出数据并播放。
        是整个音频播放系统的核心循环。
        
        工作流程:
            1. 检查停止事件，如果设置则退出循环
            2. 确保音频流处于活动状态
            3. 获取当前问题的音频队列
            4. 从队列中取出一个音频数据块
            5. 通过 PyAudio 流播放音频
            6. 如果队列为空，清除播放状态标志
        
        状态管理:
            - is_speaking_event: 播放时设置，队列空时清除
            - stop_event: 用于通知线程退出
        
        线程同步:
            - 使用 stream_lock 保护流操作
            - 使用 audioid_audio_map_lock 访问队列字典
        
        错误处理:
            - 捕获所有播放异常，记录日志但不中断循环
            - 队列为空时短暂休眠，避免空转
        
        性能优化:
            - 队列空时休眠 0.01 秒，减少 CPU 占用
            - 使用 get_nowait() 非阻塞获取，提高响应速度
        
        注意:
            - 此方法作为守护线程运行
            - 不应直接调用，由构造函数自动启动
            - 通过 stop_event 优雅退出
        """
        while not self.stop_event.is_set():            
            # 确保音频流处于活动状态
            if not self.playing_stream.is_active():
                self.playing_stream.start_stream()
                continue
            
            queue = None
            try:
                # 获取当前音频id
                q_text = self.get_audioid()
                if q_text not in self.audioid_audio_map:
                    time.sleep(0.01)  # 队列不存在，短暂休眠
                    continue
                
                # 获取音频队列
                queue = self.audioid_audio_map.get(q_text)
                if queue is None:
                    time.sleep(0.01)
                    continue
                
                # 非阻塞获取音频数据
                audio_data = queue.get_nowait()
            except Empty:
                # 队列为空，短暂休眠后继续
                time.sleep(0.01)
                continue

            try:
                if audio_data is None:
                    time.sleep(0.01)
                    break
                
                # 播放音频数据
                with self.stream_lock:
                    self.is_speaking_event.set()  # 设置播放状态
                    self.playing_stream.write(audio_data)  # 写入音频流
                
                # 如果队列已空，清除播放状态
                if queue and queue.empty():
                    self.is_speaking_event.clear()

            except Exception as e:
                logger.info(f"播放音频时发生错误: {e}")

class AudioPlayerNode(Node):
    """
    ROS2 音频播放节点
    
    订阅音频文件路径话题，接收到消息后立即播放对应的音频文件。
    新消息会打断当前播放，立即播放新的音频。
    
    话题:
        订阅: /audio_file_path (std_msgs/String)
            - 消息内容: 音频文件的绝对路径或相对路径
            - 支持格式: .wav, .pcm, .mp3
    
    """
    
    def __init__(self):
        """
        初始化音频播放节点
        
        创建资源:
            - AudioPlayer 实例
            - /audio_file_path 话题订阅器
        
        配置:
            - 队列大小: 10（保留最近10条消息）
        """
        super().__init__('audio_player_node')
        
        # 创建音频播放器实例
        self.audio_player = AudioPlayer()
        self.get_logger().info('音频播放器已初始化')
        
        # 创建订阅器，订阅音频文件路径话题
        self.subscription = self.create_subscription(
            String,
            '/audio_file_path',
            self.audio_path_callback,
            10  # 队列大小
        )
        self.get_logger().info('已订阅话题: /audio_file_path')
        self.get_logger().info('等待音频文件路径消息...')
    
    def audio_path_callback(self, msg: String):
        """
        音频路径消息回调函数
        
        当接收到新的音频文件路径时：
            1. 打断当前播放并清空队列
            2. 使用时间戳创建新的问题标识
            3. 播放新的音频文件
        
        参数:
            msg (String): ROS2 字符串消息，包含音频文件路径
        
        异常处理:
            - 捕获所有播放异常，记录错误日志但不中断节点运行
        """
        audio_path = msg.data.strip()
        
        if not audio_path:
            self.get_logger().warning('接收到空的音频路径')
            return
        
        self.get_logger().info(f'接收到音频路径: {audio_path}')
        
        try:
            # 停止当前播放并清空所有队列（实现打断功能）
            self.audio_player.stop_other_audio_and_clear_queue()
            
            # 使用时间戳作为音频id，确保每次播放使用新队列
            timestamp = time.time()
            audioid = f"audio_play_{timestamp}"
            self.audio_player.set_audioid(audioid)
            
            # 播放新的音频文件
            self.audio_player.play_by_path(audio_path)
            
            self.get_logger().info(f'开始播放音频: {Path(audio_path).name}')
            
        except FileNotFoundError as e:
            self.get_logger().error(f'音频文件不存在: {audio_path}')
        except ValueError as e:
            self.get_logger().error(f'不支持的音频格式: {e}')
        except Exception as e:
            self.get_logger().error(f'播放音频失败: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def destroy_node(self):
        """
        销毁节点并清理资源
        
        确保音频播放器正确关闭，释放所有资源。
        """
        self.get_logger().info('正在关闭音频播放器...')
        self.audio_player.close()
        super().destroy_node()
        self.get_logger().info('音频播放节点已关闭')


def main(args=None):
    """
    ROS2 节点主函数
    
    初始化 ROS2 上下文，创建并运行音频播放节点。
    """
    rclpy.init(args=args)
    
    node = AudioPlayerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('接收到键盘中断信号')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()