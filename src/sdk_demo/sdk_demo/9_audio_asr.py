#!/usr/bin/env python3
"""
ASR 语音识别节点 (Automatic Speech Recognition Node)
====================================================

此模块提供 ASR 语音识别功能，订阅 lyre 服务发布的语音识别相关话题。

主要功能:
    1. 订阅唤醒关键词话题 (/audio_asr/keyword)
    2. 订阅语音识别结果话题 (/audio_asr/iat)
    3. 订阅 ASR 事件话题 (/audio_asr/event)
    4. 实时输出识别结果和事件日志

话题依赖:
    - /audio_asr/keyword (lyre_msgs/msg/AsrKeyword) - 唤醒关键词
    - /audio_asr/iat (lyre_msgs/msg/AsrIat) - 识别文本
    - /audio_asr/event (lyre_msgs/msg/AsrEvent) - 系统事件
    - 发布者: lyre (运行在 192.168.41.2)

lyre 服务启动:
    如果 lyre 服务未启动，可在 192.168.41.2 上执行:
    cd /home/nvidia/lyre_ros2 && . install/setup.bash && ros2 launch lyre audio.launch.py

使用示例:
    # 配置环境变量（根据运行平台选择）
    # Orin 板
    source ~/lyre_ros2/install/setup.bash
    # 或 x86 板
    source ~/ros2ws/install/setup.bash
    
    # 启动节点
    ros2 run sdk_demo audio_asr
    
    # 节点会持续监听并输出识别结果

注意事项:
    - 需要先确保 lyre 服务已启动
    - 需要配置正确的环境变量
    - 确保网络连接正常（192.168.41.2）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import time
from collections import defaultdict

try:
    from lyre_msgs.msg import AsrKeyword, AsrIat, AsrEvent
except ImportError:
    print("=" * 70)
    print("错误: 无法导入 lyre_msgs")
    print("=" * 70)
    print("请先配置 lyre_ros2 环境变量:")
    print("")
    print("  如果在天工 Orin 板上运行:")
    print("    source ~/lyre_ros2/install/setup.bash")
    print("")
    print("  如果在天工 x86 板上运行:")
    print("    source ~/ros2ws/install/setup.bash")
    print("")
    print("然后重新运行此节点:")
    print("  ros2 run sdk_demo audio_asr")
    print("=" * 70)
    sys.exit(1)


class ASRNode(Node):
    """
    ASR 语音识别节点
    
    订阅 lyre 服务发布的语音识别相关话题，实时输出识别结果。
    
    订阅话题:
        - /audio_asr/keyword: 唤醒关键词检测
        - /audio_asr/iat: 语音识别结果
        - /audio_asr/event: ASR 系统事件
    
    属性:
        keyword_sub: 关键词订阅器
        iat_sub: 识别结果订阅器
        event_sub: 事件订阅器
        topic_check_timer: 话题检查定时器
        last_keyword_time: 最后一次关键词时间
        last_iat_time: 最后一次识别结果时间
        last_event_time: 最后一次事件时间
    """
    
    # ASR 事件类型映射
    EVENT_NAMES = {
        2: "ERROR (出错)",
        3: "STATE (服务状态)",
        4: "WAKEUP (唤醒)",
        5: "SLEEP (休眠)",
        6: "VAD (语音活动检测)",
        10: "PRE_SLEEP (准备休眠)",
        13: "CONNECTED_TO_SERVER (已连接服务器)",
        14: "SERVER_DISCONNECTED (服务器断开)"
    }
    
    def __init__(self):
        """
        初始化 ASR 节点
        
        初始化流程:
            1. 创建节点
            2. 配置 QoS
            3. 创建三个话题订阅器
            4. 启动话题检查定时器
            5. 输出初始化信息
        """
        super().__init__('asr_node')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('正在初始化 ASR 语音识别节点...')
        self.get_logger().info('=' * 60)
        
        # 配置 QoS（使用 BEST_EFFORT 以兼容 lyre 服务）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 话题接收统计
        self.last_keyword_time = None
        self.last_iat_time = None
        self.last_event_time = None
        self.keyword_count = 0
        self.iat_count = 0
        self.event_count = 0
        
        # 创建订阅器
        self.get_logger().info('创建话题订阅器...')
        
        # 1. 唤醒关键词订阅
        self.keyword_sub = self.create_subscription(
            AsrKeyword,
            '/audio_asr/keyword',
            self.keyword_callback,
            qos_profile
        )
        self.get_logger().info('✓ 已订阅: /audio_asr/keyword (唤醒关键词)')
        
        # 2. 识别结果订阅
        self.iat_sub = self.create_subscription(
            AsrIat,
            '/audio_asr/iat',
            self.iat_callback,
            qos_profile
        )
        self.get_logger().info('✓ 已订阅: /audio_asr/iat (识别结果)')
        
        # 3. 事件订阅
        self.event_sub = self.create_subscription(
            AsrEvent,
            '/audio_asr/event',
            self.event_callback,
            qos_profile
        )
        self.get_logger().info('✓ 已订阅: /audio_asr/event (系统事件)')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ASR 节点初始化完成')
        self.get_logger().info('=' * 60)
        self.get_logger().info('等待 ASR 数据...')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
    
    def keyword_callback(self, msg: AsrKeyword):
        """
        唤醒关键词回调函数
        
        当检测到唤醒关键词时触发。
        
        参数:
            msg (AsrKeyword): 关键词消息
                - keyword: 检测到的关键词
                - angle: 声源方向角度
        """
        self.keyword_count += 1
        self.last_keyword_time = time.time()
        
        self.get_logger().info('─' * 60)
        self.get_logger().info('🎤 [唤醒关键词]')
        self.get_logger().info(f'   关键词: "{msg.keyword}"')
        self.get_logger().info(f'   声源角度: {msg.angle}°')
        self.get_logger().info(f'   统计: 第 {self.keyword_count} 次唤醒')
        self.get_logger().info('─' * 60)
    
    def iat_callback(self, msg: AsrIat):
        """
        语音识别结果回调函数
        
        当接收到语音识别结果时触发。
        
        参数:
            msg (AsrIat): 识别结果消息
                - id: 识别会话 ID
                - text: 识别出的文本
        """
        self.iat_count += 1
        self.last_iat_time = time.time()
        
        # 判断文本长度，采用不同的显示方式
        if len(msg.text) > 100:
            text_display = msg.text[:100] + "..."
        else:
            text_display = msg.text
        
        self.get_logger().info('─' * 60)
        self.get_logger().info(f'💬[识别结果][{text_display}]')
        self.get_logger().info('─' * 60)
    
    def event_callback(self, msg: AsrEvent):
        """
        ASR 事件回调函数
        
        当 ASR 系统产生事件时触发。
        
        参数:
            msg (AsrEvent): 事件消息
                - event: 事件类型代码
                - arg1: 事件参数1（错误码等）
                - arg2: 事件参数2
        
        事件类型:
            2: ERROR - 出错事件，arg1 是错误码
            3: STATE - 服务状态事件
            4: WAKEUP - 唤醒事件
            5: SLEEP - 休眠事件
            6: VAD - 语音活动检测事件
            10: PRE_SLEEP - 准备休眠事件
            13: CONNECTED_TO_SERVER - 与服务端建立连接
            14: SERVER_DISCONNECTED - 与服务端断开连接
        """
        self.event_count += 1
        self.last_event_time = time.time()
        
        # 获取事件名称
        event_name = self.EVENT_NAMES.get(msg.event, f"UNKNOWN ({msg.event})")
        
        # 根据事件类型选择图标
        if msg.event == 2:  # ERROR
            icon = '❌'
        elif msg.event in [4, 13]:  # WAKEUP, CONNECTED
            icon = '✅'
        elif msg.event in [5, 14]:  # SLEEP, DISCONNECTED
            icon = '⚠️'
        else:
            icon = 'ℹ️'
        
        # 统一使用 info 级别输出，避免 Logger severity 错误
        self.get_logger().info('─' * 60)
        self.get_logger().info(f'{icon} [ASR 事件] [{event_name}][arg1={msg.arg1}, arg2={msg.arg2}]')
        self.get_logger().info('─' * 60)
    
    def get_statistics(self):
        """
        获取接收统计信息
        
        返回:
            dict: 统计信息字典
                - keyword_count: 关键词接收次数
                - iat_count: 识别结果接收次数
                - event_count: 事件接收次数
                - last_keyword_time: 最后关键词时间
                - last_iat_time: 最后识别时间
                - last_event_time: 最后事件时间
        """
        return {
            'keyword_count': self.keyword_count,
            'iat_count': self.iat_count,
            'event_count': self.event_count,
            'last_keyword_time': self.last_keyword_time,
            'last_iat_time': self.last_iat_time,
            'last_event_time': self.last_event_time
        }


def main(args=None):
    """
    ASR 节点主函数
    
    初始化 ROS2 上下文，创建 ASR 节点并持续运行。
    """
    rclpy.init(args=args)
    
    try:
        # 创建节点
        node = ASRNode()
        
        # 持续运行
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n接收到键盘中断信号")
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            # 输出最终统计
            stats = node.get_statistics()
            print("\n" + "=" * 60)
            print("节点退出统计")
            print("=" * 60)
            print(f"唤醒关键词: {stats['keyword_count']} 次")
            print(f"识别结果: {stats['iat_count']} 次")
            print(f"系统事件: {stats['event_count']} 次")
            print("=" * 60)
            
            node.destroy_node()
        
        rclpy.shutdown()


if __name__ == '__main__':
    main()
