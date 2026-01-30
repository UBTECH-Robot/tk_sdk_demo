#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机状态监控演示脚本

功能说明：
    本脚本用于订阅并实时显示机器人各关节电机的状态信息
    可以监控以下关节的电机状态：腰部(waist)、手臂(arm)、腿部(leg)、头部(head)
    
使用方式：
    1. 订阅腰部电机状态（默认）：
       ros2 run sdk_demo motor_status_demo
    
    2. 订阅手臂电机状态：
       ros2 run sdk_demo motor_status_demo arm
    
    3. 订阅腿部电机状态：
       ros2 run sdk_demo motor_status_demo leg
    
    4. 订阅头部电机状态：
       ros2 run sdk_demo motor_status_demo head
"""

import math
import rclpy
from rclpy.node import Node
import sys
import argparse
import json
import os
import threading
from datetime import datetime
from bodyctrl_msgs.msg import MotorStatusMsg, MotorStatus

# 定义各个关节的电机状态话题名称
# 腰部关节的电机状态话题
topic1 = '/waist/status'
# 手臂关节的电机状态话题
topic2 = '/arm/status'
# 腿部关节的电机状态话题
topic3 = '/leg/status'
# 头部关节的电机状态话题
topic4 = '/head/status'

# 话题映射字典
# 将关节名称映射到对应的话题名称，用于根据命令行参数快速查找话题
topic_map = {"waist": topic1, "arm": topic2, "leg": topic3, "head": topic4}

# 常见错误码字典
# 用于将数字错误码转换为可读的错误说明
ERROR_CODE_MAP = {
    0: "无错误",
    1: "电机过温",        # 电机温度过高，需要冷却
    2: "过流",            # 电流超过安全值
    3: "电压过低",        # 电源电压不足
    4: "MOS过温",         # 电源管理芯片过温
    5: "堵转",            # 电机转子被卡住
    6: "电压过高",        # 电源电压过高
    7: "缺相",            # 电机相位缺失
    8: "编码器错误",      # 位置反馈传感器故障
    33072: "设备掉线",    # CAN/通信掉线
    33073: "关节位置超限"  # 关节运动超出允许范围
}

def radians_to_degrees(radians: float) -> float:
    """
    将弧度转换为角度
    
    参数：
        radians (float): 以弧度表示的角度值

    返回：
        float: 以度表示的角度值
    """
    return radians * (180.0 / math.pi)

class MotorStatusMonitor(Node):
    """
    电机状态监控节点类
    
    该类继承自ROS2的Node基类，用于创建一个订阅电机状态话题的节点
    可以根据命令行参数选择订阅不同关节的电机状态话题
    """
    
    def __init__(self, joint_name='waist', record_mode=False):
        """
        初始化电机状态监控节点
        
        参数：
            joint_name (str): 要监控的关节名称（waist/arm/leg/head）
            record_mode (bool): 是否启用记录模式（-r参数）
        
        功能：
            1. 调用父类构造函数，创建名为'motor_status_monitor'的节点
            2. 根据命令行参数确定要订阅的话题
            3. 创建话题订阅器
            4. 输出提示信息
            5. 如果启用记录模式，初始化数据存储和输入监听
        """
        # 初始化节点，节点名称为'motor_status_monitor'
        super().__init__('motor_status_monitor')
        
        # 记录模式标志和数据存储
        self.record_mode = record_mode
        self.joint_name = joint_name
        self.recorded_data = []  # 存储记录的电机状态数据
        self.latest_msg = None   # 存储最新接收的消息
        self.record_trigger = False  # 记录触发标志
        self.lock = threading.Lock()  # 线程锁，保护共享数据
        
        # 从话题映射字典中获取对应的话题名称，默认为腰部话题
        topic_name = topic_map.get(joint_name, topic1)
        
        # 创建话题订阅器
        # 参数说明：
        #   - MotorStatusMsg: 订阅的消息类型
        #   - topic_name: 订阅的话题名称（根据命令行参数动态确定）
        #   - self.status_callback: 接收到消息时的回调函数
        #   - 10: 消息队列大小，用于缓存消息
        self.subscription = self.create_subscription(
            MotorStatusMsg,
            topic_name,
            self.status_callback,
            10
        )
        
        # 打印日志，提示已开始订阅指定话题
        if self.record_mode:
            self.get_logger().info(f"已开始订阅话题 {topic_name}，记录模式已启用（按 'r' 键记录数据）")
            # 启动键盘输入监听线程
            self.input_thread = threading.Thread(target=self._input_listener, daemon=True)
            self.input_thread.start()
        else:
            self.get_logger().info(f"已开始订阅话题 {topic_name}，监控电机状态信息")

    def _input_listener(self):
        """
        键盘输入监听线程函数
        
        在记录模式下，持续监听用户输入，当用户按下 'r' 键时触发记录
        """
        print("\n提示：按 'r' 键记录当前电机状态，按 Ctrl+C 退出并保存数据\n")
        while True:
            try:
                user_input = input().strip().lower()
                if user_input == 'r':
                    with self.lock:
                        self.record_trigger = True
            except EOFError:
                # 处理输入流结束的情况
                break
            except Exception as e:
                self.get_logger().error(f"输入监听出错: {str(e)}")
    
    def status_callback(self, msg: MotorStatusMsg):
        """
        电机状态消息回调函数
        
        当接收到电机状态话题的消息时，本函数会被自动调用，显示各电机的详细信息
        
        参数：
            msg (MotorStatusMsg): 接收到的电机状态消息对象
                                  包含header（消息头）和status（电机数组）两部分
        
        消息结构说明：
            - header: 消息头，包含时间戳(stamp)和坐标系ID(frame_id)
            - status: MotorStatus 电机状态数组，每个元素包含以下字段：
                * name: 电机名称(ID)，uint16类型
                * pos: 电机位置，float32类型，单位：rad（弧度）
                * speed: 电机速度，float32类型，单位：rad/s（弧度每秒）
                * current: 电机电流，float32类型，单位：A（安培）
                * temperature: 电机温度，float32类型，单位：℃（摄氏度）
                * error: 错误代码，uint32类型
        """
        try:
            # 检查消息对象是否有status属性
            if not hasattr(msg, 'status'):
                print("接收到的消息格式不符合预期，缺少status字段")
                return
            
            motor_array = msg.status
            if not motor_array:
                return
            
            # 记录模式：只保存最新消息，等待用户触发打印和记录
            if self.record_mode:
                with self.lock:
                    self.latest_msg = msg
                    
                    # 检查是否触发记录
                    if self.record_trigger:
                        self.record_trigger = False
                        self._print_and_record_status(motor_array)
            else:
                # 普通模式：持续打印
                self._print_status(motor_array)
                
        except Exception as e:
            # 捕获并打印异常信息
            self.get_logger().error(f"处理电机状态消息时出错: {str(e)}")
    
    def _print_status(self, motor_array):
        """
        打印电机状态信息（普通模式）
        
        参数：
            motor_array: 电机状态数组
        """
        # ========== 打印表格标题 ==========
        print("\n" + "="*100)
        print(f"{'电机ID':<10} {'位置(rad)':<15} {'位置(deg)':<15} {'速度(rad/s)':<15} "
              f"{'电流(A)':<12} {'温度(℃)':<12} {'错误信息':<20}")
        print("-"*100)
        
        # ========== 遍历各电机，以表格行形式打印数据 ==========
        for motor in motor_array:
            degree = radians_to_degrees(motor.pos)
            error_message = ERROR_CODE_MAP.get(motor.error, "未知错误")
            error_display = f"⚠️ {error_message}" if motor.error != 0 else error_message
            
            print(f"{motor.name:<10} {motor.pos:<18.3f} {degree:<18.3f} {motor.speed:<18.3f} "
                  f"{motor.current:<18.2f} {motor.temperature:<18.1f} {error_display:<20}")
        
        print("="*100)
    
    def _print_and_record_status(self, motor_array):
        """
        打印并记录电机状态信息（记录模式）
        
        参数：
            motor_array: 电机状态数组
        """
        # 打印状态
        self._print_status(motor_array)
        
        # 记录数据到列表
        motor_data = {}
        for motor in motor_array:
            motor_data[motor.name] = round(float(radians_to_degrees(motor.pos)), 6)
        
        self.recorded_data.append(motor_data)
        self.get_logger().info(f"已记录第 {len(self.recorded_data)} 条数据")
    
    def save_recorded_data(self):
        """
        保存记录的数据到JSON文件
        
        文件保存路径：saved_data/5_motor_saved_status/
        文件名格式：{joint_name}_{timestamp}.json
        """
        if not self.recorded_data:
            self.get_logger().info("没有记录数据，跳过保存")
            return
        
        # 创建保存目录
        save_dir = "saved_data/5_motor_saved_status"
        os.makedirs(save_dir, exist_ok=True)
        
        # 生成文件名（包含关节名称和时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.joint_name}_{timestamp}.json"
        filepath = os.path.join(save_dir, filename)
        
        # 保存数据
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(self.recorded_data, f, ensure_ascii=False, indent=2)
            self.get_logger().info(f"已保存 {len(self.recorded_data)} 条记录到: {filepath}")
            print(f"\n数据已保存到: {filepath}")
        except Exception as e:
            self.get_logger().error(f"保存数据失败: {str(e)}")

def main(args=None):
    """
    主函数
    
    参数：
        args: 命令行参数，默认为None
        
    功能：
        1. 解析命令行参数
        2. 初始化ROS2客户端库
        3. 创建电机状态监控节点
        4. 启动节点循环，持续接收和处理消息
        5. 处理键盘中断（Ctrl+C）
        6. 保存记录数据（如果启用记录模式）
        7. 清理资源并关闭ROS2
    """
    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description="电机状态监控演示脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例用法：\n"
               "  持续监控腰部电机：ros2 run sdk_demo motor_status_demo\n"
               "  持续监控手臂电机：ros2 run sdk_demo motor_status_demo arm\n"
               "  记录模式监控腿部：ros2 run sdk_demo motor_status_demo leg -r\n"
               "  记录模式监控头部：ros2 run sdk_demo motor_status_demo head -r"
    )
    
    parser.add_argument(
        'joint',
        nargs='?',
        default='waist',
        choices=['waist', 'arm', 'leg', 'head'],
        help="要监控的关节部位（默认：waist）"
    )
    
    parser.add_argument(
        '-r', '--record',
        action='store_true',
        help="启用记录模式：按 'r' 键记录数据，退出时保存为JSON文件"
    )
    
    parsed_args = parser.parse_args(args)
    
    # 初始化ROS2的Python客户端库
    rclpy.init(args=args)
    
    # 创建电机状态监控节点实例
    node = MotorStatusMonitor(
        joint_name=parsed_args.joint,
        record_mode=parsed_args.record
    )
    
    try:
        # 启动节点循环，持续处理回调函数
        # spin()会阻塞程序，直到节点被关闭
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获键盘中断（Ctrl+C），允许用户优雅地退出程序
        print("\n程序被用户中断")
    finally:
        # 如果是记录模式，保存数据
        if node.record_mode:
            node.save_recorded_data()
        
        # 销毁节点，释放相关资源
        node.destroy_node()
        
        # 关闭ROS2客户端库，清理所有资源
        rclpy.shutdown()


# Python脚本入口点
# 该脚本作为独立程序运行时会执行main()函数
if __name__ == '__main__':
    main()
