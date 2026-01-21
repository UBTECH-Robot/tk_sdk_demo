#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机温度监控演示脚本

功能说明：
    本脚本用于订阅并实时显示机器人各关节电机的温度信息
    可以监控以下关节的电机温度：腰部(waist)、手臂(arm)、腿部(leg)、头部(head)
    
监控的电机数据包括：
    - 电机名称(name): uint16类型，用于标识电机的ID
    - 电机温度(motortemperature): float32类型，单位°C
    - MOS芯片温度(mostemperature): float32类型，单位°C（MOS是电源管理芯片）
    
使用方式：
    1. 订阅腰部电机状态（默认）：
       ros2 run sdk_demo motor_temper_demo
    
    2. 订阅手臂电机状态：
       ros2 run sdk_demo motor_temper_demo arm
    
    3. 订阅腿部电机状态：
       ros2 run sdk_demo motor_temper_demo leg
    
    4. 订阅头部电机状态：
       ros2 run sdk_demo motor_temper_demo head
"""

import rclpy
from rclpy.node import Node
import sys
from bodyctrl_msgs.msg import MotorStatusMsg1

# 定义各个关节的电机温度信息话题名称
# 腰部关节的电机温度信息话题
topic1 = '/waist/motor_status'
# 手臂关节的电机温度信息话题
topic2 = '/arm/motor_status'
# 腿部关节的电机温度信息话题
topic3 = '/leg/motor_status'
# 头部关节的电机温度信息话题
topic4 = '/head/motor_status'

# 话题映射字典
# 将关节名称映射到对应的话题名称，用于根据命令行参数快速查找话题
topic_map = {"waist": topic1, "arm": topic2, "leg": topic3, "head": topic4}

class MotorTempMonitor(Node):
    """
    电机温度监控节点类
    
    该类继承自ROS2的Node基类，用于创建一个订阅电机状态话题的节点
    可以根据命令行参数选择订阅不同关节的电机状态话题
    """
    
    def __init__(self):
        """
        初始化电机温度监控节点
        
        功能：
            1. 调用父类构造函数，创建名为'motor_temp_monitor'的节点
            2. 根据命令行参数确定要订阅的话题
            3. 创建话题订阅器
            4. 输出提示信息
        """
        # 初始化节点，节点名称为'motor_temp_monitor'
        super().__init__('motor_temp_monitor')
        
        # 默认话题为腰部电机状态话题
        self.url = topic1
        
        # 从命令行参数中获取关节名称
        # 如果提供了命令行参数，使用该参数对应的话题；否则使用默认的腰部话题
        # sys.argv[1] 是第一个命令行参数
        # 例如：脚本执行命令为 'python script.py arm' 时，sys.argv[1] = 'arm'
        joint_name = sys.argv[1] if len(sys.argv) > 1 else 'waist'
        topic_name = topic_map.get(joint_name, topic1)
        
        # 创建话题订阅器
        # 参数说明：
        #   - MotorStatusMsg1: 订阅的消息类型
        #   - topic_name: 订阅的话题名称（根据命令行参数动态确定）
        #   - self.motor_status_callback: 接收到消息时的回调函数
        #   - 10: 消息队列大小，用于缓存消息
        self.subscription = self.create_subscription(
            MotorStatusMsg1,
            topic_name,
            self.motor_status_callback,
            10
        )
        
        # 打印日志，提示已开始订阅指定话题
        self.get_logger().info(f"已开始订阅话题 {topic_name}，监控电机温度信息")

    def motor_status_callback(self, msg):
        """
        电机状态消息回调函数
        
        当接收到电机状态话题的消息时，该函数会被自动调用
        解析消息中的电机温度数据，并打印各个电机的温度信息
        
        参数：
            msg (MotorStatusMsg1): 接收到的电机状态消息对象
                                   包含header（消息头）和status（电机数组）两部分
        
        消息结构说明：
            - header: 消息头，包含时间戳(stamp)和坐标系ID(frame_id)
            - status: MotorStatus1 电机状态数组，每个元素包含以下字段：
                * name: 电机名称(ID)，uint16类型
                * motortemperature: 电机温度，float32类型，单位°C
                * mostemperature: MOS芯片温度，float32类型，单位°C
        """
        try:
            # 检查消息对象是否有status属性
            # 正常情况下，MotorStatus1消息应该包含status数组
            if hasattr(msg, 'status'):
                # 获取电机状态数组
                motor_array = msg.status
                
                # 遍历数组中的每个电机，并打印其温度信息
                for i, motor in enumerate(motor_array):
                    # 打印电机索引、名称、电机温度和MOS温度
                    # 温度值使用.1f格式，保留一位小数
                    self.get_logger().info(
                        f"电机ID={motor.name}, "
                        f"电机温度={motor.motortemperature:.1f}°C, "
                        f"MOS温度={motor.mostemperature:.1f}°C"
                    )
            else:
                # 备用处理方式：如果话题直接是一个数组（少见情况）
                self.get_logger().info("接收到电机状态数组：")
                for i, motor in enumerate(msg):
                    self.get_logger().info(
                        f"【电机{i}】 "
                        f"电机ID={motor.name}, "
                        f"电机温度={motor.motortemperature:.1f}°C, "
                        f"MOS温度={motor.mostemperature:.1f}°C"
                    )
        except Exception as e:
            # 捕获并打印异常信息
            self.get_logger().error(f"处理电机温度消息时出错: {str(e)}")


def main(args=None):
    """
    主函数
    
    参数：
        args: 命令行参数，默认为None
        
    功能：
        1. 初始化ROS2客户端库
        2. 创建电机温度监控节点
        3. 启动节点循环，持续接收和处理消息
        4. 处理键盘中断（Ctrl+C）
        5. 清理资源并关闭ROS2
    """
    # 初始化ROS2的Python客户端库
    rclpy.init(args=args)
    
    # 创建电机温度监控节点实例
    node = MotorTempMonitor()
    
    try:
        # 启动节点循环，持续处理回调函数
        # spin()会阻塞程序，直到节点被关闭
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获键盘中断（Ctrl+C），允许用户优雅地退出程序
        pass
    finally:
        # 销毁节点，释放相关资源
        node.destroy_node()
        
        # 关闭ROS2客户端库，清理所有资源
        rclpy.shutdown()


# Python脚本入口点
# 该脚本作为独立程序运行时会执行main()函数
if __name__ == '__main__':
    main()
