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

import rclpy
from rclpy.node import Node
import sys
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


class MotorStatusMonitor(Node):
    """
    电机状态监控节点类
    
    该类继承自ROS2的Node基类，用于创建一个订阅电机状态话题的节点
    可以根据命令行参数选择订阅不同关节的电机状态话题
    """
    
    def __init__(self):
        """
        初始化电机状态监控节点
        
        功能：
            1. 调用父类构造函数，创建名为'motor_status_monitor'的节点
            2. 根据命令行参数确定要订阅的话题
            3. 创建话题订阅器
            4. 输出提示信息
        """
        # 初始化节点，节点名称为'motor_status_monitor'
        super().__init__('motor_status_monitor')
        
        # 从命令行参数中获取关节名称
        # 如果提供了命令行参数，使用该参数对应的话题；否则使用默认的腰部话题
        # sys.argv[1] 是第一个命令行参数
        # 例如：脚本执行命令为 'python script.py arm' 时，sys.argv[1] = 'arm'
        joint_name = sys.argv[1] if len(sys.argv) > 1 else 'waist'
        
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
        self.get_logger().info(f"已开始订阅话题 {topic_name}，监控电机状态信息")

    def status_callback(self, msg: MotorStatusMsg):
        """
        电机状态消息回调函数
        
        当接收到电机状态话题的消息时，本函数会被自动调用，解析消息中的电机状态数据，并打印各个电机的详细信息
        
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
            # 正常情况下，MotorStatusMsg消息应该包含status数组
            if hasattr(msg, 'status'):
                # 获取电机状态数组
                motor_array = msg.status
                
                # 遍历数组中的每个电机，并打印其详细信息
                for i, motor in enumerate(motor_array):
                    # 打印电机索引和分隔线
                    self.get_logger().info(f"========== 【电机{i}】 ==========")
                    
                    # 打印电机名称/ID
                    self.get_logger().info(f"  电机ID: {motor.name}")
                    
                    # 打印电机位置
                    # 位置值保留3位小数，单位为弧度
                    self.get_logger().info(f"  位置: {motor.pos:.3f} rad")
                    
                    # 打印电机速度
                    # 速度值保留3位小数，单位为弧度每秒
                    self.get_logger().info(f"  速度: {motor.speed:.3f} rad/s")
                    
                    # 打印电机电流
                    # 电流值保留2位小数，单位为安培
                    self.get_logger().info(f"  电流: {motor.current:.2f} A")
                    
                    # 打印电机温度
                    # 温度值保留1位小数，单位为摄氏度
                    self.get_logger().info(f"  温度: {motor.temperature:.1f}℃")
                    
                    # 打印错误码
                    self.get_logger().info(f"  错误码: {motor.error}")
                    
                    # 解析并打印错误码的含义
                    self.parse_error_code(motor.error)
                    
            else:
                # 备用处理方式：如果消息格式不符合预期
                self.get_logger().warn("接收到的消息格式不符合预期，缺少status字段")
                
        except Exception as e:
            # 捕获并打印异常信息
            self.get_logger().error(f"处理电机状态消息时出错: {str(e)}")

    def parse_error_code(self, error_code: int):
        """
        解析并打印错误代码的含义
        
        参数：
            error_code (int): 电机错误代码
            
        功能：
            将数字错误码转换为易读的错误说明，并根据错误类型选择日志级别
            - 错误码为0表示无错误，使用info级别日志
            - 其他错误码使用warn级别日志
        """
        # 从错误码字典中查找对应的错误说明
        error_message = ERROR_CODE_MAP.get(error_code, "未知错误码")
        
        if error_code == 0:
            # 无错误，使用info级别日志
            self.get_logger().info(f"  错误说明: {error_message}")
        else:
            # 有错误，使用warn级别日志进行警告
            self.get_logger().warn(f"  ⚠️ 错误说明: {error_message}")


def main(args=None):
    """
    主函数
    
    参数：
        args: 命令行参数，默认为None
        
    功能：
        1. 初始化ROS2客户端库
        2. 创建电机状态监控节点
        3. 启动节点循环，持续接收和处理消息
        4. 处理键盘中断（Ctrl+C）
        5. 清理资源并关闭ROS2
    """
    # 初始化ROS2的Python客户端库
    rclpy.init(args=args)
    
    # 创建电机状态监控节点实例
    node = MotorStatusMonitor()
    
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
