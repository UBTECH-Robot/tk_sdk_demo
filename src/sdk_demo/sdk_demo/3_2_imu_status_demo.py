#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU状态订阅器演示脚本

功能说明：
    本脚本用于订阅并显示IMU（惯性测量单元）传感器的状态数据
    订阅话题：/imu/status
    消息类型：bodyctrl_msgs/msg/Imu
    
IMU消息包含以下信息：
    - header: 消息头，包含时间戳和坐标系ID
    - orientation: 姿态四元数（x, y, z, w）
    - angular_velocity: 角速度（x, y, z），单位：rad/s
    - linear_acceleration: 线性加速度（x, y, z），单位：m/s²
    - euler: 欧拉角（roll, pitch, yaw），单位：弧度
    - error: 错误代码
    - 各种协方差数据
"""

import rclpy
from rclpy.node import Node

# 导入自定义IMU消息类型
# bodyctrl_msgs是自定义消息包，Imu是其中定义的IMU数据结构
from bodyctrl_msgs.msg import Imu


class ImuStatusSubscriber(Node):
    
    def __init__(self):
        """
        初始化IMU状态订阅器节点
        
        功能：
            1. 调用父类构造函数，创建名为'imu_status_subscriber'的节点
            2. 创建订阅器，订阅'/imu/status'话题
            3. 设置消息队列大小为10
        """
        # 初始化节点，节点名称为'imu_status_subscriber'
        super().__init__('imu_status_subscriber')
        
        # 创建订阅器
        # 参数说明：
        #   - Imu: 订阅的消息类型
        #   - '/imu/status': 订阅的话题名称
        #   - self.listener_callback: 接收到消息时的回调函数
        #   - 10: 消息队列大小，用于缓存消息
        self.subscription = self.create_subscription(
            Imu,
            '/imu/status',
            self.listener_callback,
            10)
        
        # 打印日志，提示订阅已启动
        self.get_logger().info('订阅 /imu/status 话题已启动')

    def listener_callback(self, msg: Imu):
        """
        IMU消息回调函数
        
        当接收到/imu/status话题的消息时，该函数会被自动调用
        
        参数：
            msg (Imu): 接收到的IMU消息对象
            
        功能：
            解析并打印IMU消息中的所有字段数据
        """
        # 打印消息头中的时间戳
        # header.stamp包含秒(sec)和纳秒(nanosec)两部分
        # self.get_logger().info(f'header.stamp: sec={msg.header.stamp.sec}, nanosec={msg.header.stamp.nanosec}')
        
        # 提取并打印姿态四元数
        # 四元数(Quaternion)是表示3D旋转的一种方式，由四个分量(x, y, z, w)组成
        # 四元数可以避免万向锁问题，是表示姿态的常用方法
        ori = msg.orientation
        self.get_logger().info(f'姿态四元数: x={ori.x:.3f}, y={ori.y:.3f}, z={ori.z:.3f}, w={ori.w:.3f}')

        # 提取并打印角速度
        # 角速度表示物体绕x、y、z轴的旋转速度，单位：rad/s（弧度每秒）
        # x轴：横滚（roll）角速度
        # y轴：俯仰（pitch）角速度
        # z轴：偏航（yaw）角速度
        ang_vel = msg.angular_velocity
        self.get_logger().info(f'角速度: x={ang_vel.x:.3f}, y={ang_vel.y:.3f}, z={ang_vel.z:.3f}')

        # 提取并打印线性加速度
        # 线性加速度表示物体在x、y、z轴方向的加速度，单位：m/s²
        # 包含重力加速度的影响（静止时z轴约为9.8 m/s²）
        lin_acc = msg.linear_acceleration
        self.get_logger().info(f'线性加速度: x={lin_acc.x:.3f}, y={lin_acc.y:.3f}, z={lin_acc.z:.3f}')

        # 提取并打印欧拉角
        # 欧拉角是另一种表示姿态的方式，更直观易懂，单位：弧度
        # roll：横滚角，绕x轴旋转
        # pitch：俯仰角，绕y轴旋转
        # yaw：偏航角，绕z轴旋转
        euler = msg.euler
        self.get_logger().info(f'欧拉角: roll={euler.roll:.3f}, pitch={euler.pitch:.3f}, yaw={euler.yaw:.3f}')

        # 打印错误代码
        # error字段用于指示IMU是否正常工作，0表示正常
        self.get_logger().info(f'错误代码: {msg.error}')
        # 打印协方差数据
        # 协方差用于表示测量的不确定性/噪声水平
        # 协方差数组是float64[3]类型，分别对应x、y、z三个轴的方差
        
        # 角速度协方差：表示角速度测量的不确定性
        self.get_logger().info(f'角速度协方差: {list(msg.angular_velocity_covariance)}')
        
        # 姿态协方差：表示姿态测量的不确定性
        self.get_logger().info(f'姿态协方差: {list(msg.orientation_covariance)}')
        
        # 线性加速度协方差：表示加速度测量的不确定性
        self.get_logger().info(f'线性加速度协方差: {list(msg.linear_acceleration_covariance)}\n')


def main(args=None):
    """
    主函数
    
    参数：
        args: 命令行参数，默认为None
        
    功能：
        1. 初始化ROS2客户端库
        2. 创建IMU状态订阅器节点
        3. 启动节点循环，持续接收消息
        4. 处理键盘中断（Ctrl+C）
        5. 清理资源并关闭ROS2
    """
    # 初始化ROS2的Python客户端库
    rclpy.init(args=args)
    
    # 创建IMU状态订阅器节点实例
    node = ImuStatusSubscriber()
    
    try:
        # 启动节点循环，持续处理回调函数
        # spin()会阻塞程序，直到节点被关闭
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获键盘中断（Ctrl+C），优雅地退出程序
        pass
    finally:
        # 销毁节点，释放相关资源
        node.destroy_node()
        
        # 关闭ROS2客户端库
        rclpy.shutdown()


# Python脚本入口点
if __name__ == '__main__':
    main()
