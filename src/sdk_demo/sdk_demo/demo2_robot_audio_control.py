#!/usr/bin/env python3

import os
from rclpy.node import Node
from hric_msgs.srv import SetMotionMode
from geometry_msgs.msg import TwistStamped
from hric_msgs.msg import MotionStatus
from std_msgs.msg import String
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from enum import Enum

class WalkMode(Enum):
    Stop = 0
    Zero = 1
    Stand = 3
    Walk = 5
    Run = 7


qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

class RobotAudioControl(Node):
    def __init__(self):
        super().__init__('robot_audio_control')
        self.audio_text_subscription = self.create_subscription(
            String,
            '/audio_text',
            self.audio_text_callback,
            qos
        )
        self.audio_text_subscription  # prevent unused variable warning
        self.get_logger().info("RobotAudioControl 节点已启动，正在订阅 audio_text")

        self.set_motion_mode_cli = self.create_client(SetMotionMode, '/hric/motion/set_motion_mode')

        while not self.set_motion_mode_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/hric/motion/set_motion_mode service not available, waiting...')

        self.vel_publisher = self.create_publisher(TwistStamped, '/hric/robot/cmd_vel', 10)

        self.is_console_control = False  # True 表示导航操控，False 表示遥控器操控
        self.walk_mode = WalkMode.Stop   # 行走状态：0-stop，1-zero，3-stand，5-walk，7-run

        self.walk_status_subscription = self.create_subscription(
            MotionStatus,
            '/hric/motion/status',
            self.motion_status_callback,
            10
        )
        self.walk_status_subscription

    def audio_text_callback(self, msg: String):
        self.get_logger().info(f"Received audio text: {msg.data}")

        if not self.is_console_control:
            self.get_logger().info("当前是遥控器操控模式，忽略语音指令")
            return

        if self.walk_mode is WalkMode.Stop and "回零" in msg.data:
            self.get_logger().info("收到回零指令，设置运动模式为回零")
            self.set_motion_mode(2, is_need_swing_arm=True)
        elif self.walk_mode is WalkMode.Zero and "站立" in msg.data:
            self.get_logger().info("收到站立指令，设置运动模式为站立")
            self.set_motion_mode(3, is_need_swing_arm=True)
        elif (self.walk_mode is WalkMode.Stand or self.walk_mode is WalkMode.Run) and "原地踏步" in msg.data:
            self.get_logger().info("收到原地踏步指令，设置运动模式为原地踏步")
            self.set_motion_mode(4, is_need_swing_arm=True)
        elif self.walk_mode is WalkMode.Walk and "原地跑步" in msg.data:
            self.get_logger().info("收到原地跑步指令，设置运动模式为原地跑步")
            self.set_motion_mode(5, is_need_swing_arm=True)
        # elif self.walk_mode is WalkMode.Stand and ("失能" in msg.data or "僵停" in msg.data or "下电" in msg.data):
        #     self.get_logger().info("收到失能指令，设置运动模式为僵停")
        #     self.set_motion_mode(1, is_need_swing_arm=True)
        # 这样用语音指令可以让机器人失能太危险了，暂时不支持
        

    def motion_status_callback(self, msg: MotionStatus):
        self.is_console_control = msg.is_console_control
        self.walk_mode = WalkMode(msg.walk_mode)

    def publish_velocity(self, linear_x: float, linear_y: float = 0.0, linear_z: float = 0.0,
                         angular_x: float = 0.0, angular_y: float = 0.0, angular_z: float = 0.0):
        ''' 发布速度指令到机器人
        :param linear_x: 线速度 x 分量，正值为前进，负值为后退
        :param linear_y: 线速度 y 分量
        :param linear_z: 线速度 z 分量，为 0 即可，因为天工没法上下运动
        :param angular_x: 角速度 x 分量
        :param angular_y: 角速度 y 分量
        :param angular_z: 角速度 z 分量，正值为左转，负值为右转
        '''
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = linear_x
        msg.twist.linear.y = linear_y
        msg.twist.linear.z = linear_z

        msg.twist.angular.x = angular_x
        msg.twist.angular.y = angular_y
        msg.twist.angular.z = angular_z

        self.vel_publisher.publish(msg)
        self.get_logger().info(f'Published velocity command: linear.x={linear_x}')

    def set_motion_mode(self, walk_mode_request: int, is_need_swing_arm: bool = False):
        '''
        调用服务设置机器人的运动模式
        :param walk_mode_request: 运动模式请求, 1: 僵停, 2: 回零, 3: 站立, 4: 原地踏步, 5: 原地跑步

        '''
        request = SetMotionMode.Request()
        request.walk_mode_request = walk_mode_request
        request.is_need_swing_arm = is_need_swing_arm

        future = self.set_motion_mode_cli.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Response received: {future.result()}')
            return future.result()
        else:
            self.get_logger().error('Service call failed')
            return None
    
def main(args=None):
    rclpy.init(args=args)
    robot_audio_control = RobotAudioControl()

    def stop_handle():        
        print("接收到终止信号，准备终止程序...")

        robot_audio_control.destroy_node()
        print("机器人语音控制节点已销毁，正在关闭 rclpy...")
        if rclpy.ok():
            rclpy.shutdown()

    try:
        print("开始接收音频数据...")
        rclpy.spin(robot_audio_control)
    except KeyboardInterrupt:
        print("接收到 Ctrl+C，准备退出...")
    finally:
        stop_handle()

if __name__ == '__main__':
    main()

# colcon build
# source install/setup.bash

# ros2 run sdk_demo robot_audio_control
