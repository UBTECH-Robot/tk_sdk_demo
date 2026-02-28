#!/usr/bin/env python3
"""
GraspPreparePoseNode

抓取准备节点。在抓取开始前，将手臂、头部、手指初始化到准备姿态。

ros2 run grab_demo grasp_prepare_pose_node
"""

import time
import rclpy
from rclpy.node import Node
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
from grab_demo.arm_control_mixin import ArmControlMixin, VELOCITY_LIMIT, CURRENT_LIMIT
from grab_demo.hand_control_mixin import HandControlMixin
prepare_pose = {
    "head": [
        {1: 0.0, 2: 0.3, 3: 0.0},
    ]
}

class GraspPreparePoseNode(ArmControlMixin, HandControlMixin, Node):
    def __init__(self):
        super().__init__('grasp_prepare_pose_node')

        self._init_head_control()

        self.get_logger().info('=== GraspPreparePoseNode 已启动 ===')

    # ------------------------------------------------------------------
    # 初始化
    # ------------------------------------------------------------------
    def _init_head_control(self):
        """初始化头部控制发布者"""
        self.head_pos_cmd_publisher = self.create_publisher(CmdSetMotorPosition, '/head/cmd_pos', 10)
        self.get_logger().info("✓ 头部电机位置模式控制发布者已创建（话题：/head/cmd_pos）")
        time.sleep(0.5)  # 确保发布者初始化完成

    def hand_pose_init(self):
        """手指初始化为张开状态"""
        self.get_logger().info('执行手指张开初始化...')
        self.hand_open()
        self.get_logger().info('✓ 手指已张开')

    def head_pose_init(self):
        """
        头部电机位置调整，以便让头部相机可顺利看到正前方的桌面上的物体
        """
        self.get_logger().info("")
        self.get_logger().info("=" * 50)
        self.get_logger().info("【头部电机位置调整】开始执行")
        self.get_logger().info("=" * 50)
        
        for pose in prepare_pose['head']:
            # 创建消息头
            header = self._create_header("head")
            
            # 创建位置模式命令消息
            msg = CmdSetMotorPosition()
            msg.header = header
            
            for motor_id, position in pose.items():
                # 创建单个电机的位置命令
                cmd = SetMotorPosition()
                cmd.name = motor_id  # 电机ID
                cmd.pos = position  # 目标位置
                cmd.spd = VELOCITY_LIMIT  # 速度限制（弧度/秒）
                cmd.cur = CURRENT_LIMIT  # 电流限制（安培）
                
                # 添加到消息数组
                msg.cmds.append(cmd)
                
                self.get_logger().info(f"  电机 {motor_id}：运动到位置（{position} rad）")
            
            # 发送命令
            self.head_pos_cmd_publisher.publish(msg)
            self.get_logger().info("✓ 头部电机位置调整命令已发送")
            time.sleep(1.5)  # 给电机足够的时间运动到位

def main():
    rclpy.init()
    node = GraspPreparePoseNode()
    try:
        node.arm_pose_init()

        node.head_pose_init()

        node.hand_pose_init()

    except KeyboardInterrupt:
        print('\n用户中断程序')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
