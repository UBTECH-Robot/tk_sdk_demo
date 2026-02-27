#!/usr/bin/env python3
"""
GraspExecutorNode

抓取执行节点。订阅 /grasp_candidate 话题，接收来自感知节点的抓取候选点，
按以下状态机驱动完整的抓取-放置流程：

    IDLE → CONFIRMING → MOVING → VERIFYING → PLACING → IDLE

用户交互（input()）运行在独立后台线程，不阻塞 rclpy.spin。

ros2 run grab_demo grasp_executor_node
"""

import json
import threading
import time
import traceback
from enum import Enum, auto
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from grab_demo_msgs.msg import GraspCandidate
from grab_demo.arm_control_mixin import ArmControlMixin, VELOCITY_LIMIT
from grab_demo.pose_verification_mixin import PoseVerificationMixin
prepare_pose = {
    "head": [
        {1: 0.0, 2: 0.3, 3: 0.0},
    ]
}
VELOCITY_LIMIT = 0.4   # 速度限制（弧度/秒）
CURRENT_LIMIT  = 5.0   # 电流限制（安培）

class GraspPreparePoseNode(ArmControlMixin, Node):
    def __init__(self):
        super().__init__('grasp_prepare_pose_node')

        self._init_arm_control()
        self._init_head_control()

        self.get_logger().info('=== GraspPreparePoseNode 已启动 ===')

    # ------------------------------------------------------------------
    # 初始化
    # ------------------------------------------------------------------

    def _init_arm_control(self):
        """初始化手臂控制发布者和关节状态订阅"""
        self.arm_pos_cmd_publisher = self.create_publisher(
            CmdSetMotorPosition, '/arm/cmd_pos', 10
        )
        self.get_logger().info('✓ 手臂控制发布者已创建（/arm/cmd_pos）')

        # 发布 IK 解算结果 JSON，供 GUI 可视化（与 ik_client_node 保持一致）
        self.joint_command_pub = self.create_publisher(String, '/gui/joint_command', 10)
        self.get_logger().info('✓ GUI 关节命令发布者已创建（/gui/joint_command）')

    def _init_head_control(self):
        """初始化头部控制发布者"""
        self.head_pos_cmd_publisher = self.create_publisher(CmdSetMotorPosition, '/head/cmd_pos', 10)
        self.get_logger().info("✓ 头部电机位置模式控制发布者已创建（话题：/head/cmd_pos）")
        time.sleep(0.5)  # 确保发布者初始化完成
        self.head_pose_init()

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
            
            # 为每个电机创建回零命令
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

    def _publish_model_ghost(self, joint_positions: dict):
        """将 {11: 1.023345, 12: 0.567890, ...} 形式的关节角度以 JSON 格式发布到 /gui/joint_command，供 GUI 可视化"""
        json_str = json.dumps(joint_positions)
        msg      = String()
        msg.data = json_str
        self.joint_command_pub.publish(msg)

    def _execute_arm_movement(self, joint_positions: dict):
        """向手臂发送运动命令（含 7s 等待，确保动作完成后再继续）"""
        self.arm_to_pose(joint_positions, spd=VELOCITY_LIMIT / 2)

def main():
    rclpy.init()
    node = GraspPreparePoseNode()
    try:
        node.arm_pose_init()

        node.head_pose_init()

    except KeyboardInterrupt:
        print('\n用户中断程序')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
