#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
灵巧手多模式控制节点 - ROS2版本

功能说明：
1. 通过ROS2话题控制左右手灵巧手。
2. 支持多种控制模式：位置控制、力矩控制、速度控制。
3. 依次控制每个手指（关节ID 1~6）运动到目标状态。
4. 控制顺序为：小指→无名指→中指→食指→拇指弯曲→拇指旋转。

控制话题：
  - /inspire_hand/ctrl/left_hand  - 左手控制
  - /inspire_hand/ctrl/right_hand - 右手控制

话题数据类型：sensor_msgs/JointState

============================================================================
使用说明 - ROS2命令调用
============================================================================

【位置控制模式】
    ros2 run sdk_demo hand_control pos

【力矩控制模式】
    ros2 run sdk_demo hand_control torque

【速度控制模式】
    ros2 run sdk_demo hand_control vel

【其他参数】
  --target <百分比>：设置目标控制值，默认0.30（30%）
  --interval <秒数>：设置关节控制间隔，默认0.5秒
  示例：ros2 run sdk_demo hand_control pos --target 0.1

============================================================================

"""

from enum import Enum
from typing import Optional
import argparse
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ControlMode(Enum):
    """控制模式枚举
    
    定义灵巧手支持的各种控制模式，便于扩展新模式。
    """
    POS = "pos"      # 位置控制模式
    TORQUE = "torque"          # 力矩控制模式
    VEL = "vel"      # 速度控制模式


class ControlConfig:
    """控制配置类
    
    封装控制参数，便于配置管理和复用。
    
    属性：
        mode (ControlMode): 控制模式
        target_value (float): 目标控制值（位置/力矩/速度比例，范围0.0~1.0）
        interval (float): 发布间隔（秒）
    """
    
    def __init__(
        self,
        mode: ControlMode = ControlMode.POS,
        target_value: float = 0.10,
        interval: float = 0.2
    ):
        """初始化控制配置"""
        self.mode = mode
        self.target_value = target_value
        self.interval = interval


class InspireHandControllerDemo(Node):
    """灵巧手多模式控制节点
    
    该节点实现了一个可扩展的灵巧手控制系统，支持多种控制模式。
    按顺序执行一次握拳动作，完成后自动退出。
    
    属性：
        left_hand_publisher: 左手控制话题发布器
        right_hand_publisher: 右手控制话题发布器
        config: 控制配置
    """

    # ========== 常量定义 ==========
    
    # 关节ID顺序（与需求一致）
    JOINT_ID_SEQUENCE = ["1", "2", "3", "4", "5"]
    # JOINT_ID_SEQUENCE = ["1", "2", "3", "4", "5", "6"]

    # 关节ID对应中文名称（用于日志）
    JOINT_NAME_MAP = {
        "1": "小指",
        "2": "无名指",
        "3": "中指",
        "4": "食指",
        "5": "拇指弯曲",
        "6": "拇指旋转",
    }

    # 控制模式对应的描述文本
    CONTROL_MODE_DESCRIPTIONS = {
        ControlMode.POS: "位置控制",
        ControlMode.TORQUE: "力矩控制",
        ControlMode.VEL: "速度控制",
    }

    def __init__(self, config: Optional[ControlConfig] = None):
        """
        初始化节点
        
        参数：
            config (ControlConfig, optional): 控制配置。如果为None，使用默认配置。
        """
        super().__init__("inspire_hand_controller_demo")

        # ========== 话题发布器初始化 ==========
        
        # 创建左右手控制话题发布器
        self.left_hand_publisher = self.create_publisher(
            JointState, "/inspire_hand/ctrl/left_hand", 10
        )
        self.right_hand_publisher = self.create_publisher(
            JointState, "/inspire_hand/ctrl/right_hand", 10
        )

        # ========== 控制配置初始化 ==========
        
        # 如果未提供配置，使用默认配置
        self.config = config if config is not None else ControlConfig()

        self.get_logger().info(
            f"灵巧手控制节点已初始化\n"
            f"  控制模式：{self.CONTROL_MODE_DESCRIPTIONS.get(self.config.mode, '未知')}\n"
            f"  目标值：{self.config.target_value:.2f}\n"
            f"  关节间隔：{self.config.interval}秒"
        )

    def execute_hand_grasp(self):
        """
        执行握拳动作 - 一次性顺序控制所有关节
        
        该方法按顺序控制每个手指到目标位置，完成后自动返回。
        不使用定时器，直接用 time.sleep 实现延迟，更直观简洁。
        """
        self.get_logger().info(
            f"开始执行握拳动作（{self.CONTROL_MODE_DESCRIPTIONS.get(self.config.mode, '未知模式')}）..."
        )
        
        # 遍历所有关节，依次执行控制
        for i, joint_id in enumerate(self.JOINT_ID_SEQUENCE, start=1):
            # 根据控制模式调用相应的控制方法
            if self.config.mode == ControlMode.POS:
                self._execute_position_control(joint_id)
            elif self.config.mode == ControlMode.TORQUE:
                self._execute_torque_control(joint_id)
            elif self.config.mode == ControlMode.VEL:
                self._execute_velocity_control(joint_id)
            else:
                self.get_logger().warning(f"未知的控制模式：{self.config.mode}")
                continue
            
            # 除了最后一个关节，其他关节之间留有延迟
            if i < len(self.JOINT_ID_SEQUENCE):
                time.sleep(self.config.interval)
        
        # 握拳动作完成
        self.get_logger().info(
            "✓ 握拳动作已完成，所有关节按顺序执行结束"
        )

    def _execute_position_control(self, joint_id: str):
        """
        位置控制模式执行函数
        
        发布单个关节的位置控制指令，使关节运动到目标位置。
        
        参数：
            joint_id (str): 关节ID（字符串形式）
        """
        # 获取关节中文名称
        joint_name = self.JOINT_NAME_MAP.get(joint_id, f"未知关节{joint_id}")

        # 构建位置控制消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [joint_id]
        msg.position = [self.config.target_value]

        # 发布到左右手控制话题
        self.left_hand_publisher.publish(msg)
        self.right_hand_publisher.publish(msg)

        # 输出控制日志
        self.get_logger().info(
            f"[位置控制] 关节：{joint_name}(ID={joint_id})，"
            f"目标位置：{self.config.target_value}"
        )

    def _execute_torque_control(self, joint_id: str):
        """
        力矩控制模式执行函数
        
        发布单个关节的力矩控制指令。
        在此模式下，effort字段用于指定目标力矩百分比。
        
        参数：
            joint_id (str): 关节ID（字符串形式）
        """
        # 获取关节中文名称
        joint_name = self.JOINT_NAME_MAP.get(joint_id, f"未知关节{joint_id}")

        # 构建力矩控制消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [joint_id]
        # 力矩控制使用effort字段
        msg.effort = [self.config.target_value]

        # 发布到左右手控制话题
        self.left_hand_publisher.publish(msg)
        self.right_hand_publisher.publish(msg)

        # 输出控制日志
        self.get_logger().info(
            f"[力矩控制] 关节：{joint_name}(ID={joint_id})，"
            f"目标力矩：{self.config.target_value}"
        )

    def _execute_velocity_control(self, joint_id: str):
        """
        速度控制模式执行函数
        
        发布单个关节的速度控制指令。
        在此模式下，velocity字段用于指定目标速度百分比。
        
        参数：
            joint_id (str): 关节ID（字符串形式）
        """
        # 获取关节中文名称
        joint_name = self.JOINT_NAME_MAP.get(joint_id, f"未知关节{joint_id}")

        # 构建速度控制消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [joint_id]
        # 速度控制使用velocity字段
        msg.vel = [self.config.target_value]

        # 发布到左右手控制话题
        self.left_hand_publisher.publish(msg)
        self.right_hand_publisher.publish(msg)

        # 输出控制日志
        self.get_logger().info(
            f"[速度控制] 关节：{joint_name}(ID={joint_id})，"
            f"目标速度：{self.config.target_value}"
        )

    def add_new_control_mode(self, mode: ControlMode, description: str):
        """
        注册新的控制模式
        
        未来如需添加新的控制模式（如力反馈控制、自适应控制等），
        可通过此方法进行注册。
        
        参数：
            mode (ControlMode): 新的控制模式
            description (str): 模式描述
        """
        self.CONTROL_MODE_DESCRIPTIONS[mode] = description
        self.get_logger().info(f"已注册新的控制模式：{description}")


def main(args=None):
    
    # ========== 命令行参数解析 ==========
    
    # 定义参数解析器
    parser = argparse.ArgumentParser(
        description="灵巧手多模式控制节点\n"
                    "支持位置、力矩、速度三种控制模式",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例用法：\n"
               "  位置控制：ros2 run sdk_demo hand_control pos\n"
               "  力矩控制：ros2 run sdk_demo hand_control torque --target 0.50\n"
               "  速度控制：ros2 run sdk_demo hand_control vel"
    )
    
    # 位置参数：控制模式（必需）
    parser.add_argument(
        'mode',
        choices=['pos', 'torque', 'vel'],
        help="控制模式选择："
             "pos 为位置控制，"
             "torque 为力矩控制，"
             "vel 为速度控制"
    )
    
    # 可选参数：目标值
    parser.add_argument(
        '--target',
        type=float,
        default=0.50,
        help="目标控制值比例（范围0.0~1.0，默认：0.50）"
    )
    
    # 可选参数：发布间隔
    parser.add_argument(
        '--interval',
        type=float,
        default=0.3,
        help="关节控制间隔，单位秒（默认：0.2秒）"
    )
    
    # 解析命令行参数
    # 注意：ROS2会在args中传入额外的参数，需要分离
    parsed_args = parser.parse_args(args)
    
    # ========== 根据参数选择控制模式 ==========
    
    # 将模式字符串映射到ControlMode枚举
    mode_map = {
        'pos': ControlMode.POS,
        'torque': ControlMode.TORQUE,
        'vel': ControlMode.VEL
    }
    
    selected_mode = mode_map[parsed_args.mode]
    
    # 创建控制配置
    config = ControlConfig(
        mode=selected_mode,
        target_value=parsed_args.target,
        interval=parsed_args.interval
    )
    
    # ========== ROS2初始化和运行 ==========
    
    rclpy.init(args=args)
    
    # 创建节点实例
    node = InspireHandControllerDemo(config=config)
    
    try:
        # 执行握拳动作（一次性动作）
        node.execute_hand_grasp()
        
        # 动作执行完毕，正常退出
        node.get_logger().info("程序执行完成，退出")
        
    except KeyboardInterrupt:
        # 捕获Ctrl+C中断信号
        print("\n程序被用户中断")
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
