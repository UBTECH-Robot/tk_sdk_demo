#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂电机控制演示脚本

功能说明：
    本脚本演示了如何使用三种不同的控制模式来控制双臂电机的运动
    
使用方法(要先确保机器人本体服务是启动着的)：
    执行位置控制模式示例（默认）：
        ros2 run sdk_demo arm_motor_control pos
    
    执行力位混合控制模式示例：
        ros2 run sdk_demo arm_motor_control imp
    
    执行速度控制模式示例：
        ros2 run sdk_demo arm_motor_control vel
        对应命令行示例：
        ros2 topic pub /arm/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{name: 1, spd: -1, cur: 5.0 }]}" --once
        --once参数表示只发布一次消息，实测电机只会转动一下，然后停止。
    
    仅执行回零示例：
        ros2 run sdk_demo arm_motor_control home
    
    标零示例（实际应用中，标零接口必须配合标零工具使用！否则可能导致电机位置错误，影响机器人正常运行）：
        ros2 run sdk_demo arm_motor_control zero

控制模式说明：

    1. 位置模式（Position Control）
       - 控制话题：/arm/cmd_pos
       - 消息类型：bodyctrl_msgs/msg/CmdSetMotorPosition
       - 特点：指定目标位置和速度限制，电机会自动规划路径运动到目标位置
       - 应用：需要精确位置控制的场景
       - 参数说明：
         * name: 电机ID
         * pos: 目标位置（弧度）
         * spd: 速度限制（弧度/秒）
         * cur: 电流限制（安培）

    2. 力位混合模式（Impedance Control）
       - 控制话题：/arm/cmd_ctrl
       - 消息类型：bodyctrl_msgs/msg/CmdMotorCtrl
       - 特点：同时控制位置和力，可模拟弹性、阻尼等特性，适合与环境交互
       - 应用：需要与环境交互、实现合规运动的场景
       - 参数说明：
         * name: 电机ID
         * kp: 位置刚性系数，值越大位置控制越硬
         * kd: 速度阻尼系数，值越大阻尼效果越强
         * pos: 目标位置（弧度）
         * spd: 目标速度（弧度/秒）
         * tor: 目标扭矩（牛米）

    3. 速度模式（Velocity Control）
       - 控制话题：/arm/cmd_vel
       - 消息类型：bodyctrl_msgs/msg/CmdSetMotorSpeed
       - 特点：指定目标速度，电机会持续按该速度运动，不受位置限制
       - 应用：需要连续运动、速度控制的场景
       - 参数说明：
         * name: 电机ID
         * spd: 目标速度（弧度/秒）
         * cur: 电流限制（安培）

    4. 回零操作（Homing）
       - 功能：让所有电机回到零位（0弧度）
       - 应用：初始化电机位置，执行新的控制模式前的重置步骤
       - 每个控制模式执行前都会自动调用回零操作
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String
from builtin_interfaces.msg import Time
from bodyctrl_msgs.msg import (
    CmdSetMotorPosition, SetMotorPosition,
    CmdMotorCtrl, MotorCtrl,
    CmdSetMotorSpeed, SetMotorSpeed
)
import time
import sys

# 双臂包含14个电机（左右臂各7个电机），电机ID和运动范围：
motor_angle_limits_dict = {
    # 左臂电机
    11: [-170, 170],   #（左肩关节俯仰 Left Shoulder Pitch）: -170度到+170度
    12: [-15, 150],    #（左肩关节翻滚 Left Shoulder Roll）: -15度到+150度
    13: [-170, 170],   #（左肩关节偏航 Left Shoulder Yaw）: -170度到+170度
    14: [-150, 15],    #（左肘关节俯仰 Left Elbow Pitch）: -150度到+15度
    15: [-170, 170],   #（左腕关节偏航 Left Wrist Yaw）: -170度到+170度
    16: [-45, 60],     #（左腕关节俯仰 Left Wrist Pitch）: -45度到+60度
    17: [-95, 75],     #（左腕关节翻滚 Left WristRoll）: -95度到+75度
  
    # 右臂电机  
    21: [-170, 170],   # 右肩关节俯仰 Left Shoulder Pitch）: -170度到+170度
    22: [-150, 15],    # 右肩关节翻滚 Left Shoulder Roll）: -150度到+15度
    23: [-170, 170],   # 右肩关节偏航 Left Shoulder Yaw）: -170度到+170度
    24: [-150, 15],    # 右肘关节俯仰 Left Elbow Pitch）: -150度到+15度
    25: [-170, 170],   # 右腕关节偏航 Left Wrist Yaw）: -170度到+170度
    26: [-45, 60],     # 右腕关节俯仰 Left Wrist Pitch）: -45度到+60度
    27: [-75, 95],     # 右腕关节翻滚 Left WristRoll）: -75度到+95度
}
# 运动超过范围则电机会断开连接，无法再被控制，可手动将电机复位到合理位置后重启机器人本体服务(注意确保重启时机器人是安全固定在移位机上的)

# 电机ID列表，用于批量控制
arm_MOTOR_IDS = [12]

import math

# 控制参数定义
VELOCITY_LIMIT = 1.0  # 速度限制（弧度/秒）
CURRENT_LIMIT = 5.0  # 电流限制（安培）

# 力位混合模式的参数
KP = 20.0  # 位置刚性系数 - 越大位置控制越硬
KD = 10.0  # 速度阻尼系数 - 越大阻尼效果越强

# 速度模式的速度
CONTROL_SPEED = 1.0  # 目标速度（弧度/秒，平缓速度）

# 速度模式安全参数
VELOCITY_MODE_DURATION = 5.0  # 速度模式持续时间（秒）- 防止电机无限运转
VELOCITY_MODE_STOP_DURATION = 0.5  # 停止命令持续时间（秒）- 确保电机停止

def degree_to_radian(degree):
    """
    度转换为弧度，1弧度 ≈ 57.3度，1度 ≈ 0.01745弧度
    """
    return degree * math.pi / 180.0

def radian_to_target_radian(radian):
    """
    将最大弧度转换为目标弧度，暂定为最大弧度的三分之一
    
    :param radian: 电机的最大弧度
    """
    return radian / 3

class HeadMotorController(Node):
    """
    创建一个控制双臂电机的节点，提供三种不同的电机控制模式
    """
    
    def __init__(self):
        """
        初始化双臂电机控制节点
        
        功能：
            1. 调用父类构造函数，创建名为'arm_motor_controller'的节点
            2. 创建三个发布者，分别发送三种不同控制模式的命令
            3. 记录当前电机位置（用于位置模式的相对运动）
        """
        # 初始化节点，节点名称为'arm_motor_controller'
        super().__init__('arm_motor_controller')
        
        # 创建位置模式命令发布者
        # 发布到 /arm/cmd_pos 话题，消息类型为 CmdSetMotorPosition
        self.pos_cmd_publisher = self.create_publisher(
            CmdSetMotorPosition,
            '/arm/cmd_pos',
            10
        )
        self.get_logger().info("✓ 位置模式发布者已创建（话题：/arm/cmd_pos）")
        
        # 创建力位混合模式命令发布者
        # 发布到 /arm/cmd_ctrl 话题，消息类型为 CmdMotorCtrl
        self.ctrl_cmd_publisher = self.create_publisher(
            CmdMotorCtrl,
            '/arm/cmd_ctrl',
            10
        )
        self.get_logger().info("✓ 力位混合模式发布者已创建（话题：/arm/cmd_ctrl）")
        
        # 创建速度模式命令发布者
        # 发布到 /arm/cmd_vel 话题，消息类型为 CmdSetMotorSpeed
        self.vel_cmd_publisher = self.create_publisher(
            CmdSetMotorSpeed,
            '/arm/cmd_vel',
            10
        )
        self.get_logger().info("✓ 速度模式发布者已创建（话题：/arm/cmd_vel）")
        
        # 创建标零命令发布者（需要配合标零工具使用）
        # 发布到 /arm/cmd_set_zero 话题，用于将指定关节的当前位置标记为零位
        self.zero_cmd_publisher = self.create_publisher(
            String,
            '/arm/cmd_set_zero',
            10
        )
        self.get_logger().info("✓ 标零发布者已创建（话题：/arm/cmd_set_zero）")
                
        self.get_logger().info("=" * 50)
        self.get_logger().info("双臂电机控制节点已初始化完成")
        self.get_logger().info("=" * 50)

    def create_header(self):
        """
        创建消息头
        
        返回值：
            Header: 包含时间戳和坐标系信息的消息头
            
        说明：
            所有命令消息都需要包含header信息，用于时间同步和坐标系标识
        """
        # 获取当前时间（秒和纳秒）
        now = self.get_clock().now()
        
        # 创建Header对象
        header = Header()
        header.stamp.sec = int(now.nanoseconds // 1_000_000_000)  # 转换为秒
        header.stamp.nanosec = int(now.nanoseconds % 1_000_000_000)  # 剩余纳秒
        header.frame_id = 'head'  # 坐标系ID
        
        return header

    def homing(self):
        """
        电机回零 - 让所有电机回到零位
        
        功能：
            将所有电机运动到零位（0弧度）
            这是执行其他控制模式前的必要初始化步骤
            确保所有电机位置状态一致
            
        工作原理：
            使用位置模式让所有电机运动到目标位置0弧度
            速度限制为1.0 rad/s，电流限制为5.0A
            
        应用场景：
            - 程序启动时初始化电机位置
            - 执行新的控制模式前重置状态
            - 系统复位或故障恢复
        """
        self.get_logger().info("")
        self.get_logger().info("=" * 50)
        self.get_logger().info("【电机回零】开始执行")
        self.get_logger().info("=" * 50)
        
        # 创建消息头
        header = self.create_header()
        
        # 创建位置模式命令消息
        msg = CmdSetMotorPosition()
        msg.header = header
        
        # 为每个电机创建回零命令
        for motor_id in arm_MOTOR_IDS:
            # 创建单个电机的位置命令
            cmd = SetMotorPosition()
            cmd.name = motor_id  # 电机ID
            cmd.pos = 0.0  # 目标位置：0弧度（零位）
            cmd.spd = VELOCITY_LIMIT  # 速度限制（弧度/秒）
            cmd.cur = CURRENT_LIMIT  # 电流限制（安培）
            
            # 添加到消息数组
            msg.cmds.append(cmd)
            
            self.get_logger().info(f"  电机 {motor_id}：运动到零位（0 rad）")
        
        # 发送命令
        self.pos_cmd_publisher.publish(msg)
        self.get_logger().info("✓ 回零命令已发送")
        
        time.sleep(1)  # 给电机足够的时间运动到零位

    def position_control_mode(self):
        """
        位置控制模式演示
        
        功能：
            演示如何使用位置模式控制双臂电机
            
        控制原理：
            1. 指定目标位置
            2. 指定速度限制
            3. 电机自动规划轨迹并运动到目标位置
            4. 达到目标位置后自动停止
            
        应用场景：
            - 需要精确位置控制
            - 需要可重复的位置运动
        """
        self.get_logger().info("")
        self.get_logger().info("=" * 50)
        self.get_logger().info("【位置模式】开始演示")
        self.get_logger().info("=" * 50)
        
        # 创建消息头
        header = self.create_header()
        
        # 创建位置模式命令消息
        msg = CmdSetMotorPosition()
        msg.header = header
        
        # 为每个电机创建控制命令        
        for motor_id in arm_MOTOR_IDS:
            # 获取该电机的目标位置
            motor_max_pos_degree = motor_angle_limits_dict[motor_id][1]
            target_pos = radian_to_target_radian(degree_to_radian(motor_max_pos_degree))
            
            # 创建单个电机的位置命令
            cmd = SetMotorPosition()
            cmd.name = motor_id  # 电机ID
            cmd.pos = target_pos  # 目标位置（弧度）
            cmd.spd = VELOCITY_LIMIT  # 速度限制（弧度/秒）
            cmd.cur = CURRENT_LIMIT  # 电流限制（安培）
            
            # 添加到消息数组
            msg.cmds.append(cmd)
            
            # 转换为度数显示
            target_deg = target_pos * 180 / math.pi
            self.get_logger().info(
                f"  电机 {motor_id}："
                f" 目标位置={target_pos:.4f} rad ({target_deg:.1f}°)，"
                f" 速度限制={VELOCITY_LIMIT} rad/s，"
                f" 电流限制={CURRENT_LIMIT} A"
            )
        
        # 发送命令
        self.pos_cmd_publisher.publish(msg)
        self.get_logger().info("✓ 位置模式命令已发送")

    def impedance_control_mode(self):
        """
        力位混合模式演示
        
        ===== 通俗理解 =====
        
        想象电机后面装了一个"虚拟弹簧-阻尼系统"：
        
        • kp（位置刚性系数 = 弹簧硬度）
          - 就像弹簧的硬度一样
          - kp=0：完全软，无弹性，电机不会自动回到目标位置
          - kp=20：中等硬度，像橡皮筋，偏离一点会有适中的拉力拉回
          
        • kd（速度阻尼系数 = 减速器/阻尼器）
          - 就像减速器一样，防止运动太快太急
          - kd=0：无阻尼，运动会很快，容易振荡（抖动）
          - kd=10：中等阻尼，运动平稳，像在水里移动
          - kd=20：强阻尼，运动很慢很平稳，但响应迟钝
        
        ===== 参数调节建议 =====
        
        • 如果电机容易抖动/震荡 → 增加 kd（加阻尼）
          例如：kd从10改到20或30
        
        • 如果电机反应太慢/跟不上 → 减少 kd（减阻尼）
          例如：kd从30改到10或5
        
        • 如果电机容易偏离目标位置 → 增加 kp（增加硬度）
          例如：kp从20改到30或40
        
        • 如果电机运动不够柔和/过于僵硬 → 减少 kp（减少硬度）
          例如：kp从20改到10或5
        
        ===== 参数组合建议 =====
        
        • 平衡型（推荐用于一般场景）：kp=20, kd=10
        • 柔软型（与环境交互，防碰撞）：kp=10, kd=5
        
        ===== 功能说明 =====
        
        演示如何使用力位混合模式控制双臂电机
        
        • pos (目标位置)：期望电机运动到这个位置（弧度）
        • spd (目标速度)：期望电机以什么速度运动（0=静止，正值=正向旋转）
        • tor (目标扭矩)：期望施加的额外力（牛米，0=不施加额外力）
        
        工作原理（技术细节）：
        电机的执行命令 = kp×位置偏差 + kd×速度偏差 + 目标扭矩
        简单说就是：
        - 离目标越远 → 用力越大去追
        - 运动越快 → 用力越大去减速
        - 结合起来就像弹簧+阻尼一样
        
        应用场景：
            - 需要与环境交互（如碰撞检测）→ 用 kp 小、kd 小的参数
            - 需要实现柔性控制（安全与人接触）→ 用 kp 中等、kd 中等的参数
            - 需要在外力干扰下维持位置 → 用 kp 大、kd 小的参数
            - 需要限制反应力（与人交互不伤人）→ 用 kp 小的参数
        """
        self.get_logger().info("")
        self.get_logger().info("=" * 50)
        self.get_logger().info("【力位混合模式】开始演示")
        self.get_logger().info("=" * 50)
        
        # 创建消息头
        header = self.create_header()
        
        # 创建力位混合模式命令消息
        msg = CmdMotorCtrl()
        msg.header = header
        
        # 为每个电机创建控制命令        
        for motor_id in arm_MOTOR_IDS:
            # 获取该电机的目标位置            
            motor_max_pos_degree = motor_angle_limits_dict[motor_id][1]
            target_pos = radian_to_target_radian(degree_to_radian(motor_max_pos_degree))
            
            # 创建单个电机的力位混合命令
            cmd = MotorCtrl()
            cmd.name = motor_id  # 电机ID
            cmd.kp = KP  # 位置刚性系数
            cmd.kd = KD  # 速度阻尼系数
            cmd.pos = target_pos  # 目标位置（弧度）
            cmd.spd = 0.0  # 目标速度（弧度/秒，0表示静止）
            cmd.tor = 0.0  # 目标扭矩（牛米，0表示不施加额外扭矩）
            
            # 添加到消息数组
            msg.cmds.append(cmd)
            
            # 转换为度数显示
            target_deg = target_pos * 180 / math.pi
            self.get_logger().info(
                f"  电机 {motor_id}："
                f" kp={KP}（刚性），"
                f" kd={KD}（阻尼），"
                f" 目标位置={target_pos:.4f} rad ({target_deg:.1f}°)"
            )
        
        # 发送命令
        self.ctrl_cmd_publisher.publish(msg)
        self.get_logger().info("✓ 力位混合模式命令已发送")

    def velocity_control_mode(self):
        """
        速度控制模式演示 - 带安全保护
        
        ===== 通俗理解 =====
        
        速度模式就像给电机一个持续的转速命令，电机会一直按这个速度转动。
        
        • spd (目标速度)：告诉电机转多快
          - spd=0.5：以0.5弧度/秒的速度正向旋转
          - spd=-0.5：以0.5弧度/秒的速度反向旋转
          - spd=0：停止转动
        
        • cur (电流限制)：限制电机用多大的力
          - 防止电机过载
          - 保护电源系统        
       
        ===== 本脚本的安全措施 =====
        
        ✓ 时间限制（VELOCITY_MODE_DURATION = 5秒）
          - 电机只运转 5 秒
          - 防止电机无限运转
        
        ===== 实际使用建议 =====
        
        1. 在可控环境中运行（有人监管）
        2. 在运行前检查周围环境是否安全
        3. 有紧急停止按钮时保持手指在其上
        4. 不要在无人看管的情况下运行
        
        ===== 参数说明 =====
        
        - spd (速度)：正值逆时针运动，负值顺时针运动（单位：rad/s）
        - cur (电流限制)：限制电机的最大电流（单位：A）
        
        应用场景：
            - 需要短时间的连续旋转（3秒以内）
            - 需要速度控制而不关心位置
            - 演示和测试用途（生产环境需要更复杂的安全机制）
        """
        self.get_logger().info("")
        self.get_logger().info("=" * 50)
        self.get_logger().info("【速度模式】开始演示（带安全保护）")
        self.get_logger().info("=" * 50)
        
        # 安全警告
        self.get_logger().warn(f"⚠️  将运行 {VELOCITY_MODE_DURATION} 秒后自动停止")
        self.get_logger().warn("⚠️  请确保周围环境安全，远离旋转部件")
        self.get_logger().warn("⚠️  如发现异常，立即按 Ctrl+C 停止程序")
        
        for i in range(int(VELOCITY_MODE_DURATION)):
            # 创建消息头
            header = self.create_header()
            
            # 创建速度模式命令消息
            msg = CmdSetMotorSpeed()
            msg.header = header
            
            # 为每个电机创建控制命令
            self.get_logger().info("\n发送运动命令...")
            for i, motor_id in enumerate(arm_MOTOR_IDS):
                # 创建单个电机的速度命令
                cmd = SetMotorSpeed()
                cmd.name = motor_id  # 电机ID
                
                # 所有电机以相同平缓速度正向运动
                cmd.spd = CONTROL_SPEED
                direction = "正向"
                
                cmd.cur = CURRENT_LIMIT  # 电流限制（安培）
                
                # 添加到消息数组
                msg.cmds.append(cmd)
                
                self.get_logger().info(
                    f"  电机 {motor_id}："
                    f" 目标速度={cmd.spd:.3f} rad/s（{direction}），"
                    f" 电流限制={CURRENT_LIMIT} A"
                )
            
            # 发送命令
            self.vel_cmd_publisher.publish(msg)
            self.get_logger().info("✓ 速度模式命令已发送")
            time.sleep(1)  # 每秒发送一次命令以保持运动
        
        # 实测发现，每发布一个速度模式命令电机会持续运动一段时间（实测约1秒），停止发送命令后电机就不会再转动
        self.get_logger().info("✓ 电机已停止")

    def set_zero(self):
        """
        标零方法 - 将指定关节的当前位置设置为零位
        
        ⚠️ 重要提示：此接口必须配合标零工具使用！否则可能导致电机位置错误，影响机器人正常运行。
        
        功能说明：
            标零的作用是将指定关节的当前物理位置记录为该关节的零位参考点
            
        使用场景：
            - 更换电机后重新进行零点标定
            - 进行姿态矫正或重新定标
            
        标零工具说明：
            标零过程需要使用物理标零工具来完成：
            1. 使用标零工具将机器人的双臂关节固定在设计的零位
            2. 运行此标零方法，向驱动器发送标零命令
            3. 驱动器会在当前物理位置记录零点信息
                        
        安全警告：
            ⚠️ 不正确的标零会导致：
               - 关节位置信息错误
               - 机器人动作异常或危险
               - 可能损伤机器人硬件
            
            ⚠️ 标零前必须：
               1. 确认物理位置已使用标零工具设置到零位
               2. 停止所有其他控制命令
               3. 确保机器人安全固定在移位机上
        """
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().warn("【电机标零】准备执行 - 必须配合标零工具使用！")
        self.get_logger().info("=" * 60)
        
        # 显示警告信息
        self.get_logger().warn("")
        self.get_logger().warn("⚠️  严重警告！执行标零前请确认：")
        self.get_logger().warn("⚠️  1. 所有双臂关节已使用标零工具物理固定在设计的零位")
        self.get_logger().warn("⚠️  2. 机器人已安全固定在移位机上")
        self.get_logger().warn("⚠️  3. 周围没有人员")
        self.get_logger().warn("⚠️  4. 所有其他控制程序已停止")
        self.get_logger().warn("")
        
        # 创建消息头
        header = self.create_header()
        
        self.get_logger().info("发送标零命令到所有双臂关节...")
        
        # 为每个电机发送标零命令
        for motor_id in arm_MOTOR_IDS:
            # 创建标零命令消息（String 类型）
            # 格式：电机ID作为字符串 （如 "1", "2", "3"）
            cmd = String()
            cmd.data = str(motor_id)  # 电机ID 作为字符串
            
            self.get_logger().info(f"  电机 {motor_id}：发送标零命令")
            
            # 发送标零命令
            self.zero_cmd_publisher.publish(cmd)
            time.sleep(0.1)  # 给驱动器处理时间
        
        self.get_logger().warn("")
        self.get_logger().warn("✓ 标零命令已发送")
        self.get_logger().warn("✓ 驱动器现在应该在处理标零请求...")
        self.get_logger().warn("✓ 请等待驱动器完成标零并手动重启 proc_manager 服务")
        
        time.sleep(2)


def main(args=None):
    # 初始化ROS2的Python客户端库
    rclpy.init(args=args)
    
    # 创建双臂电机控制节点实例
    controller = HeadMotorController()
    
    try:
        # 给ROS2系统一些时间来初始化
        time.sleep(1)
        
        # 解析命令行参数
        mode = "position"  # 默认模式
        if len(sys.argv) > 1:
            arg = sys.argv[1].lower()
            
            # 位置模式
            if arg in ["position", "pos", "p"]:
                mode = "position"
            # 阻抗模式
            elif arg in ["impedance", "imp", "i"]:
                mode = "impedance"
            # 速度模式
            elif arg in ["velocity", "vel", "v"]:
                mode = "velocity"
            # 仅回零
            elif arg in ["homing", "home", "h"]:
                mode = "homing"
            # 标零
            elif arg in ["zero", "z"]:
                mode = "zero"
            else:
                controller.get_logger().warn(f"未知的模式：{arg}")
                controller.get_logger().info("支持的模式：position(pos), impedance(imp), velocity(vel), homing(home), zero(z)")
                controller.get_logger().info("默认执行位置控制模式")
                mode = "position"
        
        # 显示执行的模式
        mode_description = {
            "position": "位置控制模式（Position Control）",
            "impedance": "力位混合控制模式（Impedance Control）",
            "velocity": "速度控制模式（Velocity Control）",
            "homing": "仅回零",
            "zero": "标零模式（Set Zero）"
        }
        
        controller.get_logger().info("")
        controller.get_logger().info("=" * 60)
        controller.get_logger().info(f"即将执行：{mode_description.get(mode, '未知模式')}")
        controller.get_logger().info("=" * 60)
        
        # 执行选定的模式
        if mode == "homing":
            # 仅执行回零
            controller.homing()
            
        elif mode == "position":
            # 执行位置模式
            controller.homing()  # 先回零
            time.sleep(1)
            controller.position_control_mode()
            
        elif mode == "impedance":
            # 执行阻抗模式
            controller.homing()  # 先回零
            time.sleep(1)
            controller.impedance_control_mode()
            
        elif mode == "velocity":
            # 执行速度模式
            controller.homing()  # 先回零
            time.sleep(1)
            controller.velocity_control_mode()
        
        elif mode == "zero":
            controller.homing()  # 这里只是示例，所以先回零，确保所有关节都在零位
            time.sleep(1)
            controller.homing()  # 这里只是示例，所以先回零，确保所有关节都在零位
            # 执行标零
            controller.set_zero()
        
        # 完成提示
        controller.get_logger().info("")
        controller.get_logger().info("=" * 60)
        controller.get_logger().info("✓ 执行完成！")
        controller.get_logger().info("=" * 60)
        
        # 保持节点运行以便观察效果
        time.sleep(2)
        
    except KeyboardInterrupt:
        # 捕获键盘中断（Ctrl+C），允许用户优雅地退出程序
        controller.get_logger().warn("\n程序被用户中断（Ctrl+C）")
        pass
    finally:
        # 销毁节点，释放相关资源
        controller.destroy_node()
        
        # 关闭ROS2客户端库，清理所有资源
        rclpy.shutdown()


# Python脚本入口点
# 该脚本作为独立程序运行时会执行main()函数
if __name__ == '__main__':
    main()
