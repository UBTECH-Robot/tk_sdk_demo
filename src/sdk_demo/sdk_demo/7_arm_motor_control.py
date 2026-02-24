#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂电机控制演示脚本

功能说明：
    本脚演示了如何使用三种不同的控制模式来控制双臂电机的运动
    
使用方法(要先确保机器人是固定在移位机上，本体服务是启动着的，但是不能启动运控服务，也就是手动关掉 proc_manager 服务后再执行 ros2 launch body_control body.launch.py)：
    执行位置控制模式示例（默认）：
        ros2 run sdk_demo arm_motor_control pos
        # 直接用ros命令控制的示例：
        # ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 17, pos: 0.6024, spd: 0.2, cur: 8.0 }]}"
    
    执行力位混合控制模式示例：
        ros2 run sdk_demo arm_motor_control imp
    
    执行速度控制模式示例（手臂只会运行3秒，停止后关节位置也不会固定，是不受力的状态，有可能受重力影响往下掉）：
        ros2 run sdk_demo arm_motor_control vel
    
    仅执行回零示例：
        ros2 run sdk_demo arm_motor_control home

    仅右手挥手示例：
        ros2 run sdk_demo arm_motor_control wave
    
    标零示例（标零接口必须配合标零工具使用！否则可能导致电机位置错误，影响机器人正常运行），没有标零工具就不要随意尝试此方法，很危险！

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
    # 左臂电机，数组前两个元素是运动范围，第三个元素是设定的位置模式和力位混合模式的目标角度
    11: [-170, 170, 20],   #（左肩关节俯仰 Left Shoulder Pitch）: -170度到+170度
    12: [-15, 150, 10],    #（左肩关节翻滚 Left Shoulder Roll）: -15度到+150度
    13: [-170, 170, 30],   #（左肩关节偏航 Left Shoulder Yaw）: -170度到+170度
    14: [15, -150, -40],    #（左肘关节俯仰 Left Elbow Pitch）: +15度到-150度
    15: [-170, 170, 30],   #（左腕关节偏航 Left Wrist Yaw）: -170度到+170度，仅无疆版本支持
    16: [60, -45, -20],     #（左腕关节俯仰 Left Wrist Pitch）: -45度到+60度，仅无疆版本支持
    17: [75, -95, -30],     #（左腕关节翻滚 Left Wrist Roll）: -95度到+75度，仅无疆版本支持
  
    # 右臂电机  
    21: [-170, 170, 20],   # 右肩关节俯仰 Right Shoulder Pitch）: -170度到+170度
    22: [15, -150, -10],    # 右肩关节翻滚 Right Shoulder Roll）: +15度到-150度
    23: [-170, 170, -30],   # 右肩关节偏航 Right Shoulder Yaw）: -170度到+170度
    24: [15, -150, -40],    # 右肘关节俯仰 Right Elbow Pitch）: +15度到-150度
    25: [-170, 170, -30],   # 右腕关节偏航 Right Wrist Yaw）: -170度到+170度，仅无疆版本支持
    26: [60, -45, -20],     # 右腕关节俯仰 Right Wrist Pitch）: -45度到+60度，仅无疆版本支持
    27: [-75, 95, 30],     # 右腕关节翻滚 Right Wrist Roll）: -75度到+95度，仅无疆版本支持
}
# 运动超过范围则电机会断开连接，无法再被控制，可手动将电机复位到合理位置后重启机器人本体服务(注意确保重启时机器人是安全固定在移位机上的)

wave_right_arm_motor_pos_list = [
  {"duration": 1.5, 21: 2.951215,    22: -7.782454, 23: -17.252648, 24: -17.236174,  25: -17.010969,  26: -16.336146, 27: 8.412006  },
  {"duration": 1.5, 21: 27.949216,   22: -3.690023, 23: -12.126181, 24: -77.161084,  25: -11.480701,  26: -5.902431,  27: -0.522127  },
  {"duration": 3, 21: 18.90610,    22: -3.690023, 23: -15.909614, 24: -127.037666, 25: 64.002233,   26: 1.032424,   27: -6.064361  },
  {"duration": 1.5, 21: 10.007183,   22: -3.685953, 23: -15.386394, 24: -141.891197, 25: 60.680248,   26: 0.803503,   27: -27.170629  },
  {"duration": 1.5, 21: 5.982044,    22: -3.690023, 23: 2.888050,   24: -136.061548, 25: 86.319603,   26: 6.234351,   27: -38.185347  },
  {"duration": 1.5, 21: 5.986169,    22: -3.690023, 23: 17.748438,  24: -136.207113, 25: 86.32646,    26: 8.82078,    27: -37.248546  },
  {"duration": 1.5, 21: 5.983464,    22: -3.688657, 23: -13.947137, 24: -135.379021, 25: 64.947039,   26: 1.267984,   27: -36.4157  },
  {"duration": 1.5, 21: 5.986169,    22: -3.687319, 23: 17.984653,  24: -139.564834, 25: 86.98153,    26: 8.169097,   27: -35.539933  },
  {"duration": 1.5, 21: 5.979339,    22: -3.685953, 23: -15.70908,  24: -134.962926, 25: 71.510032,   26: 3.035937,   27: -36.259125  },
  {"duration": 1.5, 21: 5.982044,    22: -3.690023, 23: 13.491208,  24: -139.251738, 25: 89.305161,   26: 11.777213,  27: -36.517989  },
  {"duration": 3, 21: 5.979339,    22: -3.685953, 23: -25.853577, 24: -134.719881, 25: 63.826451,   26: -0.715394,  27: -36.52695  },
  {"duration": 1.5, 21: 23.643957,   22: -5.736266, 23: -5.968329,  24: -86.936205,  25: 17.887118,   26: -6.182032,  27: -3.912086  },
  {"duration": 1.5, 21: 5.266568,    22: -4.129478, 23: -13.249501, 24: -34.026063,  25: -7.266065,   26: -11.226645, 27: -1.407702  }
]

# 电机ID列表，用于批量控制
# arm_MOTOR_IDS = [11, 12, 13, 14, 15, 16, 17]
# arm_MOTOR_IDS = [21, 22, 23, 24, 25, 26, 27]
arm_MOTOR_IDS = [11, 12, 13, 14, 15, 16, 17, 21, 22, 23, 24, 25, 26, 27]
# arm_MOTOR_IDS = [26, 27]
# arm_MOTOR_IDS = [11, 12, 13, 14, 15, 16]

import math

# 控制参数定义
VELOCITY_LIMIT = 0.1  # 速度限制（弧度/秒）
CURRENT_LIMIT = 5.0  # 电流限制（安培）

# 力位混合模式的参数
KP = 20.0  # 位置刚性系数 - 越大位置控制越硬
KD = 10.0  # 速度阻尼系数 - 越大阻尼效果越强

# 速度模式的速度
CONTROL_SPEED = 0.1  # 目标速度（弧度/秒，平缓速度）

# 速度模式安全参数
VELOCITY_MODE_DURATION = 3.0  # 速度模式持续时间（秒）- 防止电机无限运转

leg_MOTOR_IDS = [51, 52, 53, 54, 55, 56, 61, 62, 63, 64, 65, 66]
motor_angle_limits_dict_leg = {
    51: -5.8,
    61: 5.8
}
def degree_to_radian(degree):
    """
    度转换为弧度，1弧度 ≈ 57.3度，1度 ≈ 0.01745弧度
    """
    d = degree
    if abs(d) > 60:
        d = 60 if d > 0 else -60
    return d * math.pi / 180.0

class ArmMotorController(Node):
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

        # 为了手臂回零位时不被腿部挡住，要先让双腿电机在安全位置，因此也创建一个腿部位置模式发布者
        self.leg_pos_cmd_publisher = self.create_publisher(
            CmdSetMotorPosition,
            '/leg/cmd_pos',
            10
        )
        self.get_logger().info("✓ 双腿位置模式发布者已创建（话题：/leg/cmd_pos）")
                
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
        header.frame_id = 'arm'  # 坐标系ID
        
        return header

    def leg_safe(self):
        """
        双腿电机到安全位置
        """
        self.get_logger().info("")
        self.get_logger().info("=" * 50)
        self.get_logger().info("【双腿电机到安全位置】开始执行")
        self.get_logger().info("=" * 50)
        
        # 创建消息头
        header = self.create_header()
        
        # 创建位置模式命令消息
        msg = CmdSetMotorPosition()
        msg.header = header
        
        # 为每个电机创建回零命令
        for motor_id in leg_MOTOR_IDS:
            # 创建单个电机的位置命令
            target_pos = degree_to_radian(motor_angle_limits_dict_leg.get(motor_id, 0))
            cmd = SetMotorPosition()
            cmd.name = motor_id  # 电机ID
            cmd.pos = target_pos  # 目标位置（弧度）
            cmd.spd = VELOCITY_LIMIT * 2  # 速度限制（弧度/秒）
            cmd.cur = CURRENT_LIMIT * 2  # 电流限制（安培）
            
            # 添加到消息数组
            msg.cmds.append(cmd)
            
            self.get_logger().info(f"  腿部电机 {motor_id}：运动到目标位置（{target_pos:.4f} rad）")
        
        # 发送命令
        self.leg_pos_cmd_publisher.publish(msg)
        self.get_logger().info("✓ 双腿安全位置命令已发送")
        
        time.sleep(3)  # 给电机足够的时间运动到零位

    def wave(self):
        self.leg_safe()
        self.leg_safe()

        self.get_logger().info("")
        self.get_logger().info("=" * 50)
        self.get_logger().info("【挥手】开始执行")
        self.get_logger().info("=" * 50)
        
        for pos in wave_right_arm_motor_pos_list:
            # 创建消息头
            header = self.create_header()
            
            # 创建位置模式命令消息
            msg = CmdSetMotorPosition()
            msg.header = header
            
            # 为每个电机创建位置命令
            for motor_id in arm_MOTOR_IDS:
                if motor_id not in pos:
                    continue
                
                cmd = SetMotorPosition()
                cmd.name = motor_id  # 电机ID

                motor_pos_degree = pos[motor_id]
                target_pos = motor_pos_degree * math.pi / 180.0 

                cmd.pos = target_pos  # 目标位置（弧度）
                cmd.spd = 0.4  # 速度限制（弧度/秒）
                cmd.cur = CURRENT_LIMIT  # 电流限制（安培）
                
                # 添加到消息数组
                msg.cmds.append(cmd)
                
                self.get_logger().info(f"  电机 {motor_id}：运动到{motor_pos_degree}度")
            
            # 发送命令
            self.pos_cmd_publisher.publish(msg)            
            time.sleep(pos["duration"])  # 给电机足够的时间运动到指定位置

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
        self.leg_safe()
        self.leg_safe()

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
            # print(motor_angle_limits_dict[motor_id])
            motor_pos_degree = motor_angle_limits_dict[motor_id][2]
            target_pos = degree_to_radian(motor_pos_degree)
            
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
            motor_pos_degree = motor_angle_limits_dict[motor_id][2]
            target_pos = degree_to_radian(motor_pos_degree)
            
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
          - spd=0.1：以0.1弧度/秒的速度正向旋转
          - spd=-0.1：以0.1弧度/秒的速度反向旋转
          - spd=0：停止转动
        
        • cur (电流限制)：限制电机用多大的力
          - 防止电机过载
          - 保护电源系统        
       
        ===== 本脚本的安全措施 =====
        
        ✓ 时间限制（VELOCITY_MODE_DURATION）
          - 电机只运转指定时间（秒）
          - 防止电机无限运转
        
        ===== 实际使用建议 =====
        
        1. 在可控环境中运行（有人监管）
        2. 在运行前检查周围环境是否安全
        3. 有紧急停止按钮时保持手指在其上
        4. 不要在无人看管的情况下运行
        
        ===== 参数说明 =====
        
        - spd (速度)：正值逆时针运动，负值顺时针运动（单位：rad/s）
        - cur (电流限制)：限制电机的最大电流（单位：A）
        
        """
        self.get_logger().info("")
        self.get_logger().info("=" * 50)
        self.get_logger().info("【速度模式】开始演示（带安全保护）")
        self.get_logger().info("=" * 50)
        
        # 安全警告
        self.get_logger().warn(f"⚠️  将运行 {VELOCITY_MODE_DURATION} 秒后自动停止")
        self.get_logger().warn("⚠️  请确保周围环境安全，远离旋转部件")
        self.get_logger().warn("⚠️  如发现异常，立即按 Ctrl+C 停止程序")
        
        spd = 0.0

        running_start_time = None   # 记录进入恒速阶段的时间

        while True:
            if spd < CONTROL_SPEED:
                spd = spd + 0.05 # 速度模式需要速度从0开始逐步增加，直到达到设定速度，速度跳变太大会给人感觉电机抖动
            if spd >= CONTROL_SPEED:
                spd = CONTROL_SPEED
                if running_start_time is None:
                    running_start_time = time.time()

                if time.time() - running_start_time >= VELOCITY_MODE_DURATION:
                    break
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
                
                motor_pos_degree = motor_angle_limits_dict[motor_id][2]
                
                # 所有电机以相同平缓速度正向运动
                cmd.spd = spd if motor_pos_degree >= 0 else -spd
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
            time.sleep(0.005)
        
        # 实测发现，每发布一个速度模式命令电机会持续运动一段时间（实测约1秒），停止发送命令后电机就不会再转动
        self.velocity_stop_control()
        time.sleep(1)
        self.velocity_stop_control()
        # 为确保安全，连续发两次确保电机停止转动

        self.get_logger().info("✓ 电机已停止")

    def velocity_stop_control(self):
        """
        速度模式停止 - 发送停止命令给所有电机
        
        功能：
            向所有双腿电机发送 spd=0 的命令，使电机停止运动
            
        应用场景：
            - 速度模式运动后的安全停止
            - 紧急停止
        """
        self.get_logger().info("")
        self.get_logger().info("=" * 50)
        self.get_logger().info("【速度模式停止】发送停止命令")
        self.get_logger().info("=" * 50)
        
        # 创建消息头
        header = self.create_header()
        
        # 创建速度模式命令消息
        msg = CmdSetMotorSpeed()
        msg.header = header
        
        # 为每个电机创建停止命令
        self.get_logger().info("发送停止命令...")
        for motor_id in arm_MOTOR_IDS:
            # 创建单个电机的停止命令
            cmd = SetMotorSpeed()
            cmd.name = motor_id  # 电机ID
            cmd.spd = 0.0  # 速度为0，停止运动
            cmd.cur = CURRENT_LIMIT  # 电流限制（安培）
            
            # 添加到消息数组
            msg.cmds.append(cmd)
            
            self.get_logger().info(f"  电机 {motor_id}：发送停止命令（spd=0）")
        
        # 发送命令
        self.vel_cmd_publisher.publish(msg)
        self.get_logger().info("✓ 停止命令已发送")

    def set_zero(self):
        """
        标零方法 - 将指定关节的当前位置设置为零位
        
        ⚠️ 重要提示：此接口必须配合标零工具使用！否则可能导致电机位置错误，影响机器人正常运行。没有标零工具就不要随意尝试此方法，很危险！
        
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

        joint_ids = ", ".join(str(motor_id) for motor_id in arm_MOTOR_IDS)
        user_input = input(f"请确认机器人关节[{joint_ids}]是否真的在零位，标零操作会将这些关节当前位置设置为零位，是否继续？(y/n)：").strip().lower()
        
        # 检查用户输入
        if user_input in ['是', 'yes', 'y', '1', 'ok', 'continue']:
            self.get_logger().info("✓ 用户确认，开始执行...")
        else:
            self.get_logger().warn("✗ 用户否定，取消执行")
            return
        
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
        self.get_logger().warn("✓ 请等待驱动器完成标零并手动重启 proc_manager 服务或直接断电重启机器人")
        
        time.sleep(2)


def main(args=None):
    # 初始化ROS2的Python客户端库
    rclpy.init(args=args)
    
    # 创建双臂电机控制节点实例
    controller = ArmMotorController()
    
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
            # 仅右手挥手
            elif arg in ["waving", "wave", "w"]:
                mode = "waving"
            # 双腿安全位置
            elif arg in ["leg_safe", "leg", "l"]:
                mode = "leg_safe"
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
        
        # ========== 安全确认 ==========
        controller.get_logger().info("")
        controller.get_logger().warn("【安全检查】开始执行前必须确认环境安全！")
        controller.get_logger().warn("")
        controller.get_logger().warn("⚠️  请检查以下事项：")
        controller.get_logger().warn("    ✓ 机器人周围是否没有障碍物")
        controller.get_logger().warn("    ✓ 是否远离旋转部件")
        controller.get_logger().warn("    ✓ 是否有足够的工作空间")
        controller.get_logger().warn("")
        
        # 获取用户确认
        while True:
            user_input = input("请确认机器人周围环境安全，是否继续执行？(y/n)：").strip().lower()
            
            # 检查用户输入
            if user_input in ['是', 'yes', 'y', '1', 'ok', 'continue']:
                controller.get_logger().info("✓ 用户确认环境安全，开始执行...")
                break
            elif user_input in ['否', 'no', 'n', '0', 'cancel', 'stop']:
                controller.get_logger().warn("✗ 用户否定，取消执行")
                controller.get_logger().warn("程序已退出")
                controller.destroy_node()
                rclpy.shutdown()
                return
            else:
                controller.get_logger().warn(f"无效输入: '{user_input}'，请输入 'y' 或 'n'")
        
        # ========== 执行选定的模式 ==========
        if mode == "homing":
            # 仅执行回零
            controller.homing()
        elif mode == "waving":
            # 仅右手挥手
            controller.wave()
            time.sleep(3)
        elif mode == "leg_safe":
            # 双腿电机到安全位置
            controller.leg_safe()
            time.sleep(3)
        elif mode == "position":
            # 执行位置模式
            controller.homing()  # 先回零
            time.sleep(3)
            controller.position_control_mode()
            
        elif mode == "impedance":
            # 执行阻抗模式
            controller.homing()  # 先回零
            time.sleep(3)
            controller.impedance_control_mode()
            
        elif mode == "velocity":
            # 执行速度模式
            controller.homing()  # 先回零
            time.sleep(3)
            controller.velocity_control_mode()
        
        elif mode == "zero":
            # controller.homing()  # 这里只是示例，所以先回零，确保所有关节都在零位
            # time.sleep(3)
            # controller.homing()  # 这里只是示例，所以先回零，确保所有关节都在零位
            # time.sleep(3)
            # controller.homing() # 再次确保所有关节都在零位
            # 执行标零，没有标零工具就不要随意尝试此方法，很危险！
            # controller.set_zero()
            controller.get_logger().info("默认忽略标零命令，如果你确定要标零，可在理解代码和标零含义的基础上按需要取消上述几行的注释后再编译运行此脚本")
            pass
        
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

if __name__ == '__main__':
    main()
