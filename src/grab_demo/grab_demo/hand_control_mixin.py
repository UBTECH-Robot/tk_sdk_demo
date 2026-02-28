#!/usr/bin/env python3
import time
import rclpy
from sensor_msgs.msg import JointState
from bodyctrl_msgs.srv import SetForce, SetSpeed, SetClearError

# 手指关节 ID → 可读名称映射
JOINT_NAME_MAP = {
    "1": "小指",
    "2": "无名指",
    "3": "中指",
    "4": "食指",
    "5": "拇指弯曲",
    "6": "拇指旋转",
}

class HandControlMixin:
    def __init__(self, *args, **kwargs):
        # 通过 super() 沿 MRO 链传递参数，确保 Node.__init__ 被正确调用
        super().__init__(*args, **kwargs)
        self._init_hand_control()
        self.JOINT_NAME_MAP = JOINT_NAME_MAP
        self.JOINT_ID_POS_FOR_OPEN = {"1": 1.0, "2": 1.0, "3": 1.0, "4": 1.0, "5": 1.0, "6": 0.5}  # 张开的位置，保持拇指旋转关节略微弯曲以增加适应性
        self.JOINT_ID_POS_FOR_CLOSE = {"1": 0.3, "2": 0.3, "3": 0.3, "4": 0.3, "5": 0.3, "6": 0.3}  # 闭合位置

    # ------------------------------------------------------------------
    # 初始化
    # ------------------------------------------------------------------
    def _init_hand_control(self):
        """初始化手部控制发布者和关节状态订阅"""
        # 保存控制参数
        self.tor_value = 0.1
        self.spd_value = 0.1

        # 位置话题发布器初始化
        self.left_hand_publisher = self.create_publisher(JointState, "/inspire_hand/ctrl/left_hand", 10)
        self.right_hand_publisher = self.create_publisher(JointState, "/inspire_hand/ctrl/right_hand", 10)

        # 手指关节力矩服务和手指关节速度服务客户端初始化
        self.left_hand_force_client = self.create_client(SetForce, "/inspire_hand/set_force/left_hand")
        self.right_hand_force_client = self.create_client(SetForce, "/inspire_hand/set_force/right_hand")
        self.left_hand_speed_client = self.create_client(SetSpeed, "/inspire_hand/set_speed/left_hand")
        self.right_hand_speed_client = self.create_client(SetSpeed, "/inspire_hand/set_speed/right_hand")
        
        self.get_logger().info('✓ 手部控制发布者已创建（/hand/cmd_pos）')

    # ------------------------------------------------------------------
    # 内部工具方法
    # ------------------------------------------------------------------
    def _execute_position_control(self, joint_ids: dict):
        """
        位置控制模式执行函数
        
        发布单个关节的位置控制指令，使关节运动到目标位置。
        
        参数：
            joint_ids (dict): 关节ID与目标位置的映射（字典形式，键为关节ID，值为目标位置）
        """        
        # 构建位置控制消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
    
        # 遍历所有关节，依次执行控制
        for joint_id, target_pos in joint_ids.items():
            joint_name = self.JOINT_NAME_MAP.get(joint_id, f"未知关节{joint_id}")
            msg.name.append(joint_id)
            msg.position.append(target_pos)

        # 发布到左右手控制话题
        self.left_hand_publisher.publish(msg)
        self.right_hand_publisher.publish(msg)

        # 输出控制日志
        joint_names = [self.JOINT_NAME_MAP.get(jid, f"未知关节{jid}") for jid in joint_ids]
        self.get_logger().info(
            f"[位置控制] 关节：{', '.join(joint_names)} (IDs: {', '.join(joint_ids)})，"
            f"目标位置：{', '.join(str(pos) for pos in joint_ids.values())}，"
            f"发布消息：{msg}"
        )

    def set_force(self, force_value : float):
        """
        设置手指各个关节的旋转过程中会施加的最大力矩
        
        通过 SetForce 服务发送力矩控制命令，该服务可同时控制所有6个手指关节的力矩。
                
        参数：
            force_value: 力矩比例值列表（浮点数，范围0.0~1.0，0代表0g，1代表1000g）
        """
        # 创建 SetForce 服务请求
        request = SetForce.Request()
        
        # 填充所有力反馈比例参数（这里简单将所有关节的力矩比例设置为相同值，实际应用中可根据具体场景调整）
        request.force0_ratio = force_value
        request.force1_ratio = force_value
        request.force2_ratio = force_value
        request.force3_ratio = force_value
        request.force4_ratio = force_value
        request.force5_ratio = force_value

        # 发送请求到左右手力矩控制服务
        try:
            # 左手服务调用
            future_left = self.left_hand_force_client.call_async(request)
            
            # 右手服务调用
            future_right = self.right_hand_force_client.call_async(request)

            # 轮询等待（不能用 spin_until_future_complete，节点已在 rclpy.spin() 中）
            deadline = time.time() + 2.0
            while (not future_left.done() or not future_right.done()) and time.time() < deadline:
                time.sleep(0.05)

            # 获取并打印服务响应
            if future_left.done():
                response_left = future_left.result()
                self.get_logger().info(f"  左手响应 - force_accepted: {response_left.force_accepted}")
            
            if future_right.done():
                response_right = future_right.result()
                self.get_logger().info(f"  右手响应 - force_accepted: {response_right.force_accepted}")
            
            # 输出控制日志
            self.get_logger().info(
                f"[力矩控制] 目标力矩比例：{force_value}，"
                f"服务调用参数：{request}，"
            )
        except Exception as e:
            self.get_logger().error(f"力矩控制服务调用失败: {e}")

    def set_speed(self, speed_value : float):
        """
        设置手指各个关节的转动速度
        
        通过 SetSpeed 服务发送速度控制命令，该服务可同时控制所有6个手指关节的速度。
                
        参数：
            speed_value: 速度比例值列表（浮点数，范围0.0~1.0，1代表800ms从最大角度到最小角度，0.5代表1600ms，0.25代表3200ms）
        """
        # 创建 SetSpeed 服务请求
        request = SetSpeed.Request()
        
        # 填充所有速度比例参数（这里简单将所有关节的速度比例设置为相同值，实际应用中可根据具体场景调整）
        request.speed0_ratio = speed_value
        request.speed1_ratio = speed_value
        request.speed2_ratio = speed_value
        request.speed3_ratio = speed_value
        request.speed4_ratio = speed_value
        request.speed5_ratio = speed_value

        # 发送请求到左右手速度控制服务
        try:
            # 左手服务调用 和 右手服务调用
            future_left = self.left_hand_speed_client.call_async(request)
            future_right = self.right_hand_speed_client.call_async(request)

            # 轮询等待（不能用 spin_until_future_complete，节点已在 rclpy.spin() 中）
            deadline = time.time() + 2.0
            while (not future_left.done() or not future_right.done()) and time.time() < deadline:
                time.sleep(0.05)

            # 获取并打印服务响应
            if future_left.done():
                response_left = future_left.result()
                self.get_logger().info(f"  左手响应 - speed_accepted: {response_left.speed_accepted}")
            
            if future_right.done():
                response_right = future_right.result()
                self.get_logger().info(f"  右手响应 - speed_accepted: {response_right.speed_accepted}")
            
            # 输出控制日志
            self.get_logger().info(
                f"[速度控制] 目标速度比例：{speed_value}，"
                f"服务调用参数：{request}，"
            )
        except Exception as e:
            self.get_logger().error(f"速度控制服务调用失败: {e}")

    def hand_open(self):
        """执行手部张开动作"""
        self.set_force(self.tor_value)
        time.sleep(0.5)
        self.set_speed(self.spd_value)
        time.sleep(0.5)
        self._execute_position_control(self.JOINT_ID_POS_FOR_OPEN)

    def hand_close(self):
        """执行手部闭合动作"""        
        self.set_force(self.tor_value)
        time.sleep(0.5)
        self.set_speed(self.spd_value)
        time.sleep(0.5)
        self._execute_position_control(self.JOINT_ID_POS_FOR_CLOSE)