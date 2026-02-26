#!/usr/bin/env python3
"""
ArmControlMixin

IK 解算与手臂运动控制的 Mixin 类，可被任意 rclpy.Node 子类混入。

宿主类需提供：
  - self.ik_client        (rclpy.Client for GetPositionIK '/compute_ik')
  - self.arm_pos_cmd_publisher  (Publisher for CmdSetMotorPosition '/arm/cmd_pos')
  - self.current_joint_state    (sensor_msgs.msg.JointState，由关节状态订阅回调更新)
  - self.get_logger()
  - self.get_clock()

不继承 Node，避免多继承时 MRO 冲突与重复初始化。
"""

import math
import time
import rclpy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition

# ============ 全局运动参数 ============
VELOCITY_LIMIT = 0.4   # 速度限制（弧度/秒）
CURRENT_LIMIT  = 5.0   # 电流限制（安培）

# IK 种子姿态：提供一个合理的初始关节角度，帮助 IK 找到好的解
prepare_pose = {
    "left_arm": [
        {11: 0.5, 12: 0.15, 13: 0.1, 14: -0.9,  15: 0.2,  16: 0.0, 17: 0.0},
        {11: 0.5, 12: 0.4,  13: 0.1, 14: -2.0,  15: 0.2,  16: 0.0, 17: 0.0},
    ],
    "right_arm": [
        {21: 0.5, 22: -0.15, 23: -0.1, 24: -0.9,  25: -0.2, 26: 0.0, 27: 0.0},
        {21: 0.5, 22: -0.4,  23: -0.1, 24: -2.0,  25: -0.2, 26: 0.0, 27: 0.0},
    ],
}

# 关节名称 ↔ 电机 ID 映射（左臂）
_LEFT_ARM_JOINT_MOTOR_MAP = {
    'shoulder_pitch_l_joint': '11',
    'shoulder_roll_l_joint':  '12',
    'shoulder_yaw_l_joint':   '13',
    'elbow_pitch_l_joint':    '14',
    'elbow_yaw_l_joint':      '15',
    'wrist_pitch_l_joint':    '16',
    'wrist_roll_l_joint':     '17',
}

# 关节名称 ↔ 电机 ID 映射（右臂）
_RIGHT_ARM_JOINT_MOTOR_MAP = {
    'shoulder_pitch_r_joint': '21',
    'shoulder_roll_r_joint':  '22',
    'shoulder_yaw_r_joint':   '23',
    'elbow_pitch_r_joint':    '24',
    'elbow_yaw_r_joint':      '25',
    'wrist_pitch_r_joint':    '26',
    'wrist_roll_r_joint':     '27',
}


class ArmControlMixin:
    """IK 解算与手臂运动控制功能 Mixin"""

    # ------------------------------------------------------------------
    # 内部工具方法
    # ------------------------------------------------------------------

    def _create_arm_header(self):
        """创建手臂命令消息头"""
        now = self.get_clock().now()
        header = Header()
        header.stamp.sec     = int(now.nanoseconds // 1_000_000_000)
        header.stamp.nanosec = int(now.nanoseconds  % 1_000_000_000)
        header.frame_id = 'arm'
        return header

    def _get_joint_motor_map(self, group_name: str) -> dict:
        """根据规划组名称返回对应的关节-电机映射表"""
        if 'right' in group_name:
            return _RIGHT_ARM_JOINT_MOTOR_MAP
        return _LEFT_ARM_JOINT_MOTOR_MAP

    def create_joint_state_from_motor_dict(self, motor_positions_dict: dict,
                                           group_name: str = 'left_arm') -> JointState:
        """将电机位置字典转换为 JointState 消息

        会将字典中的目标关节加入到消息中，其余关节使用当前实际关节状态填充，
        以避免 IK 求解器因缺少关节信息而失败。

        参数:
            motor_positions_dict: {电机ID(int或str): 角度(float)}
            group_name: 规划组名称，用于确定关节映射表
        """
        joint_motor_map = {
            int(k): v
            for k, v in {
                v: int(k)
                for k, v in self._get_joint_motor_map(group_name).items()
            }.items()
        }
        # 构建 motor_id(int) → joint_name 的映射
        motor_to_joint = {
            int(k): v
            for k, v in {
                v: k for k, v in self._get_joint_motor_map(group_name).items()
            }.items()
        }

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # 先填充目标组的关节
        for motor_id_int in sorted(motor_to_joint.keys()):
            joint_name = motor_to_joint[motor_id_int]
            motor_id_str = str(motor_id_int)
            if motor_id_int in motor_positions_dict:
                joint_state.name.append(joint_name)
                joint_state.position.append(motor_positions_dict[motor_id_int])
            elif motor_id_str in motor_positions_dict:
                joint_state.name.append(joint_name)
                joint_state.position.append(motor_positions_dict[motor_id_str])

        # 其余关节使用当前实际状态补全
        if self.current_joint_state is not None:
            for i, name in enumerate(self.current_joint_state.name):
                if name not in joint_state.name:
                    joint_state.name.append(name)
                    joint_state.position.append(self.current_joint_state.position[i])

        return joint_state

    # ------------------------------------------------------------------

    def call_ik_service_with_params(self, group_name: str, frame_id: str,
                                    position, orientation):
        """调用 MoveIt /compute_ik 服务进行 IK 解算

        参数:
            group_name:  MoveIt 规划组名称（如 'left_arm'）
            frame_id:    目标位姿的参考坐标系（如 'pelvis'）
            position:    geometry_msgs.msg.Point
            orientation: geometry_msgs.msg.Quaternion

        返回: GetPositionIK.Response 或 None（失败时）
        """
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name

        # 归一化四元数
        q = orientation
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        normalized_quat = Quaternion(
            x=q.x / norm, y=q.y / norm, z=q.z / norm, w=q.w / norm
        )

        request.ik_request.pose_stamped.header.frame_id = frame_id
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose.position    = position
        request.ik_request.pose_stamped.pose.orientation = normalized_quat

        self.get_logger().info(f'=== IK 目标位姿 ===')
        self.get_logger().info(f'  group_name: {group_name}')
        self.get_logger().info(f'  frame_id: {frame_id}')
        self.get_logger().info(
            f'  position: [{position.x:.6f}, {position.y:.6f}, {position.z:.6f}]'
        )
        self.get_logger().info(
            f'  orientation: [{normalized_quat.x:.6f}, {normalized_quat.y:.6f}, '
            f'{normalized_quat.z:.6f}, {normalized_quat.w:.6f}]'
        )

        # 选择 IK 种子
        if 'right' in group_name:
            seed_pose = prepare_pose['right_arm'][1]
        else:
            seed_pose = prepare_pose['left_arm'][1]

        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = \
            self.create_joint_state_from_motor_dict(seed_pose, group_name)
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 5

        future = self.ik_client.call_async(request)

        # 主线程已在 rclpy.spin()，本方法可能在后台线程调用。
        # 不能再次 spin_until_future_complete（会报 RuntimeError: Node already spinning）。
        # 只需轮询 future.done()，主线程的 spin 会自动处理回调并完成 future。
        deadline = time.time() + 12.0  # IK 超时上限： request.timeout.sec(5) + 网络传输余量
        while not future.done():
            if time.time() > deadline:
                self.get_logger().error('IK 服务调用超时（12s）')
                return None
            time.sleep(0.05)

        if future.result() is not None:
            return future.result()

        self.get_logger().error('IK 服务调用失败')
        return None

    def extract_joint_positions(self, response,
                                group_name: str = 'left_arm') -> dict | None:
        """从 IK 响应中提取目标规划组的关节角度

        返回: {电机ID字符串: 角度} 字典（按电机ID升序），或 None（失败时）
        """
        joint_motor_map = self._get_joint_motor_map(group_name)
        result_dict = {}

        self.get_logger().info(f'IK 错误码: {response.error_code.val}')

        if response.error_code.val == 1:  # SUCCESS
            joint_names     = response.solution.joint_state.name
            joint_positions = response.solution.joint_state.position

            self.get_logger().info(f'=== 提取 {group_name} 关节角度 ===')
            for i, joint_name in enumerate(joint_names):
                if joint_name in joint_motor_map:
                    motor_id = joint_motor_map[joint_name]
                    result_dict[motor_id] = round(joint_positions[i], 5)
                    self.get_logger().info(
                        f'  [{i}] {joint_name} → 电机{motor_id}: {joint_positions[i]:.5f}'
                    )

            expected = len(joint_motor_map)
            if len(result_dict) != expected:
                self.get_logger().error(
                    f'⚠ 只找到 {len(result_dict)} 个关节，应为 {expected} 个！'
                )

            return dict(sorted(result_dict.items(), key=lambda x: int(x[0])))

        self.get_logger().error(f'IK 求解失败，错误码: {response.error_code.val}')
        return None

    def arm_to_pose(self, pose: dict, spd: float = VELOCITY_LIMIT):
        """将手臂移动到指定关节角度

        参数:
            pose: {电机ID(int或str): 目标角度(float)}
            spd:  运动速度（弧度/秒），默认使用 VELOCITY_LIMIT
        """
        header = self._create_arm_header()
        msg = CmdSetMotorPosition()
        msg.header = header

        for k, v in pose.items():
            cmd = SetMotorPosition()
            cmd.name = int(k)
            cmd.pos  = float(v)
            cmd.spd  = spd
            cmd.cur  = CURRENT_LIMIT
            msg.cmds.append(cmd)
            self.get_logger().info(f'  电机 {k}：运动到目标位置（{v:.4f} rad）')

        self.arm_pos_cmd_publisher.publish(msg)
        self.get_logger().info('✓ 手臂运动命令已发送')

        # 等待手臂运动完成（IK 最多 5s + 运动 6s，取 7s 作为安全余量）
        time.sleep(7.0)

    def arm_pose_init(self) -> bool:
        """引导手臂逐段进入抓取准备姿态，每段均需用户确认再执行。

        返回: True 表示全部阶段都成功执行，False 表示用户中止。
        """
        left_arm_poses  = prepare_pose['left_arm']
        right_arm_poses = prepare_pose['right_arm']
        for idx, left_pose in enumerate(left_arm_poses):
            merged_pose = {**left_pose, **right_arm_poses[idx]}
            user_input = input(
                f'\n[{idx + 1}/{len(left_arm_poses)}] '
                f'是否移动手臂到第 {idx + 1} 个过渡姿态？'
                f'  (yes/y 确认, no/n 中止): '
            ).strip().lower()
            if user_input in ('yes', 'y'):
                self.arm_to_pose(merged_pose)
            else:
                print('✓ 手臂过渡已中止')
                return False
        return True
            
    
    def arm_pose_reverse_init(self) -> bool:
        """引导手臂逐段退回安全姿态，每段均需用户确认再执行。

        返回: True 表示全部阶段都成功执行，False 表示用户中止。
        """
        left_arm_revposes  = list(reversed(prepare_pose['left_arm']))
        right_arm_revposes = list(reversed(prepare_pose['right_arm']))
        for idx, left_pose in enumerate(left_arm_revposes):
            merged_pose = {**left_pose, **right_arm_revposes[idx]}
            user_input = input(
                f'\n[{idx + 1}/{len(left_arm_revposes)}] '
                f'是否移动手臂到第 {idx + 1} 个中止姿态？'
                f'  (yes/y 确认, no/n 中止): '
            ).strip().lower()
            if user_input in ('yes', 'y'):
                self.arm_to_pose(merged_pose)
            else:
                print('✓ 手臂移动已中止')
                return False
            
        return True