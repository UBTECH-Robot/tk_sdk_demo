#!/usr/bin/env python3
"""
ArmControlMixin

IK 解算与手臂运动控制的 Mixin 类，可被任意 rclpy.Node 子类混入。

宿主类需提供：
  - self.ik_client        (rclpy.Client for GetPositionIK '/compute_ik')
  - self.get_logger()     (由 Node 提供)
  - self.get_clock()      (由 Node 提供)

本 Mixin 自身会在 __init__ 中初始化：
  - self.arm_pos_cmd_publisher  (Publisher for CmdSetMotorPosition '/arm/cmd_pos')
  - self.joint_command_pub      (Publisher for String '/gui/joint_command')
  - self.current_joint_state    (由 /joint_states 订阅回调持续更新)

不继承 Node，避免多继承时 MRO 冲突与重复初始化。
"""

import copy
import json
import math
import threading
import time
import rclpy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from std_msgs.msg import Header, String
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition

# ============ 全局运动参数 ============
VELOCITY_LIMIT = 0.4   # 速度限制（弧度/秒）
CURRENT_LIMIT  = 5.0   # 电流限制（安培）
POSE_MATCH_ABS_TOL = math.radians(1.0)  # 姿态匹配绝对误差阈值（约 1°）

prepare_pose = {
    "left_arm": [
        {11: 0.5, 12: 0.15, 13: 0.3, 14: -0.9, 15: 0.2, 16: 0.0, 17: 0.0},
        {11: 0.5, 12: 0.4, 13: 0.6, 14: -2.5, 15: 0.2, 16: 0.0, 17: 0.0},
    ],
    "right_arm": [
        {21: 0.5, 22: -0.15, 23: -0.3, 24: -0.9, 25: -0.2, 26: 0.0, 27: 0.0},
        {21: 0.5, 22: -0.4, 23: -0.6, 24: -2.5, 25: -0.2, 26: 0.0, 27: 0.0},
    ],
}

end_pose_sequence = {
    "left_arm": [
        {11: 0.5, 12: 0.4, 13: 0.3, 14: -2.0, 15: 0.2, 16: 0.0, 17: 0.0},
        {11: 1.2, 12: 0.4, 13: 0.2, 14: -2.5, 15: 0.2, 16: 0.0, 17: 0.0},
        {11: 1.2, 12: 0.4, 13: 0.3, 14: -1.7, 15: 0.2, 16: 0.0, 17: 0.0},
        {11: 0.5, 12: 0.15, 13: 0.3, 14: -0.9, 15: 0.2, 16: 0.0, 17: 0.0},
    ],
    "right_arm": [
        {21: 0.5, 22: -0.4, 23: -0.3, 24: -2.0, 25: -0.2, 26: 0.0, 27: 0.0},
        {21: 1.2, 22: -0.4, 23: -0.2, 24: -2.5, 25: -0.2, 26: 0.0, 27: 0.0},
        {21: 1.2, 22: -0.4, 23: -0.3, 24: -1.7, 25: -0.2, 26: 0.0, 27: 0.0},
        {21: 0.5, 22: -0.15, 23: -0.3, 24: -0.9, 25: -0.2, 26: 0.0, 27: 0.0},
    ],
}

# MoveIt 响应码 → 说明映射（来自 moveit_msgs/MoveItErrorCodes）
MOVEIT_ERROR_CODES = {
     1: 'SUCCESS（成功）',
    -1: 'FAILURE（通用失败）',
    -2: 'PLANNING_FAILED（规划失败）',
    -3: 'INVALID_MOTION_PLAN（无效运动规划）',
    -4: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE（环境变化导致规划失效）',
    -5: 'CONTROL_FAILED（控制失败）',
    -6: 'UNABLE_TO_AQUIRE_SENSOR_DATA（无法获取传感器数据）',
    -7: 'TIMED_OUT（超时）',
    -8: 'PREEMPTED（被抢占/中断）',
    -9: 'START_STATE_IN_COLLISION（起始状态碰撞）',
   -10: 'START_STATE_VIOLATES_PATH_CONSTRAINTS（起始状态违反路径约束）',
   -11: 'GOAL_IN_COLLISION（目标位置碰撞）',
   -12: 'GOAL_VIOLATES_PATH_CONSTRAINTS（目标违反路径约束）',
   -13: 'GOAL_CONSTRAINTS_VIOLATED（目标约束违反）',
   -14: 'INVALID_GROUP_NAME（无效规划组名称）',
   -15: 'INVALID_GOAL_CONSTRAINTS（无效目标约束）',
   -16: 'INVALID_ROBOT_STATE（无效机器人状态）',
   -17: 'INVALID_LINK_NAME（无效链接名称）',
   -18: 'INVALID_OBJECT_NAME（无效物体名称）',
   -19: 'FRAME_TRANSFORM_FAILURE（坐标系变换失败）',
   -20: 'COLLISION_CHECKING_UNAVAILABLE（碰撞检测不可用）',
   -21: 'ROBOT_STATE_STALE（机器人状态过期）',
   -22: 'SENSOR_INFO_STALE（传感器信息过期）',
   -23: 'COMMUNICATION_FAILURE（通信失败）',
   -24: 'CRASH（崩溃）',
   -25: 'ABORT（中止）',
   -26: 'CHILD_PROCESS_FAILED（子进程失败）',
   -31: 'NO_IK_SOLUTION（目标位姿超出可达范围，无 IK 解）',
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

    def __init__(self, *args, **kwargs):
        # 通过 super() 沿 MRO 链传递参数，确保 Node.__init__ 被正确调用
        super().__init__(*args, **kwargs)
        self.prepare_pose = prepare_pose
        self.end_pose_sequence = end_pose_sequence
        self._init_arm_control()

    # ------------------------------------------------------------------
    # 初始化
    # ------------------------------------------------------------------

    def _init_arm_control(self):
        """初始化手臂控制发布者和关节状态订阅"""
        self.arm_pos_cmd_publisher = self.create_publisher(
            CmdSetMotorPosition, '/arm/cmd_pos', 10
        )
        self.get_logger().info('✓ 手臂控制发布者已创建（/arm/cmd_pos）')

        # 发布 IK 解算结果 JSON，供 GUI 可视化
        self.joint_command_pub = self.create_publisher(String, '/gui/joint_command', 10)
        self.get_logger().info('✓ GUI 关节命令发布者已创建（/gui/joint_command）')

        self.current_joint_state = None
        self._joint_state_lock = threading.Lock()
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10
        )
        self.get_logger().info('✓ 已订阅 /joint_states')

    def _joint_states_cb(self, msg: JointState):
        """关节状态订阅回调，持续更新 current_joint_state"""
        with self._joint_state_lock:
            self.current_joint_state = msg

    # ------------------------------------------------------------------
    # 内部工具方法
    # ------------------------------------------------------------------

    def _create_header(self, frame_id="arm"):
        """创建手臂命令消息头"""
        now = self.get_clock().now()
        header = Header()
        header.stamp.sec     = int(now.nanoseconds // 1_000_000_000)
        header.stamp.nanosec = int(now.nanoseconds  % 1_000_000_000)
        header.frame_id = frame_id
        return header

    def _get_joint_motor_map(self, group_name: str) -> dict:
        """根据规划组名称返回对应的关节-电机映射表"""
        if 'right' in group_name:
            return _RIGHT_ARM_JOINT_MOTOR_MAP
        return _LEFT_ARM_JOINT_MOTOR_MAP

    def _get_seed_joint_state(self, group_name: str) -> JointState:
        """获取 IK 种子关节状态（加锁读取当前实际状态）

        优先使用当前实际关节状态作为 IK 种子，包含全部关节（左臂、右臂均包含其中）。
        若 current_joint_state 尚未就绪，返回空 JointState 并记录警告。

        参数:
            group_name: MoveIt 规划组名称（仅用于日志区分左右臂）
        返回: JointState
        """
        with self._joint_state_lock:
            current = copy.copy(self.current_joint_state)  # 浅拷贝，避免长期持锁

        if current is not None:
            seed = JointState()
            seed.header.stamp = self.get_clock().now().to_msg()
            # 传递全部关节状态作为种子，规划组以外的关节数据也包含在内，以在avoid_collisions为True时参与计算，避免碰撞
            seed.name     = list(current.name)
            seed.position = list(current.position)
            return seed

        self.get_logger().warn(
            f'[{group_name}] current_joint_state 尚未就绪，使用空种子姿态'
        )
        return JointState()

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
            f'  position: (x={position.x:.6f}, y={position.y:.6f}, z={position.z:.6f})'
        )
        self.get_logger().info(
            f'  orientation: (x={normalized_quat.x:.6f}, y={normalized_quat.y:.6f}, '
            f'z={normalized_quat.z:.6f}, w={normalized_quat.w:.6f})'
        )

        # 使用当前实际关节状态作为 IK 种子（包含全部关节，左右臂均在其中）
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = self._get_seed_joint_state(group_name)
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

        ik_code = response.error_code.val
        ik_desc = MOVEIT_ERROR_CODES.get(ik_code, f'未知响应码 {ik_code}')
        self.get_logger().info(f'IK 响应码: {ik_code}  →  {ik_desc}')

        if response.error_code.val == 1:  # SUCCESS
            joint_names     = response.solution.joint_state.name
            joint_positions = response.solution.joint_state.position

            self.get_logger().info(f'=== 提取 {group_name} 关节角度 ===')
            for i, joint_name in enumerate(joint_names):
                if joint_name in joint_motor_map:
                    motor_id = joint_motor_map[joint_name]
                    result_dict[motor_id] = round(joint_positions[i], 5)
                    # self.get_logger().info(
                    #     f'  [{i}] {joint_name} → 电机{motor_id}: {joint_positions[i]:.5f}'
                    # )

            expected = len(joint_motor_map)
            if len(result_dict) != expected:
                self.get_logger().error(
                    f'⚠ 只找到 {len(result_dict)} 个关节，应为 {expected} 个！'
                )
            self.get_logger().info(json.dumps(result_dict))
            return dict(sorted(result_dict.items(), key=lambda x: int(x[0])))

        return None

    def is_pose_match_current_pose(self, target_pose: dict, log_mismatch: bool = True) -> bool:
        """判断目标电机姿态是否与当前关节状态一致。

        参数:
            target_pose: {电机ID(int或str): 目标角度(float)}

        返回:
            True: target_pose 中所有关节都与 current_joint_state 对应关节一致
            False: 任一关节不一致 / 关节缺失 / current_joint_state 未就绪
        """
        while not self.current_joint_state:
            time.sleep(0.5)
            self.get_logger().info('等待 current_joint_state 就绪')

        with self._joint_state_lock:
            current = copy.copy(self.current_joint_state)

        # 电机ID(str) -> 关节名
        motor_to_joint_map = {
            **{motor_id: joint_name for joint_name, motor_id in _LEFT_ARM_JOINT_MOTOR_MAP.items()},
            **{motor_id: joint_name for joint_name, motor_id in _RIGHT_ARM_JOINT_MOTOR_MAP.items()},
        }

        # 当前关节名 -> 当前角度
        current_joint_pos = {
            joint_name: joint_pos
            for joint_name, joint_pos in zip(current.name, current.position)
        }

        for motor_id, target_value in target_pose.items():
            motor_id_str = str(motor_id)
            joint_name = motor_to_joint_map.get(motor_id_str)
            if joint_name is None:
                self.get_logger().warn(f'未知电机ID: {motor_id_str}，无法比较姿态')
                return False

            if joint_name not in current_joint_pos:
                self.get_logger().warn(f'当前关节状态缺少关节: {joint_name}')
                return False

            current_value = current_joint_pos[joint_name]
            if not math.isclose(float(target_value), float(current_value), abs_tol=POSE_MATCH_ABS_TOL):
                if log_mismatch:
                    self.get_logger().info(
                        f'关节 {joint_name}（电机{motor_id_str}）姿态不匹配：'
                        f'目标 {target_value:.5f} rad vs 当前 {current_value:.5f} rad'
                    )
                return False

        return True

    def arm_to_pose(
        self,
        pose: dict,
        spd: float = VELOCITY_LIMIT,
        publish_ghost: bool = True,
        require_confirm: bool = True,
        confirm_prompt: str = '是否移动手臂到目标位姿？',
    ) -> bool:
        """将手臂移动到指定关节角度

        参数:
            pose: {电机ID(int或str): 目标角度(float)}
            spd:  运动速度（弧度/秒），默认使用 VELOCITY_LIMIT
            publish_ghost: 是否在发送运动命令前自动发布一次 GUI 预览
            require_confirm: 是否在发送运动前进行 yes/no 交互确认
            confirm_prompt: 用户确认提示语

        返回:
            True: 已执行运动命令
            False: 用户取消运动
        """
        if publish_ghost:
            self._publish_model_ghost(pose)

        if require_confirm:
            while True:
                user_input = input(
                    f'\n{confirm_prompt}\n'
                    '  输入 yes/y 执行\n'
                    '  输入 no/n  放弃\n'
                    '请选择: '
                ).strip().lower()
                if user_input in ('yes', 'y'):
                    break
                if user_input in ('no', 'n'):
                    return False
                print('✗ 无效输入，请输入 yes/y 或 no/n')

        header = self._create_header()
        msg = CmdSetMotorPosition()
        msg.header = header

        for k, v in pose.items():
            cmd = SetMotorPosition()
            cmd.name = int(k)
            cmd.pos  = float(v)
            cmd.spd  = spd
            cmd.cur  = CURRENT_LIMIT
            msg.cmds.append(cmd)
            # self.get_logger().info(f'  电机 {k}：运动到目标位置（{v:.4f} rad）')

        self.arm_pos_cmd_publisher.publish(msg)
        self.get_logger().info('✓ 手臂运动命令已发送')

        # 最多等待 7s；若提前到位则提前返回
        deadline = time.time() + 7.0
        while True:
            if self.is_pose_match_current_pose(pose, log_mismatch=False):
                self.get_logger().info('✓ 手臂已到达目标位姿')
                return True

            if time.time() >= deadline:
                self.get_logger().warn('等待手臂到达目标位姿超时（7s），继续后续流程')
                return True

            time.sleep(0.5)
        return True

    def _publish_model_ghost(self, joint_positions: dict):
        """将关节角度以 JSON 格式发布到 /gui/joint_command，供 GUI 可视化

        参数:
            joint_positions: {电机ID(int或str): 角度(float)}
        """
        json_str = json.dumps(joint_positions)
        msg      = String()
        msg.data = json_str
        self.joint_command_pub.publish(msg)

    def arm_pose_init(self) -> bool:
        """引导手臂逐段进入抓取准备姿态，每段均需用户确认再执行。

        返回: True 表示全部阶段都成功执行，False 表示用户中止。
        """
        left_arm_poses  = prepare_pose['left_arm']
        right_arm_poses = prepare_pose['right_arm']
        for idx, left_pose in enumerate(left_arm_poses):
            merged_pose = {**left_pose, **right_arm_poses[idx]}
            ok = self.arm_to_pose(
                merged_pose,
                publish_ghost=True,
                require_confirm=True,
                confirm_prompt=(
                    f'[{idx + 1}/{len(left_arm_poses)}] '
                    f'是否移动手臂到第 {idx + 1} 个过渡姿态？'
                ),
            )
            if not ok:
                print('✓ 手臂过渡已中止')
                return False
        return True
    
    def arm_pose_reverse_init(self) -> bool:
        """引导手臂逐段退回安全姿态，每段均需用户确认再执行。

        返回: True 表示全部阶段都成功执行，False 表示用户中止。
        """
        left_arm_end_pose  = end_pose_sequence['left_arm']
        right_arm_end_pose = end_pose_sequence['right_arm']
        for idx, left_pose in enumerate(left_arm_end_pose):
            merged_pose = {**left_pose, **right_arm_end_pose[idx]}
            ok = self.arm_to_pose(
                merged_pose,
                publish_ghost=True,
                require_confirm=True,
                confirm_prompt=(
                    f'[{idx + 1}/{len(left_arm_end_pose)}] '
                    f'是否移动手臂到第 {idx + 1} 个中止姿态？'
                ),
            )
            if not ok:
                print('✓ 手臂移动已中止')
                return False
            
        return True