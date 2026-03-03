#!/usr/bin/env python3
"""
GraspExecutorNode

抓取执行节点。订阅 /grasp_candidate 话题，接收来自感知节点的抓取候选点，
按以下状态机驱动完整的抓取-放置流程：

    IDLE → MOVING → VERIFYING → PLACING → IDLE

用户交互（input()）运行在独立后台线程，不阻塞 rclpy.spin。

ros2 run grab_demo grasp_executor_node
"""

import threading
import time
import traceback
from enum import Enum, auto
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from bodyctrl_msgs.msg import CmdSetMotorPosition
from tf2_ros import Buffer, TransformListener
from grab_demo_msgs.msg import GraspCandidate
from grab_demo.arm_control_mixin import ArmControlMixin, VELOCITY_LIMIT
from grab_demo.hand_control_mixin import HandControlMixin
from grab_demo.pose_verification_mixin import PoseVerificationMixin

place_locations = {
    '1': {'id': '1', 'name': '左前方',  'description': '当前抓取手臂的左侧前方', 'pose': {
        'left_arm': {11: -0.1, 12: 0.65, 13: 0.3, 14: -1.28, 15: 0.2, 16: 0.0, 17: 0.0},
        'right_arm': {21: -0.8, 22: -0.28, 23: 0.63, 24: -1.28, 25: -0.2, 26: 0.0, 27: 0.0},
    }},
    '2': {'id': '2', 'name': '右前方',  'description': '当前抓取手臂的右侧前方', 'pose': {
        'left_arm': {11: -0.80, 12: 0.28, 13: -0.63, 14: -1.28, 15: 0.2, 16: 0.0, 17: 0.0},
        'right_arm': {21: -0.1, 22: -0.65, 23: -0.3, 24: -1.28, 25: -0.2, 26: 0.0, 27: 0.0},
    }},
}

safe_pose_after_grasp = {
    "left_arm": {11: 0.0, 12: 0.3, 13: 0.4, 14: -2.3},
    "right_arm": {21: 0.0, 22: -0.3, 23: -0.4, 24: -2.3},
}

offset_for_place = {
    "left_arm": {"x": -0.02, "y": 0.1, "z": 0.0},
    "right_arm": {"x": -0.02, "y": -0.1, "z": 0.0},      
}

# ============ 状态机 ============
class GraspState(Enum):
    IDLE       = auto()   # 等待候选点
    MOVING     = auto()   # IK 解算 + 手臂运动中
    VERIFYING  = auto()   # 验证末端位置
    PLACING    = auto()   # 等待用户选择放置位置 + 放置中


class GraspExecutorNode(ArmControlMixin, HandControlMixin, PoseVerificationMixin, Node):
    """抓取执行节点：IK 解算 + 手臂控制 + 用户确认 + 末端验证"""

    def __init__(self):
        super().__init__('grasp_executor')

        self._init_state()
        self._init_ik_client()
        self._init_tf()
        self._init_grasp_candidate_sub()
        self.group_name = None

        self.get_logger().info('=== GraspExecutorNode 已启动，等待 /grasp_candidate ===')

    # ------------------------------------------------------------------
    # 初始化
    # ------------------------------------------------------------------

    def _init_state(self):
        """初始化状态机和线程锁"""
        self.state      = GraspState.IDLE
        self.state_lock = threading.Lock()

    def _init_ik_client(self):
        """初始化 MoveIt IK 服务客户端，等待服务就绪"""
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info('等待 /compute_ik 服务...')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  /compute_ik 未就绪，继续等待...')
        self.get_logger().info('✓ /compute_ik 服务已就绪')

    def _init_tf(self):
        """初始化 TF2 监听器（用于末端位置验证）"""
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _init_grasp_candidate_sub(self):
        """订阅抓取候选点话题"""
        self.grasp_candidate_sub = self.create_subscription(
            GraspCandidate, '/grasp_candidate', self._grasp_candidate_cb, 10
        )
        self.get_logger().info('✓ 已订阅 /grasp_candidate')

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------

    def _grasp_candidate_cb(self, msg: GraspCandidate):
        """收到抓取候选点时触发

        仅在 IDLE 状态下处理，其余状态直接忽略，避免重入。
        用户交互和运动控制在后台线程执行，不阻塞 spin。
        """
        with self.state_lock:
            if self.state != GraspState.IDLE:
                return
            self.state = GraspState.MOVING

        self.get_logger().info(
            f'收到抓取候选点 | 物体: {msg.object_class} '
            f'| 置信度: {msg.confidence:.2f} | 组: {msg.group_name}'
        )

        t = threading.Thread(
            target=self._run_grasp_sequence,
            args=(msg,),
            daemon=False
        )
        t.start()

    # ------------------------------------------------------------------
    # 状态机主流程（在后台线程执行）
    # ------------------------------------------------------------------

    def _run_grasp_sequence(self, candidate: GraspCandidate):
        """完整抓取-放置流程（后台线程）"""
        try:
            # if candidate.group_name != 'left_arm':
            #     self.get_logger().error(f'目前只支持左手抓取机器人左前方对应区域的物体')
            #     self._wait_for_user_continue()
            #     return
            # ── 阶段 1：打印候选点信息 ────────────────────────────────
            self._print_candidate_info(candidate)

            # ── 阶段 2：手臂运动到抓取准备姿态 ───────────────────────
            with self.state_lock:
                self.state = GraspState.MOVING

            pose_ready = False
            if self.is_angle_match_current_angle(self.prepare_pose['left_arm'][1]) and self.is_angle_match_current_angle(self.prepare_pose['right_arm'][1]):
                self.get_logger().info('✓ 手臂已在准备姿态，无需移动')
            else:
                ee_link = 'R_base_link' if 'right' in candidate.group_name else 'L_base_link'
                transform, actual_pos, actual_quat = self.get_current_end_effector_transform(ee_link=ee_link)
                if transform is not None and actual_pos is not None and actual_pos.z > 0.1:
                    self.get_logger().info(f'当前末端位置: x={actual_pos.x:.4f}, y={actual_pos.y:.4f}, z={actual_pos.z:.4f}')
                    # 如果末端坐标的z轴大于10cm，则认为当前末端位置可以直接移动到准备姿态的最终位置，不会再有障碍物碰撞风险，当然移动之前也会让用户确认
                    pose_ready = self.arm_to_prepare_end_angle()
                else:
                    self.get_logger().warn('无法获取当前末端位置或者末端位置过低，需要完整执行准备姿态运动')
                    pose_ready = self.arm_angle_init()

            if not pose_ready:
                print('手臂未到达准备姿态，取消本次抓取。')
                self._wait_for_user_continue()
                return
            
            self.hand_open(candidate.group_name)  # 确保手指张开

            # ── 阶段 3：预抓取位姿 IK 解算（抬高 Z）───────────────────
            offset = offset_for_place[candidate.group_name]
            pre_grasp_joint_positions = self._solve_ik(candidate, x_offset=offset['x'], y_offset=offset['y'], z_offset=offset['z'])
            if pre_grasp_joint_positions is None:
                self.get_logger().error('预抓取位姿 IK 解算失败，取消本次抓取')
                self._wait_for_user_continue()
                return

            # ── 阶段 4：移动到预抓取位姿（含预览+确认） ─────────────
            if not self.arm_to_angle(
                pre_grasp_joint_positions,
                spd=VELOCITY_LIMIT / 2,
                publish_ghost=True,
                require_confirm=True,
                confirm_prompt='是否先移动到预抓取位姿？',
            ):
                print('用户已放弃本次抓取。')
                self._wait_for_user_continue()
                return

            with self.state_lock:
                self.state = GraspState.MOVING

            # ── 阶段 5：最终抓取位姿 IK 解算 ─────────────────────────
            final_joint_positions = self._solve_ik(candidate)
            if final_joint_positions is None:
                self.get_logger().error('最终抓取位姿 IK 解算失败，取消本次抓取')
                self._wait_for_user_continue()
                return

            # ── 阶段 6：执行最终下探运动（含预览+确认） ──────────────
            if not self.arm_to_angle(
                final_joint_positions,
                spd=VELOCITY_LIMIT / 2,
                publish_ghost=True,
                require_confirm=True,
                confirm_prompt='是否从预抓取位姿移动到抓取位姿？',
            ):
                print('用户已放弃本次抓取。')
                self._wait_for_user_continue()
                return

            with self.state_lock:
                self.state = GraspState.MOVING

            # ── 阶段 7：验证末端位置 ──────────────────────────────────
            with self.state_lock:
                self.state = GraspState.VERIFYING

            ee_link    = 'R_base_link' if 'right' in candidate.group_name else 'L_base_link'
            base_frame = candidate.pose.header.frame_id or 'pelvis'
            move_success = self.verify_end_effector_position(
                candidate.pose.pose.position,
                candidate.pose.pose.orientation,
                base_frame=base_frame,
                ee_link=ee_link
            )
            if not move_success:
                self.get_logger().error('末端位置验证失败，可能未成功运动到目标位姿')
                self._wait_for_user_continue()
                return

            # ── 阶段 8：询问用户是否执行抓取 ─────────────────────────
            should_close = self._prompt_confirm_action('是否执行抓取（闭合手指）？')
            if not should_close:
                print('用户已放弃抓取，手臂退回安全姿态。')
                self._wait_for_user_continue()
                return

            self.hand_close(candidate.group_name)

            safe_pose = self.prepare_pose[candidate.group_name][1]

            # yes = self.arm_to_angle(safe_pose_after_grasp[candidate.group_name], spd=VELOCITY_LIMIT / 2, publish_ghost=False, require_confirm=False)
            yes = self.arm_to_angle(safe_pose_after_grasp[candidate.group_name], spd=VELOCITY_LIMIT / 2)

            # yes = self.arm_to_angle(safe_pose, spd=VELOCITY_LIMIT / 2)  # 运动到安全姿态
            if not yes:
                self.get_logger().error('已取消运动到安全位姿，本次抓取流程结束。')
                self._wait_for_user_continue()
                return

            # ── 阶段 9：放置 ──────────────────────────────────────────
            with self.state_lock:
                self.state = GraspState.PLACING

            place_location = self._prompt_place_location()
            self.get_logger().info(f'准备放置到: {place_location["name"]}')
            put_pose = place_location['pose'][candidate.group_name]
            
            yes = self.arm_to_angle(put_pose, spd=VELOCITY_LIMIT / 2)
            if not yes:
                self.get_logger().error('已取消运动到放置位姿，本次抓取流程结束。')
                self._wait_for_user_continue()
                return

            time.sleep(2.0)
            print(f'✓ 已将物体放置到: {place_location["name"]}')

            self.hand_open(candidate.group_name)  # 放开物体

            yes = self.arm_to_angle(safe_pose, spd=VELOCITY_LIMIT / 2)  # 运动到安全姿态
            if not yes:
                self.get_logger().error('已取消运动到安全位姿，本次抓取流程结束。')

            self._wait_for_user_continue()

        except Exception:
            self.get_logger().error(f'抓取流程异常:\n{traceback.format_exc()}')
        finally:
            with self.state_lock:
                self.state = GraspState.IDLE
            self.get_logger().info('状态已恢复为 IDLE')

    # ------------------------------------------------------------------
    # IK + 手臂运动（拆分为独立步骤，由 _run_grasp_sequence 编排）
    # ------------------------------------------------------------------

    def _solve_ik(self, candidate: GraspCandidate, x_offset: float = 0.0, y_offset: float = 0.0, z_offset: float = 0.0) -> dict | None:
        """执行 IK 解算，支持对目标位置施加偏移（失败返回 None）"""
        pos        = candidate.pose.pose.position
        orient     = candidate.pose.pose.orientation
        frame_id   = candidate.pose.header.frame_id or 'pelvis'
        group_name = candidate.group_name or 'left_arm'

        target_pos = type(pos)()
        target_pos.x = pos.x + x_offset
        target_pos.y = pos.y + y_offset
        target_pos.z = pos.z + z_offset

        response = self.call_ik_service_with_params(group_name, frame_id, target_pos, orient)
        if response is None:
            return None

        return self.extract_joint_positions(response, group_name)

    # ------------------------------------------------------------------
    # 用户交互方法（运行在后台线程，直接使用 input()）
    # ------------------------------------------------------------------

    def _print_candidate_info(self, candidate: GraspCandidate):
        print('\n' + '=' * 50)
        print('【检测到抓取目标】')
        print(f'  物体类别 : {candidate.object_class}')
        print(f'  YOLO置信度: {candidate.confidence:.2f}')
        print(f'  规划组   : {candidate.group_name}')
        pos = candidate.pose.pose.position
        print(f'  目标位置 : x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}')
        print(f'  坐标系   : {candidate.pose.header.frame_id}')
        print('=' * 50)

    def _prompt_confirm_action(self, question: str) -> bool:
        """通用 yes/no 交互确认，返回 True 表示确认执行"""
        while True:
            user_input = input(
                f'\n{question}\n'
                '  输入 yes/y 执行\n'
                '  输入 no/n  放弃\n'
                '请选择: '
            ).strip().lower()
            if user_input in ('yes', 'y'):
                return True
            if user_input in ('no', 'n'):
                return False
            print('✗ 无效输入，请输入 yes/y 或 no/n')

    def _prompt_place_location(self) -> dict:
        """询问用户选择放置位置，返回选定位置信息字典"""

        print('\n' + '-' * 50)
        print('请选择放置位置：')
        for key, info in place_locations.items():
            print(f'  {key}. {info["name"]:8} - {info["description"]}')
        print('-' * 50)

        while True:
            choice = input('请选择 (1-5): ').strip()
            if choice in place_locations:
                print(f'✓ 已选择: {place_locations[choice]["name"]}')
                return place_locations[choice]
            print('✗ 无效选择，请输入 1-5')

    def _wait_for_user_continue(self):
        """等待用户操作：Enter 继续；输入 q 并回车退出程序。"""
        print('-' * 50)
        user_input = input('按 Enter 键继续检测下一个物体；输入 q 并回车退出程序...\n').strip().lower()
        if user_input == 'q':
            self.get_logger().info('收到退出指令（q），正在关闭抓取程序...')
            if rclpy.ok():
                rclpy.shutdown()


def main():
    rclpy.init()
    node = GraspExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n用户中断程序')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
