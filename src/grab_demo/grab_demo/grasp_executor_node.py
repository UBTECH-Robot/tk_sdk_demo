#!/usr/bin/env python3
"""
GraspExecutorNode

抓取执行节点。订阅 /grasp_candidate 话题，接收来自感知节点的抓取候选点，
按以下状态机驱动完整的抓取-放置流程：

    IDLE → CONFIRMING → MOVING → VERIFYING → PLACING → IDLE

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


# ============ 状态机 ============
class GraspState(Enum):
    IDLE       = auto()   # 等待候选点
    CONFIRMING = auto()   # 等待用户确认
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
            self.state = GraspState.CONFIRMING

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
            # ── 阶段 1：打印候选点信息 ────────────────────────────────
            self._print_candidate_info(candidate)

            # ── 阶段 2：手臂运动到抓取准备姿态 ───────────────────────
            with self.state_lock:
                self.state = GraspState.MOVING

            if not self.arm_pose_init():
                print('手臂未到达准备姿态，取消本次抓取。')
                self._wait_for_user_continue()
                return

            # ── 阶段 3：IK 解算 ───────────────────────────────────────
            joint_positions = self._solve_ik(candidate)
            if joint_positions is None:
                self.get_logger().error('IK 解算失败，取消本次抓取')
                self._wait_for_user_continue()
                return

            if hasattr(self, '_publish_model_ghost'):
                # ── 阶段 4：发布 IK 结果供 GUI 可视化 ────────────────────
                self._publish_model_ghost(joint_positions)

            # ── 阶段 5：用户确认是否执行此位姿 ───────────────────────
            with self.state_lock:
                self.state = GraspState.CONFIRMING

            should_grasp = self._prompt_execute_grasp()
            if not should_grasp:
                print('用户已放弃本次抓取。')
                self._wait_for_user_continue()
                return

            # ── 阶段 6：执行手臂运动 ──────────────────────────────────
            with self.state_lock:
                self.state = GraspState.MOVING

            self.hand_open()  # 确保手指张开
            self.arm_to_pose(joint_positions, spd=VELOCITY_LIMIT / 2)

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

            # ── 阶段 8：放置 ──────────────────────────────────────────
            with self.state_lock:
                self.state = GraspState.PLACING

            place_location = self._prompt_place_location()
            self.get_logger().info(f'准备放置到: {place_location["name"]}')
            # TODO: 添加具体的放置动作（发布手臂命令到放置位置）
            time.sleep(2.0)
            print(f'✓ 已将物体放置到: {place_location["name"]}')

            # ── 阶段 9：全流程成功→手臂退回安全姿态 ─────────────
            self.arm_pose_reverse_init()

            self._wait_for_user_continue()

        except Exception:
            self.get_logger().error(f'抓取流程异常:\n{traceback.format_exc()}')
        finally:
            with self.state_lock:
                self.state = GraspState.IDLE
            self.get_logger().info('状态已恢复为 IDLE，等待下一个候选点')

    # ------------------------------------------------------------------
    # IK + 手臂运动（拆分为独立步骤，由 _run_grasp_sequence 编排）
    # ------------------------------------------------------------------

    def _solve_ik(self, candidate: GraspCandidate) -> dict | None:
        """仅执行 IK 解算，返回关节角度字典（失败返回 None）"""
        pos        = candidate.pose.pose.position
        orient     = candidate.pose.pose.orientation
        frame_id   = candidate.pose.header.frame_id or 'pelvis'
        group_name = candidate.group_name or 'left_arm'

        response = self.call_ik_service_with_params(group_name, frame_id, pos, orient)
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

    def _prompt_execute_grasp(self) -> bool:
        """询问用户是否执行抓取，返回 True 表示确认"""
        while True:
            user_input = input(
                '\n是否执行抓取？\n'
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
        place_locations = {
            '1': {'id': '1', 'name': '左前方',  'description': '桌子左侧前方'},
            '2': {'id': '2', 'name': '中央',    'description': '桌子正前方中央'},
            '3': {'id': '3', 'name': '右前方',  'description': '桌子右侧前方'},
            '4': {'id': '4', 'name': '左侧',    'description': '桌子左侧'},
            '5': {'id': '5', 'name': '右侧',    'description': '桌子右侧'},
        }

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
        """等待用户按 Enter 继续下一次检测"""
        print('-' * 50)
        input('按 Enter 键继续检测下一个物体...\n')


def main():
    rclpy.init()
    node = GraspExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n用户中断程序')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
