#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import json
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from std_msgs.msg import Header, String
import math
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
import time
from tf2_ros import Buffer, TransformListener
from grab_demo_msgs.msg import IKRequest
from grab_demo.pose_verification_mixin import PoseVerificationMixin

prepare_pose = {
    "left_arm": [
        {11: 0.5, 12: 0.15, 13: 0.1, 14: -0.9, 15: 0.2, 16: 0.0, 17: 0.0},
        {11: 0.5, 12: 0.4, 13: 0.1, 14: -2.0, 15: 0.2, 16: 0.0, 17: 0.0},
    ],
    "right_arm": [
        {21: 0.5, 22: -0.15, 23: -0.1, 24: -0.9, 25: -0.2, 26: 0.0, 27: 0.0},
        {21: 0.5, 22: -0.4, 23: -0.1, 24: -2.0, 25: -0.2, 26: 0.0, 27: 0.0},
    ],
    "head": [
        {1: 0.0, 2: 0.3, 3: 0.0},
    ]
}
reversed_prepare_pose = {
    "left_arm": [
        {11: 0.5, 12: 0.15, 13: 0.1, 14: -0.9, 15: 0.2, 16: 0.0, 17: 0.0},
        {11: 0.5, 12: 0.4, 13: 0.1, 14: -2.0, 15: 0.2, 16: 0.0, 17: 0.0},
        {11: 1.2, 12: 0.4, 13: 0.1, 14: -2.5, 15: 0.2, 16: 0.0, 17: 0.0},
    ],
    "right_arm": [
        {21: 0.5, 22: -0.15, 23: -0.1, 24: -0.9, 25: -0.2, 26: 0.0, 27: 0.0},
        {21: 0.5, 22: -0.4, 23: -0.1, 24: -2.0, 25: -0.2, 26: 0.0, 27: 0.0},
        {21: 1.2, 22: -0.4, 23: -0.1, 24: -2.5, 25: -0.2, 26: 0.0, 27: 0.0},
    ],
    "head": [
        {1: 0.0, 2: 0.3, 3: 0.0},
    ]
}

VELOCITY_LIMIT = 0.4  # 速度限制（弧度/秒）
CURRENT_LIMIT = 5.0  # 电流限制（安培）

class IKClientNode(PoseVerificationMixin, Node):
    def __init__(self):
        super().__init__('ik_client')
        self.client = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # TF2监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建发布器
        self.pose_publisher = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        
        self.arm_pos_cmd_publisher = self.create_publisher(CmdSetMotorPosition, '/arm/cmd_pos', 10)

        self.current_joint_state = None
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.get_logger().info("已订阅 /joint_states 话题")
        
        self.joint_command_pub = self.create_publisher(String, "/gui/joint_command", 10)
        self.get_logger().info("✓ GUI关节命令发布者已创建（话题：/gui/joint_command）")

        # 订阅IKRequest话题
        self.ik_request_sub = self.create_subscription(
            IKRequest,
            '/ik_request',
            self.ik_request_callback,
            10
        )
        self.get_logger().info("✓ 已订阅 /ik_request 话题")

        # 定义目标位姿
        # self.target_position = Point(x=0.374878, y=0.047079, z=0.310416)
        # self.target_orientation = Quaternion(x=-0.041013, y=0.712226, z=-0.690219, w=0.121038)

        self.target_position = Point(x=0.659072, y=0.316812, z=0.264747)
        self.target_orientation = Quaternion(x=-0.041013, y=0.712226, z=-0.690219, w=0.121038)

        self.frame_id = 'pelvis'
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待IK服务...')
    
    def joint_states_callback(self, msg):
        self.current_joint_state = msg

    def ik_request_callback(self, msg):
        """接收IKRequest消息，执行IK求解并控制手臂"""
        self.get_logger().info('=== 收到IK请求 ===')
        self.get_logger().info(f'  group_name: {msg.group_name}')
        self.get_logger().info(f'  frame_id: {msg.frame_id}')
        self.get_logger().info(f'  position: [{msg.position.x:.6f}, {msg.position.y:.6f}, {msg.position.z:.6f}]')
        self.get_logger().info(f'  orientation: [{msg.orientation.x:.6f}, {msg.orientation.y:.6f}, {msg.orientation.z:.6f}, {msg.orientation.w:.6f}]')
        
        self.target_position = msg.position
        self.target_orientation = msg.orientation
        self.frame_id = msg.frame_id

        # 发布抓取位姿到RViz可视化
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = msg.frame_id
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position = msg.position
        pose_msg.pose.orientation = msg.orientation
        self.pose_publisher.publish(pose_msg)
        
        # 调用IK服务（使用消息中的参数）
        response = self.call_ik_service_with_params(
            msg.group_name,
            msg.frame_id,
            msg.position,
            msg.orientation
        )
        
        if response:
            # 提取关节位置
            joint_positions = self.extract_joint_positions(response)
            
            if joint_positions:
                # 发布JSON命令到GUI
                json_output = json.dumps(joint_positions)
                self.get_logger().info(f'=== 发布关节命令 ===')
                self.get_logger().info(f'JSON命令: {json_output}')
                
                gui_msg = String()
                gui_msg.data = json_output
                self.joint_command_pub.publish(gui_msg)
                
                # 控制手臂移动到目标位置
                self.arm_to_pose(joint_positions, spd=VELOCITY_LIMIT/2)
                self.get_logger().info('✓ IK求解完成，手臂已移动到目标位置')
            else:
                self.get_logger().error('✗ 提取关节位置失败')
        else:
            self.get_logger().error('✗ IK服务调用失败')

    def create_joint_state_from_motor_dict(self, motor_positions_dict):
        """将电机位置字典转换为JointState消息"""
        motor_joint_map = {
            11: 'shoulder_pitch_l_joint',
            12: 'shoulder_roll_l_joint',
            13: 'shoulder_yaw_l_joint',
            14: 'elbow_pitch_l_joint',
            15: 'elbow_yaw_l_joint',
            16: 'wrist_pitch_l_joint',
            17: 'wrist_roll_l_joint'
        }
        
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # 先添加左臂7个关节（从字典中获取）
        for motor_id, joint_name in sorted(motor_joint_map.items()):
            motor_id_str = str(motor_id)
            if motor_id in motor_positions_dict:
                joint_state.name.append(joint_name)
                joint_state.position.append(motor_positions_dict[motor_id])
            elif motor_id_str in motor_positions_dict:
                joint_state.name.append(joint_name)
                joint_state.position.append(motor_positions_dict[motor_id_str])
        
        # 添加其他关节（使用当前状态的值，避免IK求解器因缺少关节而失败）
        if self.current_joint_state is not None:
            for i, name in enumerate(self.current_joint_state.name):
                if name not in joint_state.name:
                    joint_state.name.append(name)
                    joint_state.position.append(self.current_joint_state.position[i])
        
        return joint_state

    def publish_grasp_pose(self):
        """发布抓取位姿到 /grasp_pose 话题"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position = self.target_position
        pose_msg.pose.orientation = self.target_orientation
        
        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f'已发布抓取位姿到 /grasp_pose 话题')
    
    def call_ik_service_with_params(self, group_name, frame_id, position, orientation):
        """使用指定参数调用IK服务"""
        # 创建请求
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        
        # 归一化四元数（确保是单位四元数）
        q = orientation
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        normalized_quat = Quaternion(
            x=q.x/norm, 
            y=q.y/norm, 
            z=q.z/norm, 
            w=q.w/norm
        )
        
        # 设置目标位姿
        request.ik_request.pose_stamped.header.frame_id = frame_id
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose.position = position
        request.ik_request.pose_stamped.pose.orientation = normalized_quat
        
        self.get_logger().info('=== 目标位姿 ===')
        self.get_logger().info(f'  group_name: {group_name}')
        self.get_logger().info(f'  frame_id: {frame_id}')
        self.get_logger().info(f'  position: [{position.x:.6f}, {position.y:.6f}, {position.z:.6f}]')
        self.get_logger().info(f'  orientation: [{normalized_quat.x:.6f}, {normalized_quat.y:.6f}, {normalized_quat.z:.6f}, {normalized_quat.w:.6f}]')
        self.get_logger().info(f'  四元数模: {norm:.6f}')

        # 根据group_name选择IK种子
        if 'left' in group_name:
            seed_pose = prepare_pose["left_arm"][1]
        elif 'right' in group_name:
            seed_pose = prepare_pose["right_arm"][1]
        else:
            seed_pose = prepare_pose["left_arm"][1]
        self.get_logger().info('=== 使用的IK种子状态 ===')
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = self.create_joint_state_from_motor_dict(seed_pose)
        request.ik_request.avoid_collisions = True
        
        # 设置超时
        request.ik_request.timeout.sec = 5
        
        # 发送请求
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            return response
        else:
            self.get_logger().error('服务调用失败')
            return None
    
    def call_ik_service(self):
        """使用类属性调用IK服务（向后兼容）"""
        return self.call_ik_service_with_params(
            'left_arm',
            self.frame_id,
            self.target_position,
            self.target_orientation
        )
    
    def extract_joint_positions(self, response):
        # 定义需要提取的关节名称及其对应的电机ID
        joint_motor_map = {
            'shoulder_pitch_l_joint': '11',
            'shoulder_roll_l_joint': '12',
            'shoulder_yaw_l_joint': '13',
            'elbow_pitch_l_joint': '14',
            'elbow_yaw_l_joint': '15',
            'wrist_pitch_l_joint': '16',
            'wrist_roll_l_joint': '17'
        }
        
        # 提取关节位置
        result_dict = {}
        
        self.get_logger().info(f'IK错误码: {response.error_code.val}')
        
        if response.error_code.val == 1:  # SUCCESS
            joint_names = response.solution.joint_state.name
            joint_positions = response.solution.joint_state.position
            
            self.get_logger().info(f'=== IK返回的完整关节解 ===')
            self.get_logger().info(f'IK返回关节总数: {len(joint_names)}')
            
            # 显示所有返回的关节
            # for i, joint_name in enumerate(joint_names):
            #     if joint_name in joint_motor_map:
            #         marker = "→" if joint_name in joint_motor_map else " "
            #         self.get_logger().info(f'{marker} [{i}] {joint_name}: {joint_positions[i]:.5f}')
            
            self.get_logger().info(f'=== 提取左臂7个关节 ===')
            for i, joint_name in enumerate(joint_names):
                if joint_name in joint_motor_map:
                    motor_id = joint_motor_map[joint_name]
                    result_dict[motor_id] = round(joint_positions[i], 5)
                    self.get_logger().info(f'  [{i}] {joint_name} → 电机{motor_id}: {joint_positions[i]:.5f}')
            
            # 检查是否所有7个关节都找到了
            if len(result_dict) != 7:
                self.get_logger().error(f'⚠ 警告：只找到{len(result_dict)}个左臂关节，应为7个！')
            
            # 按照电机ID排序
            sorted_dict = dict(sorted(result_dict.items(), key=lambda x: int(x[0])))
            return sorted_dict
        else:
            self.get_logger().error(f'IK求解失败，错误码: {response.error_code.val}')
            return None
    
    def verify_ik_solution_with_fk(self, joint_positions_dict, response):
        """IK解算结果角度进行FK计算后的位姿与指定目标位姿对比"""
        self.get_logger().info('=== 开始FK验证IK解 ===')
        
        # 等待FK服务
        if not self.fk_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('FK服务不可用')
            return
        
        # 构建FK请求
        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = self.frame_id
        fk_request.fk_link_names = ['L_base_link']  # 末端执行器链接
        
        # 使用IK解算出的关节状态
        fk_request.robot_state = RobotState()
        fk_request.robot_state.joint_state = response.solution.joint_state
        
        # 调用FK服务
        fk_future = self.fk_client.call_async(fk_request)
        rclpy.spin_until_future_complete(self, fk_future)
        
        if fk_future.result() is not None:
            fk_response = fk_future.result()
            if fk_response.error_code.val == 1 and len(fk_response.pose_stamped) > 0:
                fk_pose = fk_response.pose_stamped[0].pose
                
                # 使用公共方法比较位姿
                is_ok, pos_error, angle_diff = self.compare_poses(
                    fk_pose.position, fk_pose.orientation,
                    self.target_position, self.target_orientation,
                    pos_threshold=0.001, angle_threshold=1.0,
                    pos1_label="IK解对应的FK计算",
                    pos2_label="IK目标"
                )
                
                # 根据结果输出不同级别的日志
                if is_ok:
                    self.get_logger().info('✓ IK解的FK验证通过：IK找到了正确的解')
                elif pos_error > 0.1:
                    self.get_logger().error(
                        f'✗✗✗ IK解的FK验证失败！IK求解器返回了错误的解！\n'
                        f'    位置误差高达 {pos_error*1000:.1f}mm\n'
                        f'    >>> 这说明MoveIt的IK配置有严重问题\n'
                        f'    >>> 检查 kinematics.yaml 中的 ik_link 是否正确\n'
                        f'    >>> 或者IK请求的frame_id不正确'
                    )
                else:
                    self.get_logger().warn(f'⚠ IK解精度较低: 位置误差{pos_error*1000:.1f}mm, 角度差{angle_diff:.1f}°')
            else:
                self.get_logger().error(f'FK求解失败: {fk_response.error_code.val}')
    
    def verify_execution(self, target_joint_positions_dict):
        """验证关节是否真的移动到了目标位置"""
        self.get_logger().info('=== 等待6秒后验证执行结果 ===')
        time.sleep(6.0)
        
        # 刷新关节状态
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.2)
        
        while self.current_joint_state is None:
            self.get_logger().info('等待接收当前关节状态...')
            rclpy.spin_once(self, timeout_sec=1.0)
        
        # 关节名称映射
        motor_joint_map = {
            '11': 'shoulder_pitch_l_joint',
            '12': 'shoulder_roll_l_joint',
            '13': 'shoulder_yaw_l_joint',
            '14': 'elbow_pitch_l_joint',
            '15': 'elbow_yaw_l_joint',
            '16': 'wrist_pitch_l_joint',
            '17': 'wrist_roll_l_joint'
        }
        
        self.get_logger().info('=== 目标 vs 实际关节角度对比 ===')
        all_match = True
        max_error = 0.0
        max_error_joint = None
        
        for motor_id, target_pos in target_joint_positions_dict.items():
            joint_name = motor_joint_map.get(motor_id)
            if joint_name and joint_name in self.current_joint_state.name:
                idx = self.current_joint_state.name.index(joint_name)
                actual_pos = self.current_joint_state.position[idx]
                error = abs(actual_pos - target_pos)
                
                if error > max_error:
                    max_error = error
                    max_error_joint = joint_name
                
                status = "✓" if error < 0.02 else "✗"
                self.get_logger().info(
                    f'{status} 电机{motor_id} ({joint_name}): '
                    f'目标={target_pos:.5f}, 实际={actual_pos:.5f}, 误差={error:.5f} rad ({error*180/math.pi:.2f}°)'
                )
                
                if error >= 0.02:
                    all_match = False
            else:
                self.get_logger().warn(f'⚠ 电机{motor_id}对应的关节{joint_name}未找到')
        
        if all_match:
            self.get_logger().info('✓ 所有关节都已到达目标位置（误差<0.02rad=1.15°）')
        else:
            self.get_logger().error(
                f'✗ 部分关节未到达目标位置！\n'
                f'  最大误差：{max_error:.5f} rad ({max_error*180/math.pi:.2f}°) 在关节 {max_error_joint}\n'
                f'  → 这可能是末端位置误差的主要原因'
            )
        
        # 查询实际末端位置（通过TF2）
        self.verify_end_effector_position(self.target_position, self.target_orientation, self.frame_id)
        
        # 关键验证：执行后再次检查FK与TF2是否一致
        self.get_logger().info('=== 关键诊断：执行后FK vs TF2验证 ===')
        self.diagnose_fk_vs_tf_after_execution()
    
    def diagnose_fk_vs_tf_after_execution(self):
        """末端真实TF2位姿与用当前角度FK计算结果位姿对比"""
        self.get_logger().info('使用当前实际关节状态验证FK与TF2...')
        
        base_frame = self.frame_id
        
        # 1. 获取TF2当前位置（使用辅助方法）
        transform, tf_pos, tf_quat = self.get_current_end_effector_transform(base_frame, 'L_base_link', max_retries=3)
        if transform is None:
            return
        
        self.get_logger().info(f'参考坐标系: {base_frame}')
        
        # 2. 使用当前实际关节状态做FK计算
        if not self.fk_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('FK服务不可用')
            return
        
        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = base_frame
        fk_request.fk_link_names = ['L_base_link']
        fk_request.robot_state = RobotState()
        fk_request.robot_state.joint_state = self.current_joint_state
        
        fk_future = self.fk_client.call_async(fk_request)
        rclpy.spin_until_future_complete(self, fk_future)
        
        if fk_future.result() is not None:
            fk_response = fk_future.result()
            if fk_response.error_code.val == 1 and len(fk_response.pose_stamped) > 0:
                fk_pose = fk_response.pose_stamped[0].pose
                
                # 使用公共方法比较位姿（只比较位置）
                is_ok, pos_error, _ = self.compare_poses(
                    fk_pose.position, None,
                    tf_pos, None,
                    pos_threshold=0.001,
                    pos1_label="FK计算",
                    pos2_label="TF2当前"
                )
                
                # 根据结果输出日志
                if is_ok:
                    self.get_logger().info(f'✓ 执行后FK与TF2仍然一致！（在{base_frame}坐标系下）')
                    self.get_logger().info('✓ URDF模型正确，手臂成功移动到期望位姿！')
                else:
                    self.get_logger().error(
                        f'✗ 执行后FK与TF2不一致！({pos_error*1000:.1f}mm)\n'
                        f'>>> URDF模型在不同关节配置下不一致\n'
                        f'>>> 这说明URDF定义有问题，或者joint_states发布的joint顺序/名称不一致'
                    )
            else:
                self.get_logger().error(f'FK求解失败: {fk_response.error_code.val}')
    
    def create_header(self):
        """
        创建消息头
        
        返回值：
            Header: 包含时间戳和坐标系信息的消息头
            
        说明：
            所有命令消息都需要包含header信息，用于时间同步和坐标系标识
        """
        now = self.get_clock().now()
        header = Header()
        header.stamp.sec = int(now.nanoseconds // 1_000_000_000)  # 转换为秒
        header.stamp.nanosec = int(now.nanoseconds % 1_000_000_000)  # 剩余纳秒
        header.frame_id = 'arm'  # 坐标系ID
        
        return header
    
    def arm_to_pose(self, pose, spd=VELOCITY_LIMIT):
        """
        将电机位置字典转换为关节位置列表
        
        参数：
            pose (dict): 包含电机ID和目标位置的字典，例如 {'11': 0.5, '12': 0.15, ...} 或者 {11: 0.5, 12: 0.15, ...} 
        """
        header = self.create_header()
        
        # 创建位置模式命令消息
        msg = CmdSetMotorPosition()
        msg.header = header
        
        # 为每个电机创建回零命令
        for k,v in pose.items():
            # 创建单个电机的位置命令
            cmd = SetMotorPosition()
            cmd.name = int(k)  # 电机ID
            cmd.pos = v
            cmd.spd = spd  # 速度限制（弧度/秒）
            cmd.cur = CURRENT_LIMIT  # 电流限制（安培）
            
            # 添加到消息数组
            msg.cmds.append(cmd)
            
            self.get_logger().info(f"  电机 {k}：运动到目标位置（{v} rad）")
        
        # 发送命令
        self.arm_pos_cmd_publisher.publish(msg)
        self.get_logger().info("✓ 手臂运动命令已发送")
        
        time.sleep(6)  # 给电机足够的时间运动到目标位置

    def arm_pose_init(self):
        left_poses = prepare_pose['left_arm']
        right_poses = prepare_pose['right_arm']
        for idx, pose in enumerate(left_poses):
            merged_poses = {**pose, **right_poses[idx]}
            user_input = input(
                "\n是否移动手臂到下一个位置？\n"
                "  输入 'yes'/'y' 确定\n"
                "  输入 'no'/'n' 取消移动\n"
                "请选择 (yes/no): "
            ).strip().lower()
            
            # 规范化用户输入
            if user_input in ['yes', 'y']:
                user_choice = 'yes'
            elif user_input in ['no', 'n']:
                return
            else:
                # 无效输入，提示重新输入
                print("✗ 无效的输入，取消移动")
                return
            
            self.arm_to_pose(merged_poses)

def ros_main():
    """
    ROS节点模式：持续运行，监听/ik_request话题
    通过 ros2 run grab_demo ik_client_node 启动
    """
    rclpy.init()
    
    ik_client = IKClientNode()
    
    ik_client.get_logger().info('=== IK客户端节点已启动 ===')
    ik_client.get_logger().info('等待 /ik_request 话题消息...')
    ik_client.get_logger().info('发布IKRequest消息到 /ik_request 话题以触发IK求解')
    
    try:
        rclpy.spin(ik_client)
    except KeyboardInterrupt:
        ik_client.get_logger().info('收到中断信号，正在关闭节点...')
    finally:
        ik_client.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    
    ik_client = IKClientNode()
        
    ik_client.arm_pose_init()
    return
    # 发布抓取位姿
    ik_client.publish_grasp_pose()
    
    # 调用IK服务
    response = ik_client.call_ik_service()
    
    if response:
        # 提取关节位置
        joint_positions = ik_client.extract_joint_positions(response)
        
        if joint_positions:
            # 验证IK解（数学验证）
            # ik_client.verify_ik_solution_with_fk(joint_positions, response)
            
            # 转换为JSON字符串并输出
            json_output = json.dumps(joint_positions)
            ik_client.get_logger().info(f'=== 发布关节命令 ===')
            ik_client.get_logger().info(f'JSON命令: {json_output}')
            print(json_output)

            msg = String()
            msg.data = json_output            
            ik_client.joint_command_pub.publish(msg)

            ik_client.arm_to_pose(joint_positions, spd=VELOCITY_LIMIT/2)
            
            # 验证执行结果（物理验证）
            # ik_client.verify_execution(joint_positions)

    
    ik_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
