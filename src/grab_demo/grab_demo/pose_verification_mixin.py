#!/usr/bin/env python3
"""
PoseVerificationMixin

末端位姿验证功能的 Mixin 类，可被任意 rclpy.Node 子类混入。

宿主类需提供：
  - self.tf_buffer (tf2_ros.Buffer)
  - self.get_logger()
  - rclpy.spin_once(self, ...) 可被正常调用（即宿主是一个 Node）

不继承 Node，避免多继承时 MRO 冲突与重复初始化。
"""

import math
import rclpy
from geometry_msgs.msg import Quaternion
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class PoseVerificationMixin:
    """末端位姿验证功能 Mixin"""

    def compare_poses(self, pos1, quat1, pos2, quat2,
                      pos_threshold=None, angle_threshold=None,
                      pos1_label="位姿1", pos2_label="位姿2"):
        """
        比较两个位姿的差异

        参数:
            pos1: Point - 第一个位姿的位置
            quat1: Quaternion/None - 第一个位姿的姿态（为 None 则不比较姿态）
            pos2: Point - 第二个位姿的位置
            quat2: Quaternion/None - 第二个位姿的姿态（为 None 则不比较姿态）
            pos_threshold: float - 位置阈值(米)，设置后用于判断
            angle_threshold: float - 角度阈值(度)，设置后用于判断
            pos1_label: str - 位姿1 的标签（用于日志）
            pos2_label: str - 位姿2 的标签（用于日志）

        返回: (is_within_threshold, pos_error_m, angle_diff_deg)
            - is_within_threshold: bool/None（有阈值时返回判断结果，否则返回 None）
            - pos_error_m: float - 位置误差(米)
            - angle_diff_deg: float/None - 角度误差(度)，无姿态时返回 None
        """
        # 计算位置误差
        pos_error = math.sqrt(
            (pos1.x - pos2.x) ** 2 +
            (pos1.y - pos2.y) ** 2 +
            (pos1.z - pos2.z) ** 2
        )

        self.get_logger().info(f'{pos1_label}位置: [{pos1.x:.6f}, {pos1.y:.6f}, {pos1.z:.6f}]')
        self.get_logger().info(f'{pos2_label}位置: [{pos2.x:.6f}, {pos2.y:.6f}, {pos2.z:.6f}]')
        self.get_logger().info(f'位置误差: {pos_error:.6f} 米 ({pos_error * 1000:.3f} 毫米)')

        angle_diff = None

        if quat1 is not None and quat2 is not None:
            norm1 = math.sqrt(quat1.x ** 2 + quat1.y ** 2 + quat1.z ** 2 + quat1.w ** 2)
            norm2 = math.sqrt(quat2.x ** 2 + quat2.y ** 2 + quat2.z ** 2 + quat2.w ** 2)

            q1_norm = Quaternion(
                x=quat1.x / norm1, y=quat1.y / norm1,
                z=quat1.z / norm1, w=quat1.w / norm1
            )
            q2_norm = Quaternion(
                x=quat2.x / norm2, y=quat2.y / norm2,
                z=quat2.z / norm2, w=quat2.w / norm2
            )

            dot_product = (
                q1_norm.x * q2_norm.x +
                q1_norm.y * q2_norm.y +
                q1_norm.z * q2_norm.z +
                q1_norm.w * q2_norm.w
            )
            angle_diff = 2 * math.acos(min(abs(dot_product), 1.0)) * 180 / math.pi

            self.get_logger().info(
                f'{pos1_label}姿态: [{q1_norm.x:.6f}, {q1_norm.y:.6f}, {q1_norm.z:.6f}, {q1_norm.w:.6f}]'
            )
            self.get_logger().info(
                f'{pos2_label}姿态: [{q2_norm.x:.6f}, {q2_norm.y:.6f}, {q2_norm.z:.6f}, {q2_norm.w:.6f}]'
            )
            self.get_logger().info(f'姿态角度差: {angle_diff:.3f} 度')

        is_within_threshold = None
        if pos_threshold is not None:
            pos_ok = pos_error < pos_threshold
            if angle_threshold is not None and angle_diff is not None:
                is_within_threshold = pos_ok and (angle_diff < angle_threshold)
            else:
                is_within_threshold = pos_ok

        return is_within_threshold, pos_error, angle_diff

    def get_current_end_effector_transform(self,
                                           base_frame='pelvis',
                                           ee_link='L_base_link',
                                           max_retries=5):
        """
        获取当前末端执行器的 TF2 变换

        参数:
            base_frame: str - 参考坐标系，默认 'pelvis'
            ee_link:    str - 末端执行器链接名，默认 'L_base_link'（右臂传 'R_base_link'）
            max_retries: int - 最大重试次数

        返回: (transform, actual_pos, actual_quat) 或 (None, None, None)
        """
        try:
            transform = None
            for attempt in range(max_retries):
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.2)

                try:
                    transform = self.tf_buffer.lookup_transform(
                        base_frame,
                        ee_link,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    break
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                    if attempt < max_retries - 1:
                        self.get_logger().debug(f'TF查询重试 {attempt + 1}/{max_retries}...')
                        for _ in range(10):
                            rclpy.spin_once(self, timeout_sec=0.2)
                    else:
                        raise

            if transform is None:
                self.get_logger().error(f'无法获取 {base_frame} → {ee_link} 变换')
                return None, None, None

            actual_pos = transform.transform.translation
            actual_quat = transform.transform.rotation
            return transform, actual_pos, actual_quat

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'TF2查询失败: {e}')
            return None, None, None
        except Exception as e:
            self.get_logger().error(f'TF2查询异常: {e}')
            return None, None, None

    def verify_end_effector_position(self,
                                     target_position,
                                     target_orientation,
                                     base_frame='pelvis',
                                     ee_link='L_base_link'):
        """
        末端真实 TF2 位姿与指定目标位姿对比

        参数:
            target_position:    geometry_msgs.msg.Point - 目标位置
            target_orientation: geometry_msgs.msg.Quaternion - 目标姿态
            base_frame: str - 参考坐标系，默认 'pelvis'
            ee_link:    str - 末端执行器链接名，默认 'L_base_link'
        """
        self.get_logger().info('=== TF2查询实际末端位置 ===')

        transform, actual_pos, actual_quat = self.get_current_end_effector_transform(
            base_frame, ee_link
        )
        if transform is None:
            return False

        self.get_logger().info(f'参考坐标系: {base_frame}')

        is_ok, pos_error, angle_diff = self.compare_poses(
            actual_pos, actual_quat,
            target_position, target_orientation,
            pos_threshold=0.005, angle_threshold=5.0,
            pos1_label="TF2实际末端", pos2_label="IK目标"
        )

        if is_ok:
            self.get_logger().info('✓ 末端位置验证通过！执行成功！')
            return True
        else:
            self.get_logger().error(
                f'✗ 末端位置误差过大！\n'
                f'  位置差: {pos_error * 1000:.1f}mm (阈值: 5mm)\n'
                f'  角度差: {angle_diff:.1f}° (阈值: 5°)\n'
                f'  → 可能的问题：IK求解精度不足或关节控制未到位'
            )
            return False
