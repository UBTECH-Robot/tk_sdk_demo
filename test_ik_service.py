#!/usr/bin/env python3
"""诊断脚本：测试 IK 服务和 move_group 状态"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import time

class IKDiagnostics(Node):
    def __init__(self):
        super().__init__('ik_diagnostics')
        
        # 订阅关节状态
        self.joint_state = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        
        # 等待一条消息
        print("[等待] 接收 /joint_states 消息...")
        for _ in range(50):  # 等待5秒
            if self.joint_state is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.joint_state is None:
            print("[错误] 无法接收 /joint_states 消息")
            return
        
        print(f"[成功] 接收到 joint_state，包含 {len(self.joint_state.name)} 个关节")
        print(f"  关节名: {self.joint_state.name[:7]}...")  # 显示前7个
        print(f"  关节数: {len(self.joint_state.position)}")
        
        # 检查 IK 服务
        ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        print("\n[检查] 等待 /compute_ik 服务...")
        if not ik_client.wait_for_service(timeout_sec=5.0):
            print("[错误] /compute_ik 服务不可用")
            return
        
        print("[成功] /compute_ik 服务已就绪")
        
        # 发送一个简单的 IK 请求
        print("\n[测试] 发送 IK 请求...")
        
        request = GetPositionIK.Request()
        request.ik_request.group_name = "left_arm"
        
        # 设置目标位姿
        pose = PoseStamped()
        pose.header.frame_id = "pelvis"
        pose.pose.position.x = 0.3
        pose.pose.position.y = 0.3
        pose.pose.position.z = 0.5
        pose.pose.orientation.w = 1.0
        
        request.ik_request.pose_stamped = pose
        request.ik_request.timeout.sec = 2
        request.ik_request.avoid_collisions = True
        
        # 设置机器人状态
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = self.joint_state
        
        print(f"  目标位姿: x={pose.pose.position.x}, y={pose.pose.position.y}, z={pose.pose.position.z}")
        print(f"  规划组: {request.ik_request.group_name}")
        print(f"  超时: {request.ik_request.timeout.sec}s")
        
        start_time = time.time()
        future = ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        elapsed = time.time() - start_time
        
        if not future.done():
            print(f"[错误] IK 请求超时 (等待时间: {elapsed:.1f}s)")
            return
        
        response = future.result()
        error_code = response.error_code.val
        
        print(f"\n[结果]")
        print(f"  错误码: {error_code}")
        print(f"  响应时间: {elapsed:.2f}s")
        
        if error_code == 1:
            print("[成功] IK 解算成功！")
            joint_state = response.solution.joint_state
            print(f"  解算出的关节数: {len(joint_state.position)}")
            for name, pos in zip(joint_state.name[:7], joint_state.position[:7]):
                print(f"    {name}: {pos:.4f} rad")
        else:
            error_messages = {
                -1: "解算失败",
                -2: "超时",
                -31: "无 IK 解",
            }
            msg = error_messages.get(error_code, f"未知错误 (code={error_code})")
            print(f"[失败] IK 解算失败: {msg}")
            
            # 打印更多诊断信息
            print(f"\n[诊断] robot_state 信息:")
            print(f"  是否包含 joint_state: {len(request.ik_request.robot_state.joint_state.name) > 0}")
            print(f"  joint_state 名称数: {len(request.ik_request.robot_state.joint_state.name)}")
            print(f"  joint_state 位置数: {len(request.ik_request.robot_state.joint_state.position)}")
    
    def joint_state_cb(self, msg):
        self.joint_state = msg


if __name__ == '__main__':
    rclpy.init()
    node = IKDiagnostics()
    rclpy.shutdown()
