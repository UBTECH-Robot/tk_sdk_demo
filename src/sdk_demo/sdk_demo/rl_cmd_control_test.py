#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from hric_msgs.srv import SetMotionMode, SetMotionNumber

# 机器人进入站立，全身运控模式
# 通过接口调用机器人出厂动作，成功执行
# 动作执行完毕后，调用运动模式切换接口，进入行走（原地踏步）
# 通过接口调用恢复站立
# 调用动作编号接口，机器人仅腰部动，手不动，如视频

class MotionTestNode(Node):
    def __init__(self):
        super().__init__('motion_test_node')
        
        # 初始化客户端
        self.mode_client = self.create_client(SetMotionMode, '/hric/motion/set_motion_mode')
        self.action_client = self.create_client(SetMotionNumber, '/hric/motion/set_motion_number')
        
        # 等待服务
        self.get_logger().info("等待服务上线...")
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            print(".", end="", flush=True)
        while not self.action_client.wait_for_service(timeout_sec=1.0):
            print(".", end="", flush=True)
        print("\n✅ 服务已连接")

    def set_mode_sync(self, mode):
        """同步设置模式"""
        req = SetMotionMode.Request()
        req.walk_mode_request = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result and result.success:
            self.get_logger().info(f"✅ 切换模式 {mode} 成功")
        else:
            self.get_logger().error(f"❌ 切换模式 {mode} 失败")

    def do_action_sync(self, num):
        """同步执行动作"""
        req = SetMotionNumber.Request()
        req.is_motion = True
        req.motion_number = num
        future = self.action_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result and result.success:
            self.get_logger().info(f"✅ 动作 {num} 指令已发送")
        else:
            self.get_logger().error(f"❌ 动作 {num} 指令发送失败")

def main(args=None):
    rclpy.init(args=args)
    node = MotionTestNode()

    try:
        print("=== 开始模拟主程序流程 ===")
        
        # 1. 模拟主程序：先切到原地踏步 (Mode 4)
        # node.set_mode_sync(4)
        # print(">>> 等待 1 秒 (模拟移动耗时)...")
        # time.sleep(1.0)
        
        # 2. 模拟主程序：切到站立 (Mode 3)
        node.set_mode_sync(3)
        print(">>> 等待 1 秒 (模拟稳定)...")
        time.sleep(5.0)
        
        # 3. 模拟主程序：执行挥手 (Action 1)
        print(">>> ！！！尝试挥手！！！")
        node.do_action_sync(1)
        
        print(">>> 等待 3 秒看动作效果...")
        time.sleep(3.0)
        
        print("=== 测试结束 ===")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()