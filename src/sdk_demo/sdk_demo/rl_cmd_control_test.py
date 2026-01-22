#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from hric_msgs.srv import SetMotionMode, SetMotionNumber
from hric_msgs.msg import MotionStatus, ActionStatus

class MotionTestNode(Node):
    def __init__(self):
        super().__init__('motion_test_node')
        
        # 初始化客户端
        self.mode_client = self.create_client(SetMotionMode, '/hric/motion/set_motion_mode')
        self.action_client = self.create_client(SetMotionNumber, '/hric/motion/set_motion_number')
        
        # 等待服务
        self.get_logger().info("等待服务上线...")
        # 订阅 /hric/motion/status，保存最新状态
        self.latest_status = None
        self.create_subscription(MotionStatus, '/hric/motion/status', self.status_callback, 10)
        # 订阅 /hric/robot/action_status，保存最新动作状态
        self.latest_action_status = None
        self.create_subscription(ActionStatus, '/hric/robot/action_status', self.action_status_callback, 10)
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            print(".", end="", flush=True)
        while not self.action_client.wait_for_service(timeout_sec=1.0):
            print(".", end="", flush=True)
        print("\n✅ 服务已连接")

    def status_callback(self, msg: MotionStatus):
        # 保存最新收到的状态消息
        self.latest_status = msg

    def action_status_callback(self, msg: ActionStatus):
        # 保存最新收到的动作状态消息
        self.latest_action_status = msg

    def ensure_no_motion(self, timeout: float = 5.0) -> bool:
        """Wait until no motion is executing. Returns True if no motion, False on timeout or unknown."""
        waited = 0.0
        interval = 0.1
        while waited < timeout:
            # If we have a status and it's not executing motion, we're good
            if self.latest_action_status is not None and not self.latest_action_status.is_motion:
                return True
            print(f"当前动作状态是[{self.latest_action_status.is_motion if self.latest_action_status else '未知'}]，等待动作执行完毕...")
            # rclpy.spin_once(self, timeout_sec=interval)
            time.sleep(interval)
            waited += interval
        return False

    def ensure_stand(self, timeout: float = 5.0) -> bool:
        """Wait until walk_mode == 3 (stand). Returns True if stand, False on timeout or unknown."""
        waited = 0.0
        interval = 0.1
        while waited < timeout:
            if self.latest_status is not None and self.latest_status.walk_mode == 3:
                return True
            print(f"当前是[{self.latest_status.walk_mode if self.latest_status else '未知'}]，等待进入站立模式...")
            # rclpy.spin_once(self, timeout_sec=interval)
            time.sleep(interval)
            waited += interval
        return False

    def set_mode_sync(self, mode):
        """同步设置模式"""
        req = SetMotionMode.Request()
        req.walk_mode_request = mode
        req.is_need_swing_arm = True
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
        # 等待并检查 motion status，若为遥控器控制或非站立模式则退出
        print("等待 motion status 消息（最多 5s）...")
        waited = 0.0
        interval = 0.1
        timeout = 5.0
        while node.latest_status is None and waited < timeout:
            rclpy.spin_once(node, timeout_sec=interval)
            waited += interval

        if node.latest_status is None:
            print("未收到 /hric/motion/status，退出程序")
            return

        status = node.latest_status
        print(f"已收到 motion status：{status}")
        # 根据描述：is_console_control True 表示导航操控，False 表示遥控器操控
        if not status.is_console_control:
            print("检测到遥控器操作，程序退出")
            return

        # 需要处于站立模式 (walk_mode == 3)
        if status.walk_mode != 3:
            print(f"当前行走模式为 {status.walk_mode}，非站立模式，程序退出")
            return


        print("=== 当前为站立模式，开始模拟主程序流程 ===")

        # 0. 模拟主程序：先执行一次挥手 (Action 1)
        node.ensure_no_motion()
        node.ensure_stand()
        print(">>> ！！！尝试挥手 0！！！")
        node.do_action_sync(1)

        node.ensure_no_motion()
        node.ensure_stand()
        
        # 1. 模拟主程序：先切到原地踏步 (Mode 4)
        node.set_mode_sync(4)
        node.ensure_no_motion()
        time.sleep(3.0)

        # 2. 模拟主程序：切到站立 (Mode 3)
        node.set_mode_sync(3)
        node.ensure_no_motion()
        node.ensure_stand()
        time.sleep(3.0)

        # 3. 模拟主程序：执行挥手 (Action 1)
        print(">>> ！！！尝试挥手 1！！！")
        node.do_action_sync(1)
        
        node.ensure_no_motion()
        node.ensure_stand()
        time.sleep(8.0)

        print("=== 测试结束 ===")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# 天工Pro SDK V2.0.6 也存在如下问题：
# 机器人平稳落地进入站立，全身运控模式后，将E上拨，
# 在终端直接通过接口调用机器人出厂动作挥手（ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: true, motion_number: 1}"），成功执行挥手动作
# 动作执行完毕后，执行本节点：ros2 run sdk_demo rl_cmd_control_test

# 期望结果是：
# 1. 先挥手
# 2，进入行走（原地踏步）
# 3. 切回站立
# 4. 再次挥手
# 5. 最终机器人保持站立状态

# 实际结果是：
# 1. 先挥手成功
# 2. 进入行走（原地踏步）成功
# 3. 切回站立成功
# 4. 再次挥手失败，机器人仅仅腰部关节有轻微的扭动，双臂无任何动作反应 - 与预期不符合
# 5. 腰部扭动结束后，机器人保持站立
