#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
灵巧手状态监测演示节点

说明：
    该节点订阅灵巧手的左右手状态话题，实时获取并输出详细的关节状态信息。
    灵巧手共有6个关节：小指、无名指、中指、食指、拇指弯曲、拇指旋转。
    所有数据都包含位置、速度和力反馈的百分比信息。

使用方式：
    ros2 run sdk_demo hand_status

订阅话题：
  - /inspire_hand/state/left_hand  - 左手状态
  - /inspire_hand/state/right_hand - 右手状态

话题数据类型：sensor_msgs/JointState
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class InspireHandStatusMonitor(Node):
    """灵巧手状态监测类"""
    
    # 关节名称映射字典
    # 关节ID对应的中文名称
    JOINT_NAMES = {
        "1": "小指",      # ID = 1
        "2": "无名指",    # ID = 2
        "3": "中指",      # ID = 3
        "4": "食指",      # ID = 4
        "5": "拇指弯曲",  # ID = 5
        "6": "拇指旋转"   # ID = 6
    }
    
    def __init__(self):
        """初始化节点"""
        super().__init__('inspire_hand_status_monitor')
        
        # 创建左手状态订阅器
        self.left_hand_subscription = self.create_subscription(
            JointState,
            '/inspire_hand/state/left_hand',
            self.left_hand_callback,
            10  # QoS队列深度
        )
        
        # 创建右手状态订阅器
        self.right_hand_subscription = self.create_subscription(
            JointState,
            '/inspire_hand/state/right_hand',
            self.right_hand_callback,
            10  # QoS队列深度
        )
        
        print('灵巧手状态监测节点已启动，开始监听话题...')
    
    def left_hand_callback(self, msg: JointState):
        """
        左手状态回调函数
        
        参数：
            msg (JointState): 左手关节状态消息
        """
        self.display_hand_status("左手", msg)
    
    def right_hand_callback(self, msg: JointState):
        """
        右手状态回调函数
        
        参数：
            msg (JointState): 右手关节状态消息
        """
        self.display_hand_status("右手", msg)
    
    def display_hand_status(self, hand_name: str, msg: JointState):
        """
        显示手部状态信息
        
        参数：
            hand_name (str): 手的名称（"左手"或"右手"）
            msg (JointState): 关节状态消息
        """
        # 获取消息的时间戳
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        # 输出分隔线
        print("\n" + "=" * 80)
        print(f"【{hand_name}状态信息】")
        print(f"时间戳: {timestamp}")
        print(f"帧ID: {msg.header.frame_id}")
        print("-" * 80)
        
        # 确保数据长度一致
        num_joints = len(msg.name)
        
        if num_joints == 0:
            print("警告：未接收到关节数据！")
            return
        
        # 输出各关节的详细信息
        print(f"{'关节ID':<8} {'位置(%)':<12} {'速度(%)':<12} {'力反馈(%)':<12} {'关节名称'}")
        print("-" * 80)
        
        for i in range(num_joints):

            # 获取关节ID（从数组索引推断，索引0对应ID1）
            joint_id = msg.name[i]
            joint_name = self.JOINT_NAMES.get(joint_id, f"未知关节{i}")
            
            # 获取位置百分比
            position = msg.position[i] if i < len(msg.position) else 0.0
            
            # 获取速度百分比
            velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
            
            # 获取力反馈百分比
            effort = msg.effort[i] if i < len(msg.effort) else 0.0
            
            # 格式化输出
            print(f"{joint_id:<4} {position:>10.2f}% {velocity:>12.2f}% {effort:>12.2f}% {joint_name:>10}")
        
        print("=" * 80)


def main(args=None):
    """
    主函数 - ROS2节点入口点
    
    参数：
        args: ROS2命令行参数
    """
    # 初始化ROS2 Python客户端库
    rclpy.init(args=args)
    
    # 创建节点实例
    node = InspireHandStatusMonitor()
    
    try:
        # 运行节点，保持监听
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获Ctrl+C中断信号
        node.get_logger().info("\n程序被用户中断")
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
