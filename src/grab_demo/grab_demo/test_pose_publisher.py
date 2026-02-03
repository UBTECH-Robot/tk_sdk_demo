#!/usr/bin/env python3
"""
测试 PoseStamped 发布者脚本
持续发布一个固定的PoseStamped消息到 /test_pose 话题

使用方法：
    python3 test_pose_publisher.py --x 1.0 --y 2.0 --z 3.0 --frame_id L_base_link
    python3 test_pose_publisher.py -x 1.0 -y 2.0 -z 3.0 -f pelvis
    python3 test_pose_publisher.py -x 0.5 -y 1.0 -z 1.5 -f pelvis

按下 Ctrl+C 停止发布
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys
import argparse


class TestPosePublisher(Node):
    def __init__(self, x, y, z, frame_id):
        super().__init__("test_pose_publisher")
        
        # 保存位置和坐标系信息
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.frame_id = str(frame_id)
        
        # 创建发布者
        self.pub = self.create_publisher(PoseStamped, "/test_pose", 10)
        
        # 创建定时器，每100ms发布一次（10Hz）
        self.timer = self.create_timer(0.1, self.publish_pose)
        
        self.get_logger().info(
            f"Test Pose Publisher started\n"
            f"  Position: ({self.x:.3f}, {self.y:.3f}, {self.z:.3f})\n"
            f"  Frame ID: {self.frame_id}\n"
            f"  Publishing to: /test_pose\n"
            f"  Press Ctrl+C to stop"
        )
    
    def publish_pose(self):
        """发布PoseStamped消息"""
        pose_stamped = PoseStamped()
        
        # 设置时间戳和坐标系
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.frame_id
        
        # 设置位置
        pose_stamped.pose.position.x = self.x
        pose_stamped.pose.position.y = self.y
        pose_stamped.pose.position.z = self.z
        
        # 设置方向（单位四元数，无旋转）
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        
        # 发布消息
        self.pub.publish(pose_stamped)
        
        # 打印发布信息（每10次打印一次，避免日志过多）
        # 注释掉以减少输出
        # self.get_logger().info(
        #     f"Published: ({self.x:.3f}, {self.y:.3f}, {self.z:.3f}) in {self.frame_id}"
        # )


def main():
    # 命令行参数解析
    parser = argparse.ArgumentParser(
        description="Test PoseStamped publisher for ROS2",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 test_pose_publisher.py --x 1.0 --y 2.0 --z 3.0 --frame_id L_base_link
  python3 test_pose_publisher.py -x 0.5 -y 0.5 -z 1.5 -f pelvis
  python3 test_pose_publisher.py -x 0 -y 0 -z 1 -f ob_camera_head_depth_optical_frame
        """
    )
    
    parser.add_argument(
        '--x', '-x',
        type=float,
        required=False,
        default=1.0,
        help='X coordinate (meters)'
    )
    parser.add_argument(
        '--y', '-y',
        type=float,
        required=False,
        default=2.0,
        help='Y coordinate (meters)'
    )
    parser.add_argument(
        '--z', '-z',
        type=float,
        required=False,
        default=3.0,
        help='Z coordinate (meters)'
    )
    parser.add_argument(
        '--frame_id', '-f',
        type=str,
        required=False,
        default="pelvis",
        help='Frame ID (e.g., L_base_link, pelvis, ob_camera_head_depth_optical_frame)'
    )
    
    args = parser.parse_args()
    
    # 初始化 ROS2
    rclpy.init()
    
    # 创建节点
    node = TestPosePublisher(args.x, args.y, args.z, args.frame_id)
    
    try:
        # 运行节点
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n✓ User interrupted. Shutting down...")
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()
        print("✓ Test Pose Publisher stopped")


if __name__ == "__main__":
    main()
