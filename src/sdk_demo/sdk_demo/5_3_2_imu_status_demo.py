#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# 导入自定义消息体
from bodyctrl_msgs.msg import Imu

class ImuStatusSubscriber(Node):
    def __init__(self):
        super().__init__('imu_status_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/status',
            self.listener_callback,
            10)
        self.get_logger().info('订阅 /imu/status 话题已启动')

    def listener_callback(self, msg: Imu):
        self.get_logger().info(f'header.stamp: sec={msg.header.stamp.sec}, nanosec={msg.header.stamp.nanosec}')
        
        ori = msg.orientation
        self.get_logger().info(f'Orientation: x={ori.x:.3f}, y={ori.y:.3f}, z={ori.z:.3f}, w={ori.w:.3f}')

        ang_vel = msg.angular_velocity
        self.get_logger().info(f'Angular velocity: x={ang_vel.x:.3f}, y={ang_vel.y:.3f}, z={ang_vel.z:.3f}')

        lin_acc = msg.linear_acceleration
        self.get_logger().info(f'Linear acceleration: x={lin_acc.x:.3f}, y={lin_acc.y:.3f}, z={lin_acc.z:.3f}')

        euler = msg.euler
        self.get_logger().info(f'Euler angles: roll={euler.roll:.3f}, pitch={euler.pitch:.3f}, yaw={euler.yaw:.3f}')

        self.get_logger().info(f'Error code: {msg.error}')

        # 协方差数组是float64[3]
        self.get_logger().info(f'Angular velocity covariance: {list(msg.angular_velocity_covariance)}')
        self.get_logger().info(f'Orientation covariance: {list(msg.orientation_covariance)}')
        self.get_logger().info(f'Linear acceleration covariance: {list(msg.linear_acceleration_covariance)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImuStatusSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
