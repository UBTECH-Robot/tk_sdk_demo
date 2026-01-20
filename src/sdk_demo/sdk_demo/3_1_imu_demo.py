#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',           
            self.listener_callback,
            10
        )
        self.get_logger().info('IMU Subscriber 已启动，等待 /imu 数据...')

    def listener_callback(self, msg):
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_accel = msg.linear_acceleration

        self.get_logger().info(
            f'Orientation: x={orientation.x:.3f}, y={orientation.y:.3f}, z={orientation.z:.3f}, w={orientation.w:.3f}\n'
            f'Angular Vel: x={angular_velocity.x:.3f}, y={angular_velocity.y:.3f}, z={angular_velocity.z:.3f}\n'
            f'Linear Accel: x={linear_accel.x:.3f}, y={linear_accel.y:.3f}, z={linear_accel.z:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
