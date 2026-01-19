#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bodyctrl_msgs.msg import MotorStatusMsg, MotorStatus

class MotorStatusMonitor(Node):
    def __init__(self):
        super().__init__('motor_status_monitor')
        self.topic_name = '/waist/status'
        self.subscription = self.create_subscription(
            MotorStatusMsg,
            self.topic_name,
            self.status_callback,
            10
        )
        self.get_logger().info(f"已订阅话题：{self.topic_name}")

    def status_callback(self, msg: MotorStatusMsg):
        for i, status in enumerate(msg.status):
            self.get_logger().info(f"电机 {i}:")
            self.get_logger().info(f"  名称: {status.name}")
            self.get_logger().info(f"  位置(rad): {status.pos:.3f}")
            self.get_logger().info(f"  速度(rad/s): {status.speed:.3f}")
            self.get_logger().info(f"  电流(A): {status.current:.2f}")
            self.get_logger().info(f"  温度(℃): {status.temperature:.1f}")
            self.get_logger().info(f"  错误码: {status.error}")
            self.parse_error_code(status.error)

    def parse_error_code(self, error_code: int):
        if error_code == 0:
            self.get_logger().info("  无错误")
            return

        known_errors = {
            1: "电机过温",
            2: "过流",
            3: "电压过低",
            4: "MOS 过温",
            5: "堵转",
            6: "电压过高",
            7: "缺相",
            8: "编码器错误",
            33072: "设备掉线",
            33073: "关节位置超限"
        }

        if error_code in known_errors:
            self.get_logger().warn(f"  错误说明: {known_errors[error_code]}")
        else:
            self.get_logger().warn("  错误说明: 未知错误码")

def main(args=None):
    rclpy.init(args=args)
    node = MotorStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
