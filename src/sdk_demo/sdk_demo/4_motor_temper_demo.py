#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
from bodyctrl_msgs.msg import MotorStatus1

topic1 = '/waist/motor_status'
topic2 = '/arm/motor_status'
topic3 = '/leg/motor_status'

topic_map = {"waist": topic1, "arm": topic2, "leg": topic3}

class MotorTempMonitor(Node):
    def __init__(self):
        super().__init__('waist_motor_temp_monitor')
        self.url = topic1
        topic_name = topic_map.get(sys.argv[1] if len(sys.argv) > 1 else 'waist', topic1)
        self.subscription = self.create_subscription(
            MotorStatus1,
            topic_name,
            self.motor_status_callback,
            10
        )
        self.get_logger().info(f"订阅 {topic_name} 中...")

    def motor_status_callback(self, msg):
        # msg 应该是一个 MotorStatus1[] 数组的封装消息
        # 如果该话题是 array 类型的消息，则需要确认消息定义是否是 array 包裹的
        try:
            if hasattr(msg, 'status'):
                motor_array = msg.status
                for i, motor in enumerate(motor_array):
                    self.get_logger().info(
                        f"[Motor {i}] name={motor.name}, "
                        f"motor_temp={motor.motortemperature:.1f} °C, "
                        f"mos_temp={motor.mostemperature:.1f} °C"
                    )
            else:
                # 如果话题直接是一个数组（少见，但有可能）
                self.get_logger().info("接收到电机状态数组：")
                for i, motor in enumerate(msg):
                    self.get_logger().info(
                        f"[Motor {i}] name={motor.name}, "
                        f"motor_temp={motor.motortemperature:.1f} °C, "
                        f"mos_temp={motor.mostemperature:.1f} °C"
                    )
        except Exception as e:
            self.get_logger().error(f"处理温度消息时出错: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorTempMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
