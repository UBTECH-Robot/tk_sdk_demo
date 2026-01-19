import rclpy
from rclpy.node import Node

from bodyctrl_msgs.msg import PowerBoardKeyStatus  # 自定义消息
from std_msgs.msg import Bool                     # 用于布尔字段


class PowerKeyStatusListener(Node):
    def __init__(self):
        super().__init__('power_key_status_listener')
        self.subscription = self.create_subscription(
            PowerBoardKeyStatus,
            '/power/board/key_status',
            self.listener_callback,
            10  # QoS depth
        )
        self.subscription  # 防止被回收

    def listener_callback(self, msg: PowerBoardKeyStatus):
        self.get_logger().info('🔑 接收到电源按键状态：')

        work_time_sec = msg.work_time
        hours = work_time_sec // 3600
        minutes = (work_time_sec % 3600) // 60
        seconds = work_time_sec % 60

        def status_str(bool_msg):
            return '✅ 是' if bool_msg.data else '❌ 否'

        print(f"⏱ 工作时间: {work_time_sec} 秒 ({hours} 小时 {minutes} 分钟 {seconds} 秒)")
        print(f"🛑 急停是否按下: {status_str(msg.is_estop)}")
        print(f"📡 软急停是否按下: {status_str(msg.is_remote_estop)}")
        print(f"🔌 电源是否供电正常: {status_str(msg.is_power_on)}")
        print("-" * 50)


def main(args=None):
    rclpy.init(args=args)
    node = PowerKeyStatusListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 节点已手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
