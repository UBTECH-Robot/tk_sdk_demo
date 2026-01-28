# 运行方式：
#    ros2 run sdk_demo battery_status_demo

import rclpy
from rclpy.node import Node
from bodyctrl_msgs.msg import PowerBatteryStatus  # 自定义消息类型
from std_msgs.msg import Header


class BatteryStatusListener(Node):
    def __init__(self):
        super().__init__('battery_status_listener')
        self.subscription = self.create_subscription(
            PowerBatteryStatus,
            '/power/battery/status',
            self.listener_callback,
            10  # QoS
        )
        self.subscription  # 防止变量被垃圾回收

    def listener_callback(self, msg: PowerBatteryStatus):
        self.get_logger().info('🪫 接收到电池状态信息：')

        # 电池安装状态
        installed_map = {
            0x00: '无电池',
            0x01: '小电池存在',
            0x02: '大电池存在',
            0x03: '大小电池都存在'
        }
        working_map = {
            0x01: '小电池工作',
            0x10: '大电池工作',
        }

        installed = installed_map.get(msg.battery_installed, f"未知 ({msg.battery_installed})")
        working = working_map.get(msg.battery_working, f"未知 ({msg.battery_working})")

        print(f'🔋 电池安装状态: {installed}')
        print(f'⚡ 电池工作状态: {working}')
        print(f'📈 主电池电压: {msg.master_battery_voltage:.2f} V')
        print(f'📉 主电池电流: {msg.master_battery_current:.2f} A')
        print(f'🔋 主电池电量: {msg.master_battery_power:.2f} %')
        print(f'📈 小电池电压: {msg.little_battery_voltage:.2f} V')
        print(f'📉 小电池电流: {msg.little_battery_current:.2f} A')
        print(f'🔋 小电池电量: {msg.little_battery_power:.2f} %')

        def digital_status(value):
            return '高电平' if value == 1 else '低电平'

        # 数字输入状态
        print(f'🧩 PG12A: {digital_status(msg.pg12a)}')
        print(f'🧩 PG12B: {digital_status(msg.pg12b)}')
        print(f'🧩 PG12C: {digital_status(msg.pg12c)}')
        print(f'🧩 PG12D: {digital_status(msg.pg12d)}')
        print(f'🧩 PG5CD: {digital_status(msg.pg5cd)}')
        print(f'🧩 PG5AB: {digital_status(msg.pg5ab)}')
        print(f'🧩 PGRDC2: {digital_status(msg.pgrdc2)}')
        print(f'🧩 PGRDC1: {digital_status(msg.pgrdc1)}')
        print(f'🧩 PGHeader: {digital_status(msg.pgheader)}')
        print(f'🧩 PGButton2: {digital_status(msg.pgbutton2)}')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryStatusListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
