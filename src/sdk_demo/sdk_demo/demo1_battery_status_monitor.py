import rclpy
from rclpy.node import Node
from bodyctrl_msgs.msg import PowerBatteryStatus
from std_msgs.msg import String
import os
import time
import json

class BatteryStatusMonitor(Node):
    def __init__(self):
        super().__init__('battery_status_monitor')
        self.subscription = self.create_subscription(
            PowerBatteryStatus,
            '/power/battery/status',
            self.listener_callback,
            10  # QoS
        )
        self.remind_gap = int(os.environ.get("REMIND_GAP", "30"))  # 环境变量设置提醒间隔，默认为30秒
        AUDIO_DIR = os.environ.get("AUDIO_DIR", "/home/nvidia/data/speech/low_power_audio_res") # 注意这个目录是orin板上音频文件存放的目录，需要先把对应的音频文件放到这个目录下
        self.audio_dict = {
            "1": os.path.join(AUDIO_DIR, "power_lt_1.mp3"),
            "2": os.path.join(AUDIO_DIR, "power_lt_2.mp3"),
            "5": os.path.join(AUDIO_DIR, "power_lt_5.mp3"),
            "10": os.path.join(AUDIO_DIR, "power_lt_10.mp3"),
            "20": os.path.join(AUDIO_DIR, "power_lt_20.mp3"),
        }
        self.subscription  # 防止变量被垃圾回收

        self.publisher = self.create_publisher(String, '/xunfei/tts_play', 10)
        
        self.get_logger().info('Battery Status Monitor 已启动，等待 /power/battery/status 数据...')

        self.last_publish_time = 0  # 初始化上次发布的时间

    def try_publish_tts_message(self, master_battery_power):
        current_time = time.time()  # 获取当前时间（单位：秒）

        # 如果距离上次发布超过 30 秒，则发布新消息，也就是每 30 秒播报一次提示语音，这个时间间隔可以通过环境变量 REMIND_GAP 来设置
        if current_time - self.last_publish_time <= self.remind_gap or master_battery_power >= 20:
            return

        self.last_publish_time = current_time

        filepath = self.audio_dict.get(str(20), None)
        if master_battery_power < 1:
            filepath = self.audio_dict.get(str(1), None)
        elif master_battery_power < 2:
            filepath = self.audio_dict.get(str(2), None)
        elif master_battery_power < 5:
            filepath = self.audio_dict.get(str(5), None)
        elif master_battery_power < 10:
            filepath = self.audio_dict.get(str(10), None)
        elif master_battery_power < 20:
            filepath = self.audio_dict.get(str(20), None)

        payload = {
            "file": filepath
        }
        msg = String()
        msg.data = json.dumps(payload)

        self.publisher.publish(msg)
        self.get_logger().info('Published message: "%s"' % msg.data)

    def listener_callback(self, msg: PowerBatteryStatus):
        self.get_logger().info('接收到电池状态信息：')

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

        self.get_logger().info(f'🔋 电池安装状态: {installed}')
        self.get_logger().info(f'⚡ 电池工作状态: {working}')
        self.get_logger().info(f'📈 主电池电压: {msg.master_battery_voltage:.2f} V')
        self.get_logger().info(f'📉 主电池电流: {msg.master_battery_current:.2f} A')
        self.get_logger().info(f'🔋 主电池电量: {msg.master_battery_power:.2f} %')
        self.get_logger().info(f'📈 小电池电压: {msg.little_battery_voltage:.2f} V')
        self.get_logger().info(f'📉 小电池电流: {msg.little_battery_current:.2f} A')
        self.get_logger().info(f'🔋 小电池电量: {msg.little_battery_power:.2f} %')
        
        if msg.master_battery_power < 20:
            self.get_logger().info(f'🔋 主电池电量小于20%: {msg.master_battery_power}')

            # 需要先启动orin板上的如下服务
            # . ~/voice_ws/install/setup.bash
            # ros2 launch xunfei_dev_socket xunfei_dev_all.launch.py
            # 然后可以使用以下命令播放音频
            # ros2 topic pub /xunfei/tts_play std_msgs/msg/String "{data: '{\"file\": \"/home/nvidia/data/speech/chenggong.mp3\"}'}"

            self.try_publish_tts_message(msg.master_battery_power)

        self.get_logger().info("-" * 40)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
