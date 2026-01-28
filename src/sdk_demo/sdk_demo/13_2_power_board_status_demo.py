# 运行方式：
#    ros2 run sdk_demo power_board_status_demo

import rclpy
from rclpy.node import Node

from bodyctrl_msgs.msg import PowerStatus  # 自定义消息类型


class PowerBoardStatusListener(Node):
    def __init__(self):
        super().__init__('power_board_status_listener')
        self.subscription = self.create_subscription(
            PowerStatus,
            '/power/board/status',
            self.listener_callback,
            10  # QoS depth
        )
        self.subscription  # 防止被垃圾回收

    def listener_callback(self, msg: PowerStatus):
        self.get_logger().info("📡 接收到电源板状态数据：\n")

        print("🧯 [温度信息] 当前值 / 最大值 / 最小值 (单位: °C)")
        print(f"  腰部: {msg.waist_temp:.1f} / {msg.waist_temp_max:.1f} / {msg.waist_temp_min:.1f}")
        print(f"  臂 A: {msg.arm_a_temp:.1f} / {msg.arm_a_temp_max:.1f} / {msg.arm_a_temp_min:.1f}")
        print(f"  臂 B: {msg.arm_b_temp:.1f} / {msg.arm_b_temp_max:.1f} / {msg.arm_b_temp_min:.1f}")
        print(f"  腿 A: {msg.leg_a_temp:.1f} / {msg.leg_a_temp_max:.1f} / {msg.leg_a_temp_min:.1f}")
        print(f"  腿 B: {msg.leg_b_temp:.1f} / {msg.leg_b_temp_max:.1f} / {msg.leg_b_temp_min:.1f}")

        print("\n🔌 [电流信息] 当前值 / 最大值 / 最小值 (单位: A)")
        print(f"  腰部: {msg.waist_curr:.2f} / {msg.waist_curr_max:.2f} / {msg.waist_curr_min:.2f}")
        print(f"  臂 A: {msg.arm_a_curr:.2f} / {msg.arm_a_curr_max:.2f} / {msg.arm_a_curr_min:.2f}")
        print(f"  臂 B: {msg.arm_b_curr:.2f} / {msg.arm_b_curr_max:.2f} / {msg.arm_b_curr_min:.2f}")
        print(f"  腿 A: {msg.leg_a_curr:.2f} / {msg.leg_a_curr_max:.2f} / {msg.leg_a_curr_min:.2f}")
        print(f"  腿 B: {msg.leg_b_curr:.2f} / {msg.leg_b_curr_max:.2f} / {msg.leg_b_curr_min:.2f}")

        print("\n⚡ [电压信息] 当前值 / 最大值 / 最小值 (单位: V)")
        print(f"  腰部: {msg.waist_volt:.2f} / {msg.waist_volt_max:.2f} / {msg.waist_volt_min:.2f}")
        print(f"  臂 A: {msg.arm_a_volt:.2f} / {msg.arm_a_volt_max:.2f} / {msg.arm_a_volt_min:.2f}")
        print(f"  臂 B: {msg.arm_b_volt:.2f} / {msg.arm_b_volt_max:.2f} / {msg.arm_b_volt_min:.2f}")
        print(f"  腿 A: {msg.leg_a_volt:.2f} / {msg.leg_a_volt_max:.2f} / {msg.leg_a_volt_min:.2f}")
        print(f"  腿 B: {msg.leg_b_volt:.2f} / {msg.leg_b_volt_max:.2f} / {msg.leg_b_volt_min:.2f}")
        print(f"  母线: {msg.bus_volt:.2f} / {msg.bus_volt_max:.2f} / {msg.bus_volt_min:.2f}")

        print("\n🔋 [电池信息] (单位：V / A / %)")
        print(f"  电压: {msg.battery_voltage:.2f} V")
        print(f"  电流: {msg.battery_current:.2f} A")
        print(f"  电量: {msg.battery_power:.2f} %")

        print("\n🧾 [版本信息]")
        print(f"  软件版本: {msg.software_version}")
        print(f"  硬件版本: {msg.hardware_version}")
        print("-" * 50)


def main(args=None):
    rclpy.init(args=args)
    node = PowerBoardStatusListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🔚 程序已退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
