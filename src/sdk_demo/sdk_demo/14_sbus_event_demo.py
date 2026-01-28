# 运行方式：
#    ros2 run sdk_demo sbus_event_demo

import rclpy
from rclpy.node import Node

from bodyctrl_msgs.msg import SbusData

class SbusEventListener(Node):
    def __init__(self):
        super().__init__('sbus_event_listener')
        self.subscription = self.create_subscription(
            SbusData,
            '/sbus_data/event',
            self.listener_callback,
            10
        )
        self.subscription  # 防止被回收

        # 映射按键事件常量（与 msg 定义一致）
        self.key_event_map = {
            0: "无",
            1: "A键抬起",
            2: "A键按下",
            3: "B键抬起",
            4: "B键按下",
            5: "C键抬起",
            6: "C键按下",
            7: "D键抬起",
            8: "D键按下",
            9: "E键上拨",
            10: "E键回中",
            11: "E键下拨",
            12: "F键上拨",
            13: "F键回中",
            14: "F键下拨",
            15: "G键左拨",
            16: "G键回中",
            17: "G键右拨",
            18: "H键左拨",
            19: "H键回中",
            20: "H键右拨"
        }

    def listener_callback(self, msg: SbusData):
        print("🎮 [SBUS 控制事件]")

        print(f"  🆕 当前按键事件: {self.key_event_map.get(msg.key_event_new, f'未知({msg.key_event_new})')}")
        print(f"  🕒 上次按键事件: {self.key_event_map.get(msg.key_event_old, f'未知({msg.key_event_old})')}")

        def key_state(val):
            return '✅ 按下' if val == 1 else '❌ 松开'

        def three_state(val, labels):
            if val < -0.5:
                return labels[0]
            elif val > 0.5:
                return labels[2]
            else:
                return labels[1]

        print("\n🔘 按键状态：")
        print(f"  A: {key_state(msg.button_a)}")
        print(f"  B: {key_state(msg.button_b)}")
        print(f"  C: {key_state(msg.button_c)}")
        print(f"  D: {key_state(msg.button_d)}")
        print(f"  E: {three_state(msg.button_e, ['⬆ 上拨', '⏺ 中间', '⬇ 下拨'])}")
        print(f"  F: {three_state(msg.button_f, ['⬆ 上拨', '⏺ 中间', '⬇ 下拨'])}")
        print(f"  G: {three_state(msg.button_g, ['⬅ 左拨', '⏺ 中间', '➡ 右拨'])}")
        print(f"  H: {three_state(msg.button_h, ['➡ 右拨', '⏺ 中间', '⬅ 左拨'])}")

        print("\n🕹 摇杆状态（范围 -1.0 ~ 1.0）：")
        print(f"  左摇杆: X={msg.x1:.2f}, Y={msg.y1:.2f}")
        print(f"  右摇杆: X={msg.x2:.2f}, Y={msg.y2:.2f}")
        print("-" * 50)


def main(args=None):
    rclpy.init(args=args)
    node = SbusEventListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 已手动退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
