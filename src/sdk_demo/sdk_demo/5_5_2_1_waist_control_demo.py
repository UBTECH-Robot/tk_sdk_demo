#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
import sys

class WaistMotorController(Node):
    def __init__(self):
        super().__init__('waist_motor_controller')

        # 创建Publisher
        self.publisher = self.create_publisher(
            CmdSetMotorPosition,
            '/waist/cmd_pos',
            10
        )

        self.angular = sys.argv[1] if len(sys.argv) > 1 else 0.25

        # 定时器：1秒发送一次命令
        # self.timer = self.create_timer(1.0, self.send_command)
        self.get_logger().info("已启动腰部位置控制节点，定期发送控制指令。")

    def send_command(self):
        msg = CmdSetMotorPosition()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        cmd = SetMotorPosition()
        cmd.name = 31            # 电机编号（腰部电机ID为31）
        cmd.pos = float(self.angular)   
        cmd.spd = 0.2
        cmd.cur = 5.0 

        msg.cmds.append(cmd)
        self.publisher.publish(msg)
        self.get_logger().info(
            f"发送位置命令: ID={cmd.name}, 位置={cmd.pos:.2f}rad, 速度={cmd.spd:.1f}rpm, 电流限制={cmd.cur:.1f}A"
        )

def main(args=None):
    rclpy.init(args=args)
    node = WaistMotorController()
    try:
        node.send_command()  # 直接发送一次命令
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
