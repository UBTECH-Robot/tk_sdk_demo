import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorSpeed, SetMotorSpeed

'''
关节的速度控制接口，需要提供期望速度、最大电流。
'''

# 现象：
# 左臂第一个电机可以正常转动，其他未尝试
class ArmVelocityPublisher(Node):
    def __init__(self):
        super().__init__('arm_velocity_publisher')
        self.publisher = self.create_publisher(CmdSetMotorSpeed, '/arm/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.send_velocity_command)

    def send_velocity_command(self):
        msg = CmdSetMotorSpeed()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        # 示例：让左臂四个关节（11~14）以正方向转动
        joint_ids = [11]#, 12, 13, 14]
        speed_rpm = 5.0       # 目标转速，单位为 RPM（正为正转，负为反转）
        limit_current = 1.0    # 限制最大电流，单位为 A

        for jid in joint_ids:
            cmd = SetMotorSpeed()
            cmd.name = jid
            cmd.spd = speed_rpm
            cmd.cur = limit_current
            msg.cmds.append(cmd)

        self.publisher.publish(msg)
        self.get_logger().info(f'✅ 已发送 {len(msg.cmds)} 个速度控制命令')


def main(args=None):
    rclpy.init(args=args)
    node = ArmVelocityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🔚 已终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
