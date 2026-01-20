import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorCurTor, SetMotorCurTor

'''
关节的电流控制接口，需要提供期望电流。
'''

# 现象：
# 1，current_value设置为0.5时，启动此节点直接报错：AssertionError: The 'cur_tor' field must be of type 'int'
class ArmCurrentPublisher(Node):
    def __init__(self):
        super().__init__('arm_current_publisher')
        self.publisher = self.create_publisher(CmdSetMotorCurTor, '/arm/cmd_current', 10)
        self.timer = self.create_timer(1.0, self.publish_current_command)  # 每秒发一次

    def publish_current_command(self):
        msg = CmdSetMotorCurTor()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        # ✅ 示例：控制左臂的 4 个关节（11~14）
        joint_ids = [11]#, 12, 13, 14]
        current_value = 1  # 单位依驱动器定义，常为安培（A）

        for jid in joint_ids:
            cmd = SetMotorCurTor()
            cmd.name = jid
            cmd.cur_tor = current_value
            cmd.ctrl_status = 1  # 电流控制模式
            msg.cmds.append(cmd)

        self.publisher.publish(msg)
        self.get_logger().info(f'✅ 已发送 {len(msg.cmds)} 个电流控制命令')


def main(args=None):
    rclpy.init(args=args)
    node = ArmCurrentPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🔚 用户终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
