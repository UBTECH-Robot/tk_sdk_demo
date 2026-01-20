import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

'''
文档写的比较模糊：
以字符串形式发送关节的id实现标零；
双臂关节id：
左臂：11---14,
右臂：21---24；
'''

# 现象：实测这个话题应该是将手臂对应的电机当前的角度设置为回零状态的角度
class ArmZeroPublisher(Node):
    def __init__(self):
        super().__init__('arm_zero_publisher')
        self.publisher = self.create_publisher(String, '/arm/cmd_set_zero', 10)

    def send_zero_commands(self):
        joint_ids = [11, 12, 13, 14, 21, 22, 23, 24]
        for joint_id in joint_ids:
            msg = String()
            msg.data = str(joint_id)
            self.publisher.publish(msg)
            self.get_logger().info(f'发送标零指令: 关节 {joint_id}')
            time.sleep(0.5)  # 可适当增加延迟，避免命令处理拥堵


def main(args=None):
    rclpy.init(args=args)
    node = ArmZeroPublisher()
    node.send_zero_commands()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
