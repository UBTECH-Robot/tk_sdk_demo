import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdMotorCtrl, MotorCtrl, MotorStatusMsg
import math

'''
“力位混合控制”是一种**同时考虑目标位置（或速度）和施加的力（或力矩）**来控制机械臂运动的方式。

这类控制模型常见于：

人机交互（如人推动机器人手臂）

接触场景（如机器人打磨、拧螺丝、擦窗户）

柔顺控制（如牵引教学或柔软碰撞）

🚩 核心思想：
使用 kp 和 kd 控制误差（位置、速度） → 类似弹簧阻尼系统

使用 tor 提供额外的控制“助力” → 类似人拉一下弹簧的感觉

👉 举个例子：
机器人末端要贴合墙面运动。如果你只用位置控制，它会“硬推”墙面，造成冲击；但若用力位混合控制，它可以“贴着墙”轻柔滑动。
没有动力学模型或实时传感器估计力矩，tor 就设置为 0，靠 kp/kd 自己调试轨迹即可

✅ 一、先做刚性位置控制（用于调 kp/kd）
当你不清楚参数时，建议先设置 spd = 0, tor = 0，从位置控制做起：

参数	初始建议值	含义
kp	10~50	较大的值表示“刚”，适用于精准控制
kd	0.5~5.0	起阻尼作用，防止震荡
pos	期望位置（单位 rad）	例如 π/2 是 90°
spd	0.0	暂不使用速度控制
tor	0.0	暂不使用前馈力矩

🛠 调试方法：

从 kp=10 开始，逐步增大（例如 10 → 20 → 30）

每次只调一个关节，观察其响应：是否运动平稳，是否震荡，是否跟随。

若出现振荡，增大 kd 抑制。

✅ 二、添加速度控制（柔顺控制）
启用 spd 参数，代表目标动态行为。此时 kd 的重要性更大。

参数	建议
kp	5~15（降低）
kd	1~5
spd	目标速度（rad/s），例如 0.5
tor	保持为 0.0

这时关节会更“软”，像是橡皮筋一样随动 —— 比如人轻轻推一下，它会动一点。

✅ 三、添加前馈力矩 tor
这个参数与系统建模有关，例如你知道某个关节因重力需要抵消 0.8 N·m 力矩，就可以直接设置：

text
复制
编辑
tor = 0.8
🧠 实用技巧：

如果机械臂在目标位置悬空，但电机会缓慢“下垂”，说明你可以试试加一个正向 tor 抵消重力。

不知道具体值时，从 tor=0.2 开始试起。

✅ 四、模拟一个稳定组合参考值
text
复制
编辑
kp = 30.0
kd = 2.0
pos = 1.57    # 目标 90°
spd = 0.0
tor = 0.0
如发现系统响应太“猛”，就降 kp；如震荡，升 kd；如下垂，用 tor 抵消。

📌 总结调参顺序建议
pos 控制（设定目标位置）

调 kp 看是否能跟上、是否过猛

再调 kd 稳定系统

若还需要自然的动态行为，再引入 spd

最后考虑加入 tor（前馈力矩）

'''

# 现象：
# 可以正常摆右臂
MOTOR_ARM_RIGHT_1=21
MOTOR_ARM_RIGHT_2=22

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.initialized = False
        self.angle_step = math.radians(3)  # 每步增加5度
        self.angle2 = math.radians(-30) # 右臂第二电机角度，注意负数是向外
        self.max_angle = math.radians(60)  # 最大 ±60度
        self.publisher_ = self.create_publisher(CmdMotorCtrl, '/arm/cmd_ctrl', 10)

        self.subscription = self.create_subscription(
            MotorStatusMsg,
            "/arm/status",
            self.status_callback,
            10
        )
        self.timer = None
        self.step = 0
        self.direction = 1
        
    
    def calculate_current_step_and_init(self, pos):
        self.step = int(pos / self.angle_step)
        self.get_logger().info(f"Init step calculated: {self.step}, direction: {self.direction}, pos: {pos:.3f} rad")

        if self.timer is None:
            self.timer = self.create_timer(0.1, self.send_ctrl_msg)  # 保守：每 1 秒发布一次

    def status_callback(self, msg: MotorStatusMsg):
        if self.initialized:
            return
        self.initialized = True
        for i, status in enumerate(msg.status):
            if status.name == MOTOR_ARM_RIGHT_1:
                self.calculate_current_step_and_init(status.pos)
                break


    def send_ctrl_msg(self):
        self.step += 1 * self.direction
        pos = self.step * self.angle_step
        if abs(pos) > self.max_angle:
            self.direction *= -1  # 超过最大角度则反向
            self.step += 2 * self.direction
            pos = self.step * self.angle_step

        msg = CmdMotorCtrl()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        motor = MotorCtrl()
        motor.name = MOTOR_ARM_RIGHT_1 # 右臂第一电机ID
        motor.kp = 10.0  # 保守设置
        motor.kd = 1.0
        motor.pos = pos # 负数是向前，正数是向后
        motor.spd = 0.0
        motor.tor = 0.0

        motor2 = MotorCtrl()
        motor2.name = MOTOR_ARM_RIGHT_2 # 右臂第二电机ID
        motor2.kp = 10.0  # 保守设置
        motor2.kd = 1.0
        motor2.pos = self.angle2 # 固定向外张开30度，以免右臂运动的时候打到机器人自己的大腿，负数是向外
        motor2.spd = 0.0
        motor2.tor = 0.0

        msg.cmds = [motor, motor2]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Step {self.step}: Publishing pos={pos:.3f} rad, angle2={self.angle2:.3f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
