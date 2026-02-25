#!/usr/bin/env python3
"""
IMU (Inertial Measurement Unit) 数据订阅示例程序

本程序演示如何在 ROS2 中订阅和处理 IMU 传感器数据。IMU 是惯性测量单元，
包含加速度计、陀螺仪和磁力计等传感器，用于测量物体的运动状态。

主要功能：
  - 订阅 /imu 话题的 sensor_msgs/Imu 消息
  - 解析 IMU 数据并打印各项测量值
  - 展示四元数、角速度和线性加速度的含义

IMU 传感器应用场景：
  - 机器人姿态估计和平衡
  - 运动跟踪和惯性导航
  - 防摔检测
  - 游戏控制器输入

使用方式：
  0. 确保已编译并且 ROS2 环境已正确设置(只需要在修改代码后重新执行一次)
      colcon build --packages-select sdk_demo
      source ~/sdk_demo/install/setup.bash

  1. 运行本程序，观察终端输出的 IMU 数据：
      ros2 run sdk_demo imu_demo
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
   
    def __init__(self):
        """
        初始化 IMU 订阅节点
        
        创建名为 'imu_subscriber' 的 ROS2 节点，并订阅 /imu 话题。
        消息队列深度设置为 10，即最多缓存 10 条消息。
        """
        super().__init__('imu_subscriber')
        # 创建对 /imu 话题的订阅
        # 参数说明：
        #   - Imu：消息类型，来自 sensor_msgs.msg
        #   - '/imu'：话题名称
        #   - self.listener_callback：回调函数，每收到新消息时调用一次
        #   - 10：QoS（服务质量）参数，消息队列历史深度，即最多缓存 10 条消息。
        self.subscription = self.create_subscription(
            Imu,
            '/imu',           
            self.listener_callback,
            10
        )
        self.get_logger().info('IMU Subscriber 已启动，等待 /imu 数据...')

    def listener_callback(self, msg):
        """
        IMU 消息回调函数
        
        每当接收到 /imu 话题的新消息时被调用。提取并打印 IMU 传感器数据。
        
        参数：
            msg (sensor_msgs.msg.Imu)：包含 IMU 测量数据的消息对象
                
        msg 的主要属性结构：
            - header (std_msgs/Header)：消息头，包含时间戳和坐标系信息
              * stamp (Time)：消息时间戳（秒和纳秒）
              * frame_id (str)：坐标系 ID，如 "imu"
            
            - orientation (geometry_msgs/Quaternion)：姿态四元数
              * x, y, z, w (float64)：四元数分量（单位：无）
            
            - angular_velocity (geometry_msgs/Vector3)：角速度
              * x, y, z (float64)：绕 X、Y、Z 轴的角速度（单位：弧度/秒）
            
            - linear_acceleration (geometry_msgs/Vector3)：线性加速度
              * x, y, z (float64)：X、Y、Z 方向的加速度（单位：米/秒²）
        
        ━━━━━━ 四元数基础知识 ━━━━━━
        
        四元数（Quaternion）是一种数学结构，用于表示 3D 空间中的旋转。
        由 4 个分量组成：q = (x, y, z, w)，其中 w 是标量部分，(x, y, z) 是向量部分。
        
        四元数 vs 欧拉角对比：
          
          欧拉角（Roll, Pitch, Yaw）：
            - 优点：直观易理解，人类友好
            - 缺点：会产生万向锁（Gimbal Lock）问题，插值困难
          
          四元数：
            - 优点：避免万向锁，计算高效，平滑插值
            - 缺点：不够直观，需要理解数学背景
        
        四元数的几何意义：
          
          四元数编码了一个"旋转操作"：绕着旋转轴旋转一定的角度。
          
          设：
            - (nx, ny, nz)：旋转轴的单位向量（即旋转的方向）
            - θ：旋转角度（弧度）
          
          则对应的四元数为：
            - x = sin(θ/2) * nx，旋转轴 x 分量，缩放因子是 sin(θ/2)
            - y = sin(θ/2) * ny，旋转轴 y 分量，缩放因子是 sin(θ/2)
            - z = sin(θ/2) * nz，旋转轴 z 分量，缩放因子是 sin(θ/2)
            - w = cos(θ/2)，标量部分，只与旋转角有关
          
          关键点：
            ✓ 四元数的向量部分 (x, y, z) ≠ 旋转轴，而是旋转轴乘以 sin(θ/2)
            ✓ 旋转轴信息被"编码"在这三个分量的比例中
            ✓ 旋转角信息被"编码"在 w（标量部分）和向量部分的大小中
          
          例子：
            - 若要表示"绕 Z 轴逆时针旋转 90°"：
              θ = π/2 (90°)，旋转轴 = (0, 0, 1)
              四元数 = (sin(π/4)*0, sin(π/4)*0, sin(π/4)*1, cos(π/4))
                    = (0, 0, 0.707, 0.707)
        
        实际应用：
          - 机器人姿态表示：当前机器人相对于全局坐标系的方向
          - 将四元数转换为欧拉角：可以更直观地理解滚转(Roll)、俯仰(Pitch)、偏航(Yaw)
          - 旋转向量：通过旋转轴方向和旋转角进行 3D 旋转
        """
        
        # 提取四元数（姿态信息）
        # 表示 IMU 当前的空间方向（相对于坐标系原点）
        orientation = msg.orientation
        
        # 提取角速度（旋转速率）
        # 表示 IMU 绕三个轴旋转的速率，单位为 rad/s（弧度/秒）
        # - angular_velocity.x：绕 X 轴（前后）的旋转速率（滚转 Roll 速率）
        # - angular_velocity.y：绕 Y 轴（左右）的旋转速率（俯仰 Pitch 速率）
        # - angular_velocity.z：绕 Z 轴（上下）的旋转速率（偏航 Yaw 速率）
        angular_velocity = msg.angular_velocity
        
        # 提取线性加速度
        # 表示 IMU 在三个方向上的加速度，单位为 m/s²（米/秒²）
        # 注意：包含重力加速度分量（9.8 m/s²）
        # - linear_accel.x：X 方向（前后）的加速度
        # - linear_accel.y：Y 方向（左右）的加速度  
        # - linear_accel.z：Z 方向（上下）的加速度（静止时约为 9.8 m/s²，指向上方）
        linear_accel = msg.linear_acceleration

        self.get_logger().info(
            f'\n四元数姿态 (x, y, z, w)：\n'
            f'   x={orientation.x:.4f}（绕 X 轴分量）\n'
            f'   y={orientation.y:.4f}（绕 Y 轴分量）\n'
            f'   z={orientation.z:.4f}（绕 Z 轴分量）\n'
            f'   w={orientation.w:.4f}（标量部分）'
            f'\n角速度 (rad/s)：\n'
            f'   x={angular_velocity.x:.4f}（滚转速率 Roll Rate）\n'
            f'   y={angular_velocity.y:.4f}（俯仰速率 Pitch Rate）\n'
            f'   z={angular_velocity.z:.4f}（偏航速率 Yaw Rate）'
            f'\n线性加速度 (m/s²)：\n'
            f'   x={linear_accel.x:.4f}（前后）\n'
            f'   y={linear_accel.y:.4f}（左右）\n'
            f'   z={linear_accel.z:.4f}（上下，包含重力）\n'
        )
        print('---' * 30)

def main(args=None):
    """
    主程序入口
    
    初始化 ROS2，创建 IMU 订阅节点，进入事件循环持续处理消息。
    按 Ctrl+C 可优雅地退出程序。
    """
    # 初始化 ROS2 通信系统
    rclpy.init(args=args)
    
    # 创建 IMU 订阅节点实例
    node = ImuSubscriber()
    
    try:
        # 进入阻塞式事件循环，持续处理订阅的消息
        # rclpy.spin() 会执行以下操作：
        #   1. 等待 /imu 话题的新消息
        #   2. 收到消息时调用 listener_callback 回调函数
        #   3. 循环重复 1-2，直到节点被销毁
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 时捕获中断信号
        print("\n用户中断程序")
    finally:
        # 清理资源：销毁节点、关闭 ROS2 通信
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
