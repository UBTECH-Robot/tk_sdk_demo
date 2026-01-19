
机型：TG2.0-Pro
版本：v2.0.5.1


机身高度和姿态控制接口
/hric/robot/float_base_rpyz_cmd,（但是示例命令里写的是这个）

位置调整到0.88：
ros2 topic pub /hric/robot/float_base_rpyz_cmd hric_msgs/msg/FloatBaseRPYZ "{ roll: 0.0, pitch: 0.0, yaw: 0, z: 0.88 }"
✅

腰身左右横滚，r正值是右横滚：
ros2 topic pub /hric/robot/float_base_rpyz_cmd hric_msgs/msg/FloatBaseRPYZ "{ roll: 0.02, pitch: 0.0, yaw: 0, z: 0.88 }"
ros2 topic pub /hric/robot/float_base_rpyz_cmd hric_msgs/msg/FloatBaseRPYZ "{ roll: -0.02, pitch: 0.0, yaw: 0, z: 0.88 }"
✅

前后俯仰，p正值是前俯：
ros2 topic pub /hric/robot/float_base_rpyz_cmd hric_msgs/msg/FloatBaseRPYZ "{ roll: 0.0, pitch: 0.05, yaw: 0, z: 0.88 }"
ros2 topic pub /hric/robot/float_base_rpyz_cmd hric_msgs/msg/FloatBaseRPYZ "{ roll: 0.0, pitch: -0.05, yaw: 0, z: 0.88 }"
✅

腰身左右偏转，y正值是左转：
ros2 topic pub /hric/robot/float_base_rpyz_cmd hric_msgs/msg/FloatBaseRPYZ "{ roll: 0.0, pitch: 0.0, yaw: 0.03, z: 0.88 }"
ros2 topic pub /hric/robot/float_base_rpyz_cmd hric_msgs/msg/FloatBaseRPYZ "{ roll: 0.0, pitch: 0.0, yaw: -0.03, z: 0.88 }"
✅


机身高度和姿态状态反馈接口
命令：
ros2 topic echo /hric/robot/float_base_rpyz_status
✅

动作状态反馈接口
命令：
ros2 topic echo /hric/robot/action_status
✅

x 方向速度限制状态反馈接口
命令：
ros2 topic echo /hric/robot/get_vel_limit
✅

x 方向速度限制设置接口
命令：
ros2 topic pub /hric/robot/set_vel_limit hric_msgs/msg/SetVelLimit "{ x_vel_limit_walk: 0.5, x_vel_limit_run: 1.5 }"
✅


天工Pro，2.0.5.1
F置中后，H左拨进入原地踏步，此时手臂会在前面上下摆动，符合期望；
F下拨后，H左拨进入原地踏步，此时手臂不动，也符合文档里 不控手臂 的描述；
F下拨并且天工在站立状态，此时用如下的手臂关节控制命令想让手臂电机转动：
ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 12, pos: 0.5, spd: 0.2, cur: 8.0 }]}"

手臂实际是不动的，不符合预期，因为文档里描述的是 此时可接遥操作，也和我们理解的 运控模块强化学习开放上肢 不符合

启动强化学习运控，遥控器操作天工进入站立状态，F下拨，长按A，E上拨，然后就可以用ros命令控制上肢了