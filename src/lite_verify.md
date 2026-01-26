

向指定的电机话题发布消息：
. ~/ros2ws/install/setup.bash
位置控制：
ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{
  header: {
    stamp: { sec: 0, nanosec: 0 },
    frame_id: ''
  },
  cmds: [
    {
      name: 11,
      pos: -0.3,
      spd: 0.2,
      cur: 5.0
    }
  ]
}"

ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{
  header: {
    stamp: { sec: 0, nanosec: 0 },
    frame_id: ''
  },
  cmds: [
    {
      name: 61,
      pos: 0.0,
      spd: 0.2,
      cur: 15.0
    }
  ]
}"

力位混合控制：
ros2 topic pub /arm/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{
  header: {
    stamp: { sec: 0, nanosec: 0 },
    frame_id: ''
  },
  cmds: [
    {
      name: 24,
      kp: 30.0,
      kd: 10.0,
      pos: -0.0,
      spd: 0.0,
      tor: 0.0
    }
  ]
}"

ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{
  header: {
    stamp: { sec: 0, nanosec: 0 },
    frame_id: ''
  },
  cmds: [
    {
      name: 61,
      kp: 40.0,
      kd: 10.0,
      pos: -0.5,
      spd: 0.0,
      tor: 0.0
    }
  ]
}"


速度控制
注意这个消息一旦发布，该电机将以该速度持续转动，直到该话题收到一个spd为0.0的消息，因此需要谨慎使用
ros2 topic pub /arm/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{
  header: {
    stamp: { sec: 0, nanosec: 0 },
    frame_id: ''
  },
  cmds: [
    {
      name: 11,
      spd: -0.1,
      cur: 5.0
    }
  ]
}"

ros2 topic pub /arm/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{
  header: {
    stamp: { sec: 0, nanosec: 0 },
    frame_id: ''
  },
  cmds: [
    {
      name: 11,
      spd: 0.0,
      cur: 5.0
    }
  ]
}"

电流模式 - 电机无响应

ros2 topic pub /arm/cmd_current bodyctrl_msgs/msg/CmdSetMotorCurTor "{cmds: [{name: 11, cur_tor: 12.0, ctrl_status: 1}]}"

ros2 topic pub /arm/cmd_current bodyctrl_msgs/msg/CmdSetMotorCurTor "{cmds: [{name: 11, cur_tor: 0.0, ctrl_status: 1}]}"


强化学习控制接口
手柄向上拨动F键，可通过下述ROS service切换状态，通过topic下发速度指令，遥控器按键和摇杆无效（只有C有效），F置中恢复遥控器功能。

运动状态切换：

僵停：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 1, is_need_swing_arm: false}"

回零：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 2, is_need_swing_arm: false}"

站立：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 3, is_need_swing_arm: false}"

原地踏步（回零状态放到地面后，直接调用本接口的话可以从回零状态进入原地踏步状态，但是比较危险，不要这样操作，先进入站立状态，再进入原地踏步）：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 4, is_need_swing_arm: false}"

原地跑步：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 5, is_need_swing_arm: false}"



做动作（5组动作，上一条是执行动作，下一条是取消执行，如果不执行下一条命令，则会一直重复执行该动作）：
ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: true, motion_number: 0}"
ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: false, motion_number: 0}"

ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: true, motion_number: 1}"
ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: false, motion_number: 1}"

ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: true, motion_number: 2}"
ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: false, motion_number: 2}"

ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: true, motion_number: 3}"
ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: false, motion_number: 3}"

ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: true, motion_number: 4}"
ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: false, motion_number: 4}"



矢量速度控制：
geometry_msgs/msg/TwistStamped 结构如下：
std_msgs/Header header
geometry_msgs/Twist twist

其中 geometry_msgs/Twist 结构又如下：
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular

linear 表示线速度 (单位 m/s)，x 表示前后移动，y 表示左右平移，z 表示上下（一般用于无人机等）。

angular 表示角速度 (单位 rad/s)，z 表示绕 z 轴转动（左转、右转）。

ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"


ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}}"

ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}}"



ros2 topic echo /hric/motion/status



传统运控下用接口控制机器人
需要退出强化学习的运控，启动传统运控
tmux
. /opt/ros/humble/setup.bash
. /home/ubuntu/ros2ws/install/setup.bash
ros2 launch motioncontrol_component motioncontrol_component_launch.py

手柄向右拨动H键，可通过下述ROS service切换状态，通过topic下发速度指令，遥控器按键和摇杆无效（只有C有效），H置中恢复遥控器功能。

僵停：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 1, is_need_swing_arm: false}"
✅

回零：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 2, is_need_swing_arm: false}"
✅

站立：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 3, is_need_swing_arm: false}"
✅


tmux
sudo su
. /home/ubuntu/ros2ws/install/setup.bash
ros2 launch body_control body.launch.py

tmux
. /home/ubuntu/ros2ws/install/setup.bash
ros2 run rl_control_new rl_control_node


手臂电机电流模式

ros2 topic pub /arm/cmd_current bodyctrl_msgs/msg/CmdSetMotorCurTor "{cmds: [{ name: 22, cur_tor: 0.2, ctrl_status: 1 }]}"
❌

ros2 topic pub /arm/cmd_current bodyctrl_msgs/msg/CmdSetMotorCurTor "
cmds:
- name: 22
  cur_tor: 0.2
  ctrl_status: 1
"
❌

ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 22, pos: -0.3, spd: 0.2, cur: 8.0 }]}"


ros2 topic pub /arm/cmd_current bodyctrl_msgs/msg/CmdSetMotorCurTor "{cmds: [{ name: 14, cur_tor: 1.0, ctrl_status: 1 }]}"
❌
ros2 topic pub /arm/cmd_current bodyctrl_msgs/msg/CmdSetMotorCurTor "
cmds:
- name: 14
  cur_tor: 1.0
  ctrl_status: 1
"
❌

ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 14, pos: -0.6, spd: 0.2, cur: 8.0 }]}"


ros2 topic pub /arm/cmd_current bodyctrl_msgs/CmdSetMotorCurTor "header: stamp: sec: 1750403193 nanosec: 0 frame_id: '' cmds: - name: 22 cur_tor: 0.2 ctrl_status: 1 "
❌yaml.scanner.ScannerError: mapping values are not allowed here
  in "<unicode string>", line 1, column 14:
    header: stamp: sec: 1750403193 nanosec: 0 fra ...

ros2 topic pub /arm/cmd_current bodyctrl_msgs/CmdSetMotorCurTor "header: stamp: sec: 1750403193 nanosec: 0 frame_id: '' cmds: - name: 14 cur_tor: 6.0 ctrl_status: 1 "
❌yaml.scanner.ScannerError: mapping values are not allowed here
  in "<unicode string>", line 1, column 14:
    header: stamp: sec: 1750403193 nanosec: 0 fra ...


ros2 service call /inspire_hand/set_force/right_hand bodyctrl_msgs/srv/SetForce "{force0_ratio: 0.1, force1_ratio: 0.0, force2_ratio: 0.0, force3_ratio: 0.0, force4_ratio: 0.0, force5_ratio: 0.0}"


灵巧手手指关节定义如下：
uint16 MOTOR_FINGER_LITTLE = 1,        #little finger   小指
uint16 MOTOR_FINGER_RING = 2,          #ring finger   无名指
uint16 MOTOR_FINGER_MIDDLE = 3,        #middle finger   中指
uint16 MOTOR_FINGER_FORE = 4,          #Fore finger   食指
uint16 MOTOR_FINGER_THUMB_BEND = 5,    #thumb bend  拇指弯曲
uint16 MOTOR_FINGER_THUMB_ROTATION = 6,#thumb rotation  拇指旋转

控制手指各关节角度的topic：
/inspire_hand/ctrl/left_hand
类型：
sensor_msgs/msg/JointState

控制手指各关节角度的service：
/inspire_hand/set_angle_flexible/left_hand
类型：
bodyctrl_msgs/srv/SetAngleFlexible
定义：
string[] name
float32[] angle_ratio
---
bool angle_accepted

控制手指各关节力矩的service：
/inspire_hand/set_force/left_hand
类型：
bodyctrl_msgs/srv/SetForce
定义：
float32 force0_ratio
float32 force1_ratio
float32 force2_ratio
float32 force3_ratio
float32 force4_ratio
float32 force5_ratio
---
bool force_accepted

控制手指各关节速度的service：
/inspire_hand/set_speed/left_hand
类型：
bodyctrl_msgs/srv/SetSpeed
定义：
float32 speed0_ratio
float32 speed1_ratio
float32 speed2_ratio
float32 speed3_ratio
float32 speed4_ratio
float32 speed5_ratio
---
bool speed_accepted

状态获取接口：
/inspire_hand/state/left_hand
类型：
sensor_msgs/msg/JointState

如何验证 控制手指各关节力矩的service 是否生效？这个service调用后应该有什么现象？