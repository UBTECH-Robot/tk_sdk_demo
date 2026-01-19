tmux
sudo su
. /home/ubuntu/ros2ws/install/setup.bash
ros2 launch body_control body.launch.py

tmux
. /home/ubuntu/ros2ws/install/setup.bash
ros2 run rl_control_new rl_control_node


. /home/ubuntu/ros2demo/install/setup.bash


ros2 topic echo /imu
✅

ros2 topic echo /imu/status
✅

ros2 topic echo /waist/motor_status
❌ 在 7c:2a:31:25:e7:0c 上 WARNING: topic [/waist/motor_status] does not appear to be published yet

ros2 topic echo /arm/motor_status
❌ 在 7c:2a:31:25:e7:0c 上 WARNING: topic [/arm/motor_status] does not appear to be published yet

ros2 topic echo /leg/motor_status
✅


IMPORTANT: To control individual motors using the commands below, you must first enter the root environment using `sudo su`, then execute source `. /home/ubuntu/ros2ws/install/setup.bash`, then you are ready to control individual motors.

划重点：想用下述的ros命令控制单个电机转动，请先切换到 root 用户，再进入 /home/ubuntu/ros2ws 下执行对应的 source 命令，然后再执行下述的控制命令。


腰部电机
状态：
ros2 topic echo /waist/status
✅

位置控制：
ros2 topic pub /waist/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{header: {stamp: {sec: 0, nanosec: 0 }, frame_id: ''},cmds: [{name: 31, pos: -0.3, spd: 0.2, cur: 8.0 }]}"
ros2 topic pub /waist/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{header: {stamp: {sec: 0, nanosec: 0 }, frame_id: ''},cmds: [{name: 31, pos: 0.0, spd: 0.2, cur: 8.0 }]}"
✅
使用ros2 topic pub命令时也可以省略header，本文档后续包含header的命令都暂时省略header：
ros2 topic pub /waist/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 31, pos: -0.3, spd: 0.2, cur: 8.0 }]}"
ros2 topic pub /waist/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 31, pos: 0.0, spd: 0.2, cur: 8.0 }]}"
✅

力位混合控制：
ros2 topic pub /waist/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 31,kp: 30.0,kd: 10.0,pos: -0.7,spd: 0.0,tor: 0.0}]}"
ros2 topic pub /waist/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 31,kp: 30.0,kd: 10.0,pos: 0.0,spd: 0.0,tor: 0.0}]}"
✅

速度模式：
ros2 topic pub /waist/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{name: 31,spd: 0.1,cur: 20.0}]}"
ros2 topic pub /waist/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{name: 31,spd: 0.0,cur: 8.0}]}"
❌

头部电机
状态：
ros2 topic echo /head/status
✅

位置控制：
ros2 topic pub /head/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 1, pos: 0.1, spd: 0.2, cur: 5.0 }]}"

ros2 topic pub /head/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 1, pos: 0.0, spd: 0.2, cur: 5.0 }]}"

ros2 topic pub /head/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 2, pos: -0.1, spd: 0.2, cur: 5.0 }]}"

ros2 topic pub /head/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 2, pos: 0.3, spd: 0.2, cur: 5.0 }]}"

ros2 topic pub /head/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 3, pos: -0.1, spd: 0.2, cur: 5.0 }]}"


手臂电机
状态：
ros2 topic echo /arm/status
✅

位置控制：
ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 12, pos: 0.1, spd: 0.2, cur: 8.0 }]}"
ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 12, pos: 0.5, spd: 0.2, cur: 10.0 }]}"
ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 22, pos: -0.3, spd: 0.2, cur: 8.0 }]}"
✅

ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 11, pos: -0.5, spd: 0.2, cur: 8.0 }]}"
ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 21, pos: -0.5, spd: 0.2, cur: 8.0 }]}"
✅

ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 13, pos: -0.5, spd: 0.2, cur: 8.0 }]}"
ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 23, pos: -0.5, spd: 0.2, cur: 8.0 }]}"
✅

ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 14, pos: -0.2, spd: 0.2, cur: 8.0 }]}"
ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 24, pos: -0.2, spd: 0.2, cur: 8.0 }]}"
✅

力位混合模式：
ros2 topic pub /arm/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 11, kp: 30.0, kd: 10.0, pos: 0.5, spd: 0.0, tor: 0.0}]}"
ros2 topic pub /arm/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 21, kp: 30.0, kd: 10.0, pos: 0.4, spd: 0.0, tor: 0.0}]}"
✅

ros2 topic pub /arm/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 12, kp: 50.0, kd: 10.0, pos: 0.3, spd: 0.0, tor: 0.0}]}"
ros2 topic pub /arm/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 22, kp: 50.0, kd: 10.0, pos: -0.3, spd: 0.0, tor: 0.0}]}"
✅

ros2 topic pub /arm/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 13, kp: 30.0, kd: 10.0, pos: 0.2, spd: 0.0, tor: 0.0}]}"
ros2 topic pub /arm/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 23, kp: 30.0, kd: 10.0, pos: 0.2, spd: 0.0, tor: 0.0}]}"
✅

ros2 topic pub /arm/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 14, kp: 30.0, kd: 10.0, pos: -0.5, spd: 0.0, tor: 0.0}]}"
ros2 topic pub /arm/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 24, kp: 30.0, kd: 10.0, pos: -0.5, spd: 0.0, tor: 0.0}]}"
✅

速度控制
注意这个消息一旦发布，该电机将以该速度持续转动，直到该话题收到一个spd为0.0的消息，因此需要谨慎使用
ros2 topic pub /arm/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{ name: 11, spd: -0.1, cur: 5.0 }]}"
ros2 topic pub /arm/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{ name: 11, spd: 0.0, cur: 5.0 }]}"
ros2 topic pub /arm/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{ name: 11, spd: 0.1, cur: 5.0 }]}"
✅

ros2 topic pub /arm/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{ name: 21, spd: -0.1, cur: 5.0 }]}"
ros2 topic pub /arm/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{ name: 21, spd: 0.0, cur: 5.0 }]}"
ros2 topic pub /arm/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{ name: 21, spd: 0.1, cur: 5.0 }]}"
✅

标零接口：
ros2 topic pub /arm/cmd_set_zero std_msgs/msg/String "{data: '11'}"
ros2 topic pub /arm/cmd_set_zero std_msgs/msg/String "{data: '12'}"
ros2 topic pub /arm/cmd_set_zero std_msgs/msg/String "{data: '13'}"
ros2 topic pub /arm/cmd_set_zero std_msgs/msg/String "{data: '14'}"
ros2 topic pub /arm/cmd_set_zero std_msgs/msg/String "{data: '21'}"
ros2 topic pub /arm/cmd_set_zero std_msgs/msg/String "{data: '22'}"
ros2 topic pub /arm/cmd_set_zero std_msgs/msg/String "{data: '23'}"
ros2 topic pub /arm/cmd_set_zero std_msgs/msg/String "{data: '24'}"
✅

电流模式

ros2 topic pub -r 200 /arm/cmd_current bodyctrl_msgs/msg/CmdSetMotorCurTor "{cmds: [{ name: 22, cur_tor: 5.0, ctrl_status: 1 }]}"
❌

ros2 topic pub /arm/cmd_current bodyctrl_msgs/msg/CmdSetMotorCurTor "
cmds:
- name: 22
  cur_tor: 0.2
  ctrl_status: 1
"
❌

ros2 topic pub /arm/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 22, pos: -0.5, spd: 0.2, cur: 8.0 }]}"


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

腿部电机
状态：
ros2 topic echo /leg/status
✅

位置控制：
ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 51, pos: 0.0, spd: 0.2, cur: 15.0 }]}"
ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 61, pos: -0.0, spd: 0.2, cur: 15.0 }]}"
✅

ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 52, pos: -0.2, spd: 0.2, cur: 15.0 }]}"
ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 62, pos: -0.2, spd: 0.2, cur: 15.0 }]}"
✅

ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 53, pos: 0.0, spd: 0.2, cur: 15.0 }]}"
ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 63, pos: -0.0, spd: 0.2, cur: 15.0 }]}"
✅

ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 54, pos: 0.0, spd: 0.2, cur: 15.0 }]}"
ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 64, pos: 0.0, spd: 0.2, cur: 15.0 }]}"
✅

ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 55, pos: 0.0, spd: 0.2, cur: 10.0 }]}"
ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 65, pos: 0.0, spd: 0.2, cur: 10.0 }]}"
✅

ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 56, pos: 0.0, spd: 0.2, cur: 10.0 }]}"
ros2 topic pub /leg/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition "{cmds: [{name: 66, pos: 0.0, spd: 0.2, cur: 10.0 }]}"
✅

力位混合控制：
ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 51, kp: 40.0, kd: 10.0, pos: 0.2, spd: 0.0, tor: 0.0}]}"
ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 61, kp: 40.0, kd: 10.0, pos: -0.2, spd: 0.0, tor: 0.0}]}"
✅

ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 52, kp: 40.0, kd: 10.0, pos: -0.4, spd: 0.0, tor: 0.0}]}"
ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 62, kp: 40.0, kd: 10.0, pos: -0.4, spd: 0.0, tor: 0.0}]}"
✅

ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 53, kp: 50.0, kd: 10.0, pos: 0.3, spd: 0.0, tor: 0.0}]}"
ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 63, kp: 50.0, kd: 10.0, pos: -0.3, spd: 0.0, tor: 0.0}]}"
✅

ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 54, kp: 40.0, kd: 10.0, pos: 0.2, spd: 0.0, tor: 0.0}]}"
ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 64, kp: 40.0, kd: 10.0, pos: 0.2, spd: 0.0, tor: 0.0}]}"
✅

ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 55, kp: 40.0, kd: 10.0, pos: 0.2, spd: 0.0, tor: 0.0}]}"
ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 65, kp: 40.0, kd: 10.0, pos: 0.2, spd: 0.0, tor: 0.0}]}"
✅

ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 56, kp: 40.0, kd: 10.0, pos: 0.2, spd: 0.0, tor: 0.0}]}"
ros2 topic pub /leg/cmd_ctrl bodyctrl_msgs/msg/CmdMotorCtrl "{cmds: [{name: 66, kp: 40.0, kd: 10.0, pos: 0.2, spd: 0.0, tor: 0.0}]}"
✅

速度模式：
一定注意这个消息一旦发布，该电机将以该速度持续转动，直到该话题收到一个spd为0.0的消息，因此需要谨慎使用
ros2 topic pub /leg/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{ name: 51, spd: 0.1, cur: 15.0 }]}"
ros2 topic pub /leg/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{ name: 51, spd: 0.0, cur: 15.0 }]}"
ros2 topic pub /leg/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{ name: 51, spd: -0.1, cur: 15.0 }]}"
✅

ros2 topic pub /leg/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{ name: 61, spd: -0.1, cur: 15.0 }]}"
ros2 topic pub /leg/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{ name: 61, spd: 0.0, cur: 15.0 }]}"
ros2 topic pub /leg/cmd_vel bodyctrl_msgs/msg/CmdSetMotorSpeed "{cmds: [{ name: 61, spd: 0.1, cur: 15.0 }]}"
✅


ros2 topic echo /xunfei/aiui_msg
❌
WARNING: topic [/xunfei/aiui_msg] does not appear to be published yet
Could not determine the type for the passed topic
❌ Plus文档里没有提到如何用ROS2命令启动讯飞的服务，上述的话题是需要在讯飞的服务启动后才会发布的
❌ 俊青可以直接把命令发给我，但是用户文档里也需要有这块内容，要不然用户拿到机器后无法使用ASR服务？

在 192.168.41.2 的orin板上先启动讯飞服务，可用tmux：
. ~/voice_ws/install/setup.bash
ros2 launch xunfei_dev_socket xunfei_dev_all.launch.py
启动服务后，ros2 topic echo /xunfei/aiui_msg 有输出，但是文本看不完整，还是用python代码来尝试。
代码在 5_8_1_asr_demo.py
✅

. /home/nvidia/voice_ros2/install/setup.bash
❌ 这里有一个问题，我在两台Plus上验证，A机器是目录是voice_ws，B机器目录是voice_ros2，不统一，出厂的机器语音板上语音包所在的目录名称应该统一的
ros2 launch xunfei_dev_socket xunfei_dev_all.launch.py
ros2 topic pub /xunfei/tts_play std_msgs/msg/String "{data: '{\"file\": \"/home/nvidia/data/speech/chenggong.mp3\"}'}"
ros2 topic pub /xunfei/tts_play std_msgs/msg/String "{data: '{\"file\": \"/home/nvidia/data/speech/guzhang.mp3\"}'}"
ros2 topic pub /xunfei/tts_play std_msgs/msg/String "{data: '{\"file\": \"/home/nvidia/data/speech/didianliang.wav\"}'}"
ros2 topic pub /xunfei/tts_play std_msgs/msg/String "{data: '{\"file\": \"/home/nvidia/data/speech/kaishichongdian.mp3\"}'}"
ros2 topic pub /xunfei/tts_play std_msgs/msg/String "{data: '{\"file\": \"/home/nvidia/data/speech/anjianyin.mp3\"}'}"
✅

ros2 topic echo /power/battery/status
✅

ros2 topic echo /power/board/status
✅

ros2 topic echo /power/board/key_status
✅

ros2 topic echo /sbus_data/event
✅

cat /home/ubuntu/ros2ws/version_info.json
✅

bag录制接口：
❌ /home/ubnutu/ros2ws/install/utils/lib/utils/topics.config，默认列表中存储了本体
拼写错误

ros2 launch utils record_config_topic.py
✅

强化学习控制接口
先确保本体驱动是启动的：
tmux
sudo su
. /home/ubuntu/ros2ws/install/setup.bash
ros2 launch body_control body.launch.py

再启动强化学习运控服务：
tmux
. /home/ubuntu/ros2ws/install/setup.bash
ros2 run rl_control_new rl_control_node
手柄向上拨动E键，可通过下述ROS service切换状态，通过topic下发速度指令，遥控器按键和摇杆无效（只有C有效），E置中恢复遥控器功能。


运动状态切换：

僵停：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 1, is_need_swing_arm: false}"
✅

回零：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 2, is_need_swing_arm: false}"
✅

站立：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 3, is_need_swing_arm: false}"
✅

原地踏步（回零状态放到地面后，直接调用本接口的话可以从回零状态进入原地踏步状态，但是比较危险，不要这样操作，先进入站立状态，再进入原地踏步）：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 4, is_need_swing_arm: false}"
✅

原地跑步：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 5, is_need_swing_arm: false}"
✅


手柄向上拨动E键，可通过下述命令控制机器人做动作

做动作（5组动作，上一条是执行动作，下一条是取消执行，如果不执行下一条命令，则会一直重复执行该动作）：
<!-- ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: true, motion_number: 0}"
ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: false, motion_number: 0}"
✅ 已废弃-->

ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: true, motion_number: 1}"
ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: false, motion_number: 1}"
✅

ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: true, motion_number: 2}"
ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: false, motion_number: 2}"
✅

ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: true, motion_number: 3}"
ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: false, motion_number: 3}"
✅

ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: true, motion_number: 4}"
ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: false, motion_number: 4}"
✅

ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: true, motion_number: 5}"
ros2 service call /hric/motion/set_motion_number hric_msgs/srv/SetMotionNumber "{is_motion: false, motion_number: 5}"
✅

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
✅

ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
✅

ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
✅

ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}}"
✅

ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}}"
✅

机器人下肢运动状态
ros2 topic echo /hric/motion/status
✅



机身高度和姿态控制接口，属于强化学习运控下的机身控制接口
也就是控制设备 横滚(roll)、俯仰(pitch)、偏航(yaw) 三个角度，以及 z 高度（个人理解应该是重心或者是几何中点或者是中部某个位置的高度，因为默认是0.89米，而天工身高是1.72左右）
几个参数的取值范围分别是：
 roll[-0.2,0.2]rad
 pitch[-0.3,0.4]rad
 yaw[-1.0,1.0]rad
 z[0.6,0.89]m，默认0.89m


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

原地踏步：
ros2 service call /hric/motion/set_motion_mode hric_msgs/srv/SetMotionMode "{walk_mode_request: 4, is_need_swing_arm: false}"
✅

前进：
ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
✅

左转：
ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}}"
✅

z轴运动：
ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.0, y: 0.0, z: 0.05}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
❌

ros2 topic pub /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
✅

下肢运动状态：
ros2 topic echo /hric/motion/status

ROS1传统运控
. /opt/ros/noetic/setup.bash
. /home/ubuntu/rosws/install_isolated/setup.bash
roslaunch body_control body.launch
roslaunch motion_control_joint motion.launch

手柄向右拨动H键，可通过下述ROS service切换状态，通过topic下发速度指令，遥控器按键和摇杆无效（只有C有效），H置中恢复遥控器功能。

僵停：
rosservice call /hric/motion/set_motion_mode "{walk_mode_request: 1,is_need_swing_arm: false}"
✅

回零：
rosservice call /hric/motion/set_motion_mode "{walk_mode_request: 2,is_need_swing_arm: false}"
✅

站立：
rosservice call /hric/motion/set_motion_mode "{walk_mode_request: 3,is_need_swing_arm: false}"
✅

原地踏步：
rosservice call /hric/motion/set_motion_mode "{walk_mode_request: 4,is_need_swing_arm: false}"
✅

ROS1传统运控下矢量速度控制：
geometry_msgs/msg/TwistStamped 结构如下：
std_msgs/Header header
geometry_msgs/Twist twist

其中 geometry_msgs/Twist 结构又如下：
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular

linear 表示线速度 (单位 m/s)，x 表示前后移动，y 表示左右平移，z 表示上下（一般用于无人机等）。

angular 表示角速度 (单位 rad/s)，z 表示绕 z 轴转动（左转、右转）。

rostopic pub /hric/robot/cmd_vel geometry_msgs/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
✅

rostopic pub /hric/robot/cmd_vel geometry_msgs/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
✅

rostopic pub /hric/robot/cmd_vel geometry_msgs/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
✅

rostopic pub /hric/robot/cmd_vel geometry_msgs/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}}"
✅

rostopic pub /hric/robot/cmd_vel geometry_msgs/TwistStamped "{header: {frame_id: 'base_link' }, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}}"
✅

rostopic echo /hric/motion/status
✅

适用 motion_control_joint 运控
rostopic info /hric/robot/float_base_rpyz 
❌ ERROR: Unknown topic /hric/robot/float_base_rpyz

适用 motion_control_joint 运控
rostopic echo /hric/robot/float_base_rpyz_status
❌ WARNING: topic [/hric/robot/float_base_rpyz_status] does not appear to be published yet

双臂末端笛卡尔空间位姿控制
适用于motion_control_cartesian
❌ ROS1也不存在 motion_control_cartesian 包

配合ros1的 roslaunch motion_control_joint motion.launch 命令启动的运控 来控制手臂电机，也就是说在ROS1的本体驱动和运控都启动的前提下，可以先把遥控器H右拨，然后用命令控制天工站立，然后可以用如下命令来控制手臂电机：
rostopic pub /arm/cmd_pos bodyctrl_msgs/CmdSetMotorPosition "{header: {stamp: {secs: 0, nsecs: 0}, frame_id: ''},cmds: [{name: 22, pos: -0.5, spd: 0.1, cur: 10.0}]}"
✅

rostopic pub /arm/cmd_pos bodyctrl_msgs/CmdSetMotorPosition "{header: {stamp: {secs: 0, nsecs: 0}, frame_id: ''},cmds: [{name: 22, pos: -0.7, spd: 0.1, cur: 10.0}]}"
✅

rostopic pub /arm/cmd_pos bodyctrl_msgs/CmdSetMotorPosition "{header: {stamp: {secs: 0, nsecs: 0}, frame_id: ''},cmds: [{name: 12, pos: 0.3, spd: 0.1, cur: 10.0}]}"
✅