# 项目简介

本项目是一个用来简单介绍天工基础开放SDK的项目，主要源码在 `src/sdk_demo/sdk_demo` 下，里面每一个源码文件都展示了某一个或者某几个SDK的调用方式，都有较为详细的注释，有需要可参考阅读。

# 使用方法

将本项目放到 x86 或者 orin 的根目录，

进入项目根目录，例如`/home/nvidia/tk_sdk_demo`

执行 `colcon build` 进行构建

执行 `source install.bash` 设置环境变量

每一个示例代码都使用 `ros run sdk_demo xxx` 来运行，详细的可参考每个源码文件顶部的说明，都列出了详细的运行命令及注意事项的。

# 示例列表

| 文件名 | 功能说明 | 运行命令 |
|--------|--------|---------|
| src/sdk_demo/sdk_demo/3_1_imu_demo.py | IMU 数据读取 | `ros2 run sdk_demo imu_demo` |
| src/sdk_demo/sdk_demo/3_2_imu_status_demo.py | IMU 状态获取 | `ros2 run sdk_demo imu_status_demo` |
| src/sdk_demo/sdk_demo/4_motor_temper_demo.py | 电机温度监测 | 见源码顶部 |
| src/sdk_demo/sdk_demo/5_head_motor_control.py | 头部电机控制 | 见源码顶部 |
| src/sdk_demo/sdk_demo/5_motor_status_demo.py | 电机状态查询 | 见源码顶部 |
| src/sdk_demo/sdk_demo/6_waist_motor_control.py | 腰部电机控制 | 见源码顶部 |
| src/sdk_demo/sdk_demo/7_arm_motor_control.py | 手臂电机控制 | 见源码顶部 |
| src/sdk_demo/sdk_demo/8_leg_motor_control.py | 腿部电机控制 | 见源码顶部 |
| src/sdk_demo/sdk_demo/9_audio_asr.py | 语音识别 | 见源码顶部 |
| src/sdk_demo/sdk_demo/9_audio_player.py | 音频播放 | 见源码顶部 |
| src/sdk_demo/sdk_demo/9_audio_saver.py | 音频保存 | 见源码顶部 |
| src/sdk_demo/sdk_demo/9_audio_tts.py | 文本转语音 | 见源码顶部 |
| src/sdk_demo/sdk_demo/10_depth_camera.py | 深度相机 | 见源码顶部 |
| src/sdk_demo/sdk_demo/11_hand_control.py | 手部控制 | 见源码顶部 |
| src/sdk_demo/sdk_demo/11_hand_status.py | 手部状态 | `ros2 run sdk_demo hand_status` |
| src/sdk_demo/sdk_demo/13_1_battery_status_demo.py | 电池状态 | `ros2 run sdk_demo battery_status_demo` |
| src/sdk_demo/sdk_demo/13_2_power_board_status_demo.py | 电源板状态 | `ros2 run sdk_demo power_board_status_demo` |
| src/sdk_demo/sdk_demo/13_3_stop_key_status_demo.py | 停止键状态 | `ros2 run sdk_demo stop_key_status_demo` |
| src/sdk_demo/sdk_demo/14_sbus_event_demo.py | 遥控器SBUS事件 | `ros2 run sdk_demo sbus_event_demo` |
| src/sdk_demo/sdk_demo/17_lidar_demo.py | 激光雷达数据获取示例 | `ros2 run sdk_demo lidar_demo` |
| src/sdk_demo/sdk_demo/17_save_point_cloud_pcd.py | 激光雷达点云保存(PCD格式) | `ros2 run sdk_demo save_point_cloud_pcd` |
| src/sdk_demo/sdk_demo/17_save_pointcloud_images.py | 激光雷达点云保存(图像格式) | `ros2 run sdk_demo save_pointcloud_images` | 