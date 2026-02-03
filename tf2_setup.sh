#!/bin/bash
# TF2坐标系配置脚本
# 这个脚本发布静态变换，建立相机与机器人的坐标系关系

# 通常需要运行的命令：

# 1. 如果相机直接安装在头部，且坐标系对齐，可以发布从 camera_head_link 到 ob_camera_head 的变换
# （需要先确认 Orbbec 驱动发布的具体坐标系名称）
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0 0 0 1 camera_head_link ob_camera_head_link

# 2. 发布从 ob_camera_head_link 到 ob_camera_head_color_optical_frame 的变换
# （如果Orbbec驱动没有自动发布这个变换）
# 注：根据相机类型，可能需要调整旋转参数

# 3. 查看完整的TF树
# ros2 run tf2_tools view_frames

# 4. 查看具体的两个坐标系之间的变换
# ros2 run tf2_ros tf2_echo L_base_link ob_camera_head_depth_optical_frame
# ros2 run tf2_ros tf2_echo R_base_link ob_camera_head_depth_optical_frame

# 5. 在终端查看所有活跃的TF坐标系
# ros2 run tf2_ros tf2_monitor
# 示例输出（根据实际情况可能有所不同）：
# Frame: ob_camera_head_color_frame, published by <no authority available>, Average Delay: 3249.28, Max Delay: 3249.28
# Frame: ob_camera_head_color_optical_frame, published by <no authority available>, Average Delay: 3249.28, Max Delay: 3249.28
# Frame: ob_camera_head_depth_frame, published by <no authority available>, Average Delay: 3249.28, Max Delay: 3249.28
# Frame: ob_camera_head_depth_optical_frame, published by <no authority available>, Average Delay: 3249.28, Max Delay: 3249.28
# Frame: ob_camera_head_link, published by <no authority available>, Average Delay: 2953.01, Max Delay: 2953.01

# 6. 在RViz中可视化
```bash
rviz2

# 配置步骤：
# 1. Fixed Frame 设为 "L_base_link" 或 "R_base_link"（取决于使用的基座）
# 2. Add → Pose
# 3. Topic 选择 "/grasp_pose"
# 4. 观察箭头是否指向检测物体（在基座坐标系下）
```


# 注意事项：
# - ob_camera_head_color_optical_frame 是OpenCV使用的标准光学坐标系
# - 如果驱动没有发布这个坐标系，可能需要手动添加变换
# - 变换矩阵的前3个数字是平移(x, y, z)，后4个是旋转四元数(x, y, z, w)
# - 单位四元数是 (0, 0, 0, 1)，表示无旋转
