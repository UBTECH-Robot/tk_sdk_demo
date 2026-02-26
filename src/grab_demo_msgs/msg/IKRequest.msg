# 自定义IK请求消息
# 简化IK求解的参数传递

# 机械臂组名称（如 "left_arm", "right_arm"）
string group_name

# 目标位姿的参考坐标系（如 "pelvis", "waist_yaw_link"）
string frame_id

# 目标位置（米）
geometry_msgs/Point position

# 目标姿态（四元数）
geometry_msgs/Quaternion orientation
