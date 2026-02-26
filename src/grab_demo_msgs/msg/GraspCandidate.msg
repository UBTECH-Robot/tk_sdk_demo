# 抓取候选点消息
# 由感知节点（yolo_detect_node）发布，由执行节点（grasp_executor_node）订阅

# 消息头（时间戳 + 坐标系）
std_msgs/Header header

# 目标坐标系（如 pelvis）下的抓取位姿
geometry_msgs/PoseStamped pose

# 规划组名称（left_arm / right_arm）
string group_name

# YOLO 置信度（0.0 ~ 1.0）
float32 confidence

# 检测到的物体类别名称（如 apple）
string object_class
