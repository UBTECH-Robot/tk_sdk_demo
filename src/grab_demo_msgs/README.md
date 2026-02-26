# grab_demo_msgs

自定义消息类型包，用于简化IK求解参数传递。

## 消息定义

### IKRequest.msg

简化的IK请求消息，包含IK求解所需的最小参数集：

```msg
# 机械臂组名称（如 "left_arm", "right_arm"）
string group_name

# 目标位姿的参考坐标系（如 "pelvis", "waist_yaw_link"）
string frame_id

# 目标位置（米）
geometry_msgs/Point position

# 目标姿态（四元数）
geometry_msgs/Quaternion orientation
```

## 使用方法

### 1. 创建IK请求消息

```python
from grab_demo_msgs.msg import IKRequest
from geometry_msgs.msg import Point, Quaternion

# 创建消息实例
ik_req = IKRequest()
ik_req.group_name = 'left_arm'
ik_req.frame_id = 'pelvis'
ik_req.position = Point(x=0.374878, y=0.047079, z=0.310416)
ik_req.orientation = Quaternion(x=-0.041013, y=0.712226, z=-0.690219, w=0.121038)
```

### 2. 转换为MoveIt IK请求

```python
from moveit_msgs.srv import GetPositionIK

request = GetPositionIK.Request()
request.ik_request.group_name = ik_req.group_name
request.ik_request.pose_stamped.header.frame_id = ik_req.frame_id
request.ik_request.pose_stamped.pose.position = ik_req.position
request.ik_request.pose_stamped.pose.orientation = ik_req.orientation
```

### 3. 完整示例

参考 [ik_request_example.py](../../grab_demo/grab_demo/ik_request_example.py)

## 编译

```bash
cd /home/nvidia/sdk_demo
./build.sh
```

## 优势

- **简化参数传递**：只需4个字段而不是复杂的嵌套结构
- **类型安全**：使用ROS2消息类型系统
- **可扩展**：后续可添加tolerance、timeout等可选参数
- **可序列化**：可通过话题/服务传递
