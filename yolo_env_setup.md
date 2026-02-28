环境和库文件安装
sudo apt update

# ROS 图像相关
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-sensor-msgs \
  python3-opencv \
  python3-numpy

# YOLO（CPU）
<!-- pip3 install torch==2.1.2 torchvision==0.16.2 --index-url https://download.pytorch.org/whl/cpu -->

pip3 install -i https://mirrors.cloud.tencent.com/pypi/simple tqdm==4.66.1 numpy==1.26.4 pandas==2.1.3 seaborn==0.13.0 thop py-cpuinfo==9.0.0 torch==2.1.2 torchvision==0.16.2 opencv-python==4.11.0.86

<!-- pip3 install torch==2.1.2 torchvision==0.16.2 -i https://mirrors.cloud.tencent.com/pypi/simple -->
pip3 install -i https://mirrors.cloud.tencent.com/pypi/simple ultralytics==8.2.0 --no-deps


# YOLO（GPU）
torch==2.7.0 # 单独编译的支持GPU的torch包
torchvision==0.22.0
ultralytics==8.3.232
其他包版本没有太大区别
ultralytics 8.3.121版本会报错，8.3.232不会报错

python3 - << 'EOF'
from torchvision.ops import nms
print(nms)
EOF


验证安装
python3 - << 'EOF'
from ultralytics import YOLO
print("YOLO OK")
EOF

python3 - << 'EOF'
import cv2
print("cv2:", cv2.__version__)
EOF

python3 - << 'EOF'
import torch
import ultralytics
import numpy as np
print("torch:", torch.__version__)
print("ultralytics:", ultralytics.__version__)
print("numpy:", np.__version__)
print("CUDA:", torch.cuda.is_available())
EOF


python3 - << 'EOF'
import torch
import ultralytics
import numpy as np
print("torch:", torch.__version__)
print("ultralytics:", ultralytics.__version__)
print("numpy:", np.__version__)
print("CUDA:", torch.cuda.is_available())
print("compiled cuda:", torch.version.cuda)
print("cudnn:", torch.backends.cudnn.version())
EOF

python3 - << 'EOF'
import torch
print("torch:", torch.__version__)
print("CUDA:", torch.cuda.is_available())
print("compiled cuda:", torch.version.cuda)
print("cudnn:", torch.backends.cudnn.version())
EOF


python3 - << 'EOF'
import torch
print("torch:", torch.__version__)
EOF



cd ~/sdk_demo/src

ros2 pkg create grab_demo \
  --build-type ament_python \
  --dependencies \
  rclpy \
  std_msgs \
  sensor_msgs \
  geometry_msgs \
  cv_bridge \
  image_transport


模型下载
mkdir -p ~/sdk_demo/yolo_models && cd ~/sdk_demo/yolo_models
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt


识别到了物体中心在相机坐标系内的3D坐标，通过tf树可得到点在基坐标系内的3D坐标

为了使用movit2 进行 IK 解算，先安装：

sudo apt install ros-humble-moveit ros-humble-moveit-ros-visualization

<!-- 
如果网速过慢，可尝试在 /etc/apt/apt.conf.d/95proxies 内增加代理设置：
Acquire::http::Proxy "http://10.10.33.101:7890/";
Acquire::https::Proxy "http://10.10.33.101:7890/";
 -->

<!-- 
或者修改 /etc/apt/sources.list 为如下：
deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu jammy main restricted universe multiverse
deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu jammy-updates main restricted universe multiverse
deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu jammy-security main restricted universe multiverse
 -->

. ~/tiangong2pro_tf/install/setup.bash && . ~/sdk_demo/install/setup.bash

启动tf树：
ros2 launch tiangong2pro_urdf display_with_hands.launch.py

启动 moveit2 配置向导：
ros2 run moveit_setup_assistant moveit_setup_assistant

启动 moveit2 服务：
IK解算的服务：
ros2 launch moveit2_config move_group.launch.py

可视化：
ros2 launch moveit2_config moveit_rviz.launch.py

python /home/nvidia/sdk_demo/src/grab_demo/grab_demo/yolo_detect_node.py
python /home/nvidia/sdk_demo/src/grab_demo/grab_demo/grasp_executor_node.py

python /home/nvidia/sdk_demo/src/grab_demo/grab_demo/ik_client_node.py
python /home/nvidia/sdk_demo/src/grab_demo/grab_demo/grasp_prepare_pose_node.py

<!-- ros2 run grab_demo yolo_grab_node --target_classes apple -->

相机话题复制：
ros2 run topic_tools relay /camera/color/image_raw /ob_camera_head/color/image_raw & \
ros2 run topic_tools relay /camera/depth/image_raw /ob_camera_head/depth/image_raw & \
ros2 run topic_tools relay /camera/depth/camera_info /ob_camera_head/depth/camera_info &

查询末端位姿：
ros2 run tf2_ros tf2_echo pelvis left_tcp_link
<!-- ros2 run tf2_ros tf2_echo pelvis L_base_link -->
其中的输出：
Rotation: in Quaternion [0.129, -0.147, -0.012, 0.981]
Rotation: in Quaternion [0.171, -0.246, -0.114, 0.947]
<!-- Rotation: in Quaternion [-0.109, 0.716, -0.663, 0.192] -->
Rotation: in Quaternion [-0.041, 0.712, -0.690, 0.121]

表示对应坐标系的姿态


直接发布IK解算结果到rviz的画面：
json='{"11": 0.1049, "12": 0.7222, "13": -0.2608, "14": -1.9894, "15": 0.6162, "16": 0.0082, "17": -0.2157}'
ros2 topic pub /gui/joint_command std_msgs/msg/String "{data: '$json'}"


直接用目标点的位姿调用IK解算接口：
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "
{
  ik_request: {
    group_name: 'left_arm',
    pose_stamped: {
      header: { frame_id: 'pelvis' },
      pose: {
        position: { x: 0.402417, y: 0.024490, z: 0.159641 },
        orientation: { x: -0.041, y: 0.712, z: -0.690, w: 0.121 }
      }
    },
    timeout: { sec: 2 }
  }
}"

相机服务挂掉时，模拟相机USB拔插，然后重启服务：
echo 0 | sudo tee /sys/bus/usb/devices/2-2/authorized >/dev/null
echo 1 | sudo tee /sys/bus/usb/devices/2-2/authorized >/dev/null
sudo systemctl restart orbbec_head.service
