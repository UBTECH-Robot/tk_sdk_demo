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

pip3 install -i https://mirrors.cloud.tencent.com/pypi/simple tqdm==4.66.1 numpy==1.26.4 pandas==2.1.3 seaborn==0.13.0 thop py-cpuinfo==9.0.0 torch==2.1.2 torchvision==0.16.2

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


ros2 run grab_demo yolo_grab_node --target_classes apple