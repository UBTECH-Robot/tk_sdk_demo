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
pip3 install torch==2.1.2 torchvision==0.16.2 --index-url https://download.pytorch.org/whl/cpu
pip3 install -i https://mirrors.cloud.tencent.com/pypi/simple tqdm==4.66.1 numpy==1.21.5 ultralytics==8.2.0  --no-deps

pip3 install -i https://mirrors.cloud.tencent.com/pypi/simple pandas==2.1.3 seaborn==0.13.0 thop py-cpuinfo==9.0.0 torch==2.1.2 torchvision==0.16.2 ultralytics==8.2.0



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
mkdir -p ~/sdk_demo/yolo_models
cd ~/sdk_demo/yolo_models
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt



把 YOLO 的结果变成 3D 目标点
相机内参如何获取？是否所有深度相机都统一用针孔公式计算XYZ，XYZ的物理意义是对应的点在相机坐标系的3D坐标吗？


使用 tf 树将 P_cam = (X, Y, Z) 转换成 P_base = (x, y, z) 的详细步骤

使用moveit和xarm的区别？