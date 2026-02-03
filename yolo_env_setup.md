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

