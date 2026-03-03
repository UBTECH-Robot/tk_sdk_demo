# 抓取示例

## 环境安装

本示例是运行在 192.168.41.2 的 orin 板上，所以以下安装步骤都是在该 orin 板上进行。

### ROS 图像相关
sudo apt update && sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-sensor-msgs \
  python3-opencv \
  python3-numpy

### YOLO 安装

#### 先安装依赖
```bash
pip install -i https://mirrors.cloud.tencent.com/pypi/simple tqdm==4.66.1 numpy==1.23.5 pandas==1.3.5 seaborn==0.13.0 thop py-cpuinfo==9.0.0 torchvision==0.22.0 opencv-python==4.11.0.86

pip install -i https://mirrors.cloud.tencent.com/pypi/simple ultralytics==8.3.232 --no-deps

```

#### 再安装torch
