本目录也在 tk_sdk_demo 项目内，但是和本项目展示天工行者SDK能力的代码没有太多相关性，是尝试在x86和orin上安装vnc服务的脚本，以便有桌面可查看rviz2，主要是考虑用来启动 `ros2 launch tiangong2pro_urdf grasp_pose.launch.py` 服务并通过 rviz2 查看机器人模型的。但是帧率非常差，加载了天工的模型后，帧率只有5左右，暂时无法改进。

- `sudo ./install_vnc.sh` 用来执行安装
- `./vnc_manager.sh` 查看状态
- `./vnc_manager.sh start` 启动服务
  
启动成功后，可通过浏览器访问 http://<当前orin板的ip>:8888 打开vnc桌面。

Ctrl C可停止服务。