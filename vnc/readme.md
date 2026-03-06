本目录内，是尝试在x86和orin上安装vnc服务的脚本，以便有桌面可查看rviz，但是帧率非常差，只有5左右，暂时无法改进。

- `sudo ./install_vnc.sh` 用来执行安装
- `./vnc_manager.sh` 查看状态
- `./vnc_manager.sh start` 启动服务
  
启动成功后，可通过浏览器访问 http://<当前orin板的ip>:8888 打开vnc桌面。

Ctrl C可停止服务。