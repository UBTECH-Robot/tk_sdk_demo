# install 目录说明

此目录用于存放 VNC 安装所需的 .deb 文件，可以提前下载好放在这里。

## 需要的文件

### TurboVNC (必需)
- `turbovnc_3.1.1_amd64.deb` - x86_64 架构
- `turbovnc_3.1.1_arm64.deb` - ARM64 架构 (Jetson)

### VirtualGL (有 GPU 时需要)
- `virtualgl_3.1_amd64.deb` - x86_64 架构
- `virtualgl_3.1_arm64.deb` - ARM64 架构 (Jetson)

## 下载方法

### 快速下载脚本

```bash
cd install

# x86_64 系统
wget https://sourceforge.net/projects/turbovnc/files/3.1.1/turbovnc_3.1.1_amd64.deb/download -O turbovnc_3.1.1_amd64.deb
wget https://sourceforge.net/projects/virtualgl/files/3.1/virtualgl_3.1_amd64.deb/download -O virtualgl_3.1_amd64.deb

# ARM64 系统 (Jetson)
wget https://sourceforge.net/projects/turbovnc/files/3.1.1/turbovnc_3.1.1_arm64.deb/download -O turbovnc_3.1.1_arm64.deb
wget https://sourceforge.net/projects/virtualgl/files/3.1/virtualgl_3.1_arm64.deb/download -O virtualgl_3.1_arm64.deb
```

## 使用说明

如果 `install/` 目录下已有对应架构的 .deb 文件，安装脚本会自动使用本地文件，跳过网络下载。

如果本地文件不存在，脚本会自动从网络下载。
