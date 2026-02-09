# 项目简介

本项目是一个用来简单介绍天工基础开放SDK的项目，主要源码在 `src/sdk_demo/sdk_demo` 下，里面每一个源码文件都展示了某一个或者某几个SDK的调用方式，都有较为详细的注释，有需要可参考阅读。

# 使用方法

将本项目放到 x86 或者 orin 的根目录，

进入项目根目录，例如`/home/nvidia/tk_sdk_demo`

执行 `colcon build` 进行构建

执行 `source install.bash` 设置环境变量

每一个示例代码都使用 `ros run sdk_demo xxx` 来运行，详细的可参考每个源码文件顶部的说明，都列出了详细的运行命令及注意事项的。