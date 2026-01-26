### 1，创建工程目录

如果是从0开始创建工程时先创建目录并进入（这里假设工程名为sdk_demo）：
```bash
mkdir -p ~/sdk_demo/src
cd ~/sdk_demo/src
```

### 2，创建ROS2项目

指定项目名称，编程语言，依赖包
```bash
ros2 pkg create sdk_demo --build-type ament_python --dependencies rclpy geometry_msgs hric_msgs bodyctrl_msgs std_msgs 
```
- `ros2 pkg create` 是创建包的命令
- `sdk_demo` 是包名
- `--build-type ament_python` 表示该包是纯python包
- `--dependencies rclpy geometry_msgs hric_msgs bodyctrl_msgs std_msgs` 表示该包依赖于这些ROS包构建和运行

### 3，创建Python源码文件

在有了工程项目后，可在 src/sdk_demo/sdk_demo/ 目录下新建 ros2 的 python 文件，编写 ros2 的 python 代码

如：5_6_2_1_arm_control_demo.py

然后在 src/sdk_demo/setup.py 文件里的 entry_points.console_scripts 数组内相应位置增加一行入口，例如：
```python
'arm_control_demo = sdk_demo.5_6_2_1_arm_control_demo:main',
```

### 4，编译

进入项目根目录，执行编译命令
```bash
cd ~/sdk_demo
colcon build --packages-select sdk_demo
```

### 5，运行

运行之前需要先source ros2ws里的环境变量，因为前面步骤中指定的 hric_msgs 和 bodyctrl_msgs 消息定义在这个包里
```bash
. ~/ros2ws/install/setup.bash
```

然后source当前项目的环境变量
```bash
. ~/sdk_demo/install/setup.bash
```

运行示例代码：
```bash
ros2 run sdk_demo arm_control_demo
ros2 run sdk_demo rl_cmd_control_test
```


