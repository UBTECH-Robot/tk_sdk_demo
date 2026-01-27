#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Livox 雷达数据采集与保存节点

运行脚本：
    ros2 run sdk_demo lidar_demo
    
本脚本用于订阅和处理Livox雷达的两种数据类型：
1. IMU数据 (/livox/imu)
2. 点云数据 (/livox/lidar)

背景知识：
==========

1. Livox雷达专用IMU (Inertial Measurement Unit - 惯性测量单元)
   
   1.1 为什么激光雷达需要配备IMU？
   - 运动畸变补偿：激光雷达扫描需要时间，在这期间如果雷达本身在运动（如机器人移动、
     转向），会导致点云发生畸变。IMU可以实时测量这些运动，用于后续的畸变校正
   - 姿态估计：提供雷达在空间中的精确姿态信息，这对于点云数据的坐标变换至关重要
   - 时间同步：IMU数据频率高（通常100-200Hz），可以为点云提供高频的姿态插值
   - 多传感器融合：IMU数据可以与点云融合，提高定位精度和鲁棒性
   
   1.2 Livox雷达IMU的特点
   - 硬件时间同步：IMU和激光雷达共享同一时钟源，确保数据时间戳精确对齐
   - 工厂标定：IMU与雷达的相对位置和姿态在出厂时已经标定，无需用户再次标定
   - 高采样率：通常为100Hz或200Hz，能够捕捉快速运动
   - 低延迟：IMU数据与点云数据的时间差通常在毫秒级
   
   1.3 IMU传感器组件
   - 三轴陀螺仪：测量角速度(绕X、Y、Z轴的旋转速度)，单位 rad/s
   - 三轴加速度计：测量线性加速度(沿X、Y、Z轴的加速度)，单位 m/s²
   - 磁力计（部分型号）：测量磁场方向，辅助航向估计
   
   1.4 数据格式与单位
   - 角速度：rad/s (弧度/秒)，表示旋转的快慢
   - 线性加速度：m/s² (米/秒²)，表示速度变化率
   - 方向(四元数)：(x, y, z, w)，无量纲，表示3D空间中的旋转姿态
   - 时间戳：与雷达点云精确同步，误差通常<1ms

2. PointCloud2 (点云数据)
   
   2.1 2D激光雷达 vs 3D激光雷达：点云数据的本质区别
   
   2.1.1 2D激光雷达（平面扫描雷达）
   
   工作原理：
   - 在一个平面内旋转扫描（通常是水平面）
   - 只能感知该平面上的障碍物
   - 就像在地面上画一个圆，只能"看到"与这个圆相交的物体
   
   数据特征：
   - 输出数据：一系列"距离+角度"的组合
   - 数据维度：2维 (距离r, 角度θ)
   - 典型数据格式：LaserScan消息（ROS标准）
   - 数据量：通常几百个点/帧（如360个点对应1度分辨率）
   - 扫描频率：10-50Hz
   
   典型应用场景：
   - 室内机器人导航（地面障碍物检测）
   - AGV（自动导引车）避障
   - 2D SLAM建图（构建平面地图）
   - 楼层内的路径规划
   
   局限性：
   - 无法检测平面外的障碍物（如悬空的障碍物、地面坑洼）
   - 无法获取物体的高度信息
   - 容易被"盲点"困扰（如桌子腿能检测到，但桌面检测不到）
   - 上下楼梯时无法感知台阶高度


   2.1.2 3D激光雷达（立体扫描雷达，如Livox）
   
   工作原理：
   - 在三维空间中扫描，覆盖一个立体视场
   - 能够感知前方空间内的所有物体
   - 就像用探照灯照亮整个空间，记录每一个反射点的位置
   
   数据特征：
   - 输出数据：大量的3D空间点坐标 (x, y, z)
   - 数据维度：3维 + 附加信息（强度、时间戳等）
   - 典型数据格式：PointCloud2消息（ROS标准）
   - 数据量：数万到数十万个点/帧（如50,000-100,000点/帧）
   - 扫描频率：10-20Hz
   - 单帧数据量：通常800KB-2MB（因此无法像2D雷达那样直观阅读）
     
   优势：
   - 完整的3D空间感知能力
   - 可以检测悬空障碍物、地面坑洼、楼梯台阶
   - 提供物体的高度、形状、体积信息
   - 更丰富的环境理解（如区分墙和门、识别台阶）
   
   生活类比：
   - 想象你在黑暗房间里用一束会"分裂"的激光
   - 这束激光分成成千上万条细小的光线，射向前方各个方向
   - 每条光线打到物体表面就记录下那个点的3D位置
   - 最终你得到的是整个房间的三维"点阵模型"
   - 你不仅知道墙在哪里，还知道墙有多高、门在哪里、地上有没有台阶
   
   2.1.3 数据格式对比
   
   2D雷达数据（LaserScan）- 人类可读性较高：
   ```
   角度范围: 0° 到 360°
   角度分辨率: 1°
   距离数据: [2.3, 2.5, 2.4, 3.1, ..., 2.3]  (360个数值)
   # 可以理解为：
   # 0度方向距离2.3米
   # 1度方向距离2.5米
   # 2度方向距离2.4米
   # ... 以此类推
   ```
   - 数据量小（几KB）
   - 结构简单（角度+距离）
   - 可以直接用文本查看和理解
   
   3D雷达数据（PointCloud2）- 人类几乎无法直接阅读：
   ```
   点1: (1.234, 2.456, 0.123, 强度100, 时间戳123456789)
   点2: (1.567, 2.890, 0.145, 强度95,  时间戳123456790)
   点3: (1.890, 3.123, 0.167, 强度87,  时间戳123456791)
   ... (还有49,997个点)
   ```
   - 数据量巨大（数百KB到数MB）
   - 结构复杂（3D坐标+多种属性）
   - 二进制编码，必须用专门工具可视化
   - 点是无序的3D散点，无法"逐行阅读"理解
   
   2.1.4 为什么3D点云数据"不可读"，而2D雷达数据相对"可读"？
   
   原因1：数据量差异（100倍差距）
   - 2D雷达：360个点 × 4字节 = 1.4KB
   - 3D雷达：50,000个点 × 16字节 = 800KB
   - 人脑可以处理几百个数字，但无法理解5万个3D坐标
   
   原因2：组织结构差异
   - 2D雷达：有序的角度序列（0°, 1°, 2°, ...）
     * 可以想象成"绕一圈，每隔1度测一个距离"
     * 有清晰的"从左到右"的阅读顺序
   - 3D雷达：无序的空间散点
     * 点与点之间没有明显的顺序关系
     * 就像把一盒沙子倒在地上，无法"阅读"沙粒的排列
   
   原因3：维度复杂度
   - 2D：只需理解"那个方向有多远"（1维信息）
   - 3D：需要理解"那个点在空间哪个位置"（3维信息）
     * 人脑很难同时想象5万个3D点的空间关系
   
   原因4：编码方式
   - 2D雷达：经常使用文本格式或简单数组
   - 3D雷达：为了节省空间，使用紧凑的二进制编码
     * 直接打开就是乱码
   
   类比理解：
   - 2D雷达数据 = 一本电话簿（按字母顺序排列，可以翻阅查找）
   - 3D雷达数据 = 一堆散落的便签纸（每张写着3个数字，没有顺序）
   
   2.1.5 应用选择建议
   
   选择2D激光雷达的场景：
   - 平面环境导航（如工厂、仓库）
   - 成本敏感的项目（2D雷达便宜得多）
   - 低算力平台（数据处理简单）
   - 明确不需要高度信息的应用
   
   必须选择3D激光雷达的场景：
   - 复杂地形（楼梯、坡道、越野）
   - 自动驾驶（需要识别车辆、行人高度）
   - 精细建图（需要完整3D模型）
   - 无人机（需要全方位感知）
   - 物体识别（基于3D形状）
   
   2.1.6 Livox固态雷达 vs 传统机械式3D激光雷达
   
   在3D激光雷达领域，存在两种主要技术路线：传统机械式雷达和新兴的固态雷达（如Livox）。
   它们在工作原理、性能特点、应用场景上有显著差异。
   
   A. 传统机械式3D激光雷达（如Velodyne系列）
   
   工作原理：
   - 多线束激光发射器沿垂直方向排列（如16线、32线、64线、128线）
   - 整个激光发射模块绕垂直轴高速旋转（通常10-20Hz）
   - 每条线束覆盖不同的垂直角度，旋转实现水平360°扫描
   - 就像多层的"旋转灯塔"，每层扫出一个圆环
   
   扫描模式特点：
   - 重复性扫描：每次旋转，激光打到的位置完全相同
   - 固定扫描线：每条激光束的垂直角度固定不变
   - 360°全方位覆盖：可以同时看到前后左右
   - 扫描盲区：相邻扫描线之间存在间隙（"稀疏扫描"）
   
   优势：
   - 360°全方位覆盖，无需转动车身/机器人即可感知周围
   - 技术成熟，算法生态完善（大量开源SLAM/感知算法支持）
   - 多线束版本（64线、128线）分辨率高
   - 远距离测距能力强（可达100-200米）
   
   劣势：
   - 机械旋转部件：易磨损，寿命有限（通常2000-5000小时）
   - 成本极高：16线约$4000，64线约$40000，128线超$100000
   - 体积大、重量重：不适合小型机器人和无人机
   - 功耗高：旋转电机持续工作
   - 扫描线稀疏：相邻线束间有盲区，近距离小物体可能漏检
   - 垂直分辨率受限：由线束数量决定，增加成本高
   
   典型产品：
   - Velodyne VLP-16（16线，$4000-$8000）
   - Velodyne HDL-32E（32线，$15000-$30000）
   - Velodyne HDL-64E（64线，$40000-$80000）
   - Ouster OS1/OS2系列（64/128线）
   
   B. Livox固态激光雷达（非重复扫描）
   
   工作原理：
   - 微机电系统（MEMS）或棱镜扫描：无宏观旋转部件
   - 采用"非重复扫描"模式：每次扫描路径都略有不同
   - 随时间累积，扫描点逐渐覆盖整个视场（类似"填涂"）
   - 就像用画笔在画布上"涂抹"，每次笔触不同，最终填满画布
   
   扫描模式特点：
   - 非重复性：连续扫描中，激光打点位置不断变化
   - 逐步覆盖：初期稀疏，随时间增加点云密度提升
   - 覆盖率优化：0.1秒达到50%，0.5秒接近100%，1秒后覆盖极其均匀
   - 视场角固定：通常70°-100°FOV（前方半球）
   
   生活类比：
   - 想象你用喷漆罐给墙面喷涂
   - 机械雷达：用固定的梳子刷墙，每次刷的位置完全相同（梳齿间有间隙）
   - Livox雷达：用喷枪随机喷涂，每次喷的位置略有不同
   - 初期墙面稀疏（只有零星喷点）
   - 持续喷涂0.5秒后，墙面基本被均匀覆盖
   - 继续喷涂，覆盖越来越密，几乎没有遗漏
   
   非重复扫描的革命性优势：
   - 消除盲区：随时间推移，视场内每个位置都会被扫到
   - 均匀覆盖：不像机械雷达那样有固定的"稀疏扫描线"
   - 小物体检测：细小障碍物（如电线、树枝）在累积扫描中被捕获
   - 动态适应：静态场景扫描越久密度越高，动态环境实时更新
   
   优势：
   - 固态化：无宏观旋转部件，寿命长（理论10万小时级别）
   - 成本低：相同性能下，价格仅为机械雷达的1/10-1/5
     * Livox Mid-40: $599（对标Velodyne 16线$4000）
     * Livox Avia: $1999（对标Velodyne 32线$15000）
   - 体积小、重量轻：适合小型机器人、无人机、手持设备
   - 功耗低：无旋转电机，功耗通常<10W
   - 高覆盖率：非重复扫描在短时间内实现均匀覆盖
   - 高精度：通常<2cm测距误差
   - 抗振动：固态结构，适合越野、无人机等振动环境
   
   劣势：
   - 视场角有限：通常70°-100°（前方半球），不是360°
   - 需要运动或多雷达：单雷达无法覆盖后方，需机器人转向或多雷达组合
   - 算法适配：需要支持非重复扫描模式的SLAM算法（传统算法需改进）
   - 远距离性能：部分型号测距范围略短于高端机械雷达（但对机器人足够）
   - 生态成熟度：相比Velodyne等老牌厂商，第三方支持仍在发展
   
   典型产品：
   - Livox Mid-40（$599，70°圆形FOV，260m测距，260,000点/秒）
   - Livox Mid-70（$899，70°×77°矩形FOV）
   - Livox Avia（$1999，70°×77°，450m测距，240,000点/秒）
   - Livox Horizon（$1499，81°×25°车规级）
   - Livox HAP（高空性能版，1500m测距，用于测绘）
   
   C. 对比总结表
   
   | 特性             | 传统机械式雷达       | Livox固态雷达        |
   |-----------------|---------------------|---------------------|
   | 扫描方式         | 重复扫描（固定线束）  | 非重复扫描（动态填充） |
   | 覆盖范围         | 360°全方位          | 70-100°前方半球     |
   | 覆盖均匀性       | 固定线束间有盲区     | 随时间均匀覆盖      |
   | 小物体检测       | 可能漏检（线束间隙）  | 优秀（累积扫描捕获）  |
   | 机械寿命         | 2000-5000小时       | 理论10万小时        |
   | 成本（类似性能）  | $4000-$80000       | $599-$1999         |
   | 体积重量         | 大/重              | 小/轻              |
   | 功耗            | 高（15-30W）        | 低（5-10W）         |
   | 抗振动性         | 一般（旋转部件）     | 优秀（固态）         |
   | 算法生态         | 成熟完善            | 快速发展中          |
   | 典型应用         | 自动驾驶、高端机器人  | 中小型机器人、无人机 |
   
   D. 应用场景选择建议
   
   选择传统机械式雷达的场景：
   - 需要360°全方位实时感知（如自动驾驶车辆在路口）
   - 预算充足，对成本不敏感
   - 需要使用现有成熟SLAM算法（无需改进）
   - 超远距离探测需求（>200米）
   - 对机械寿命要求不高（商用车辆有维护周期）
   
   选择Livox固态雷达的场景：
   - 成本敏感项目（预算有限）
   - 机器人/设备有运动能力（可通过转向覆盖全方位）
   - 小型平台（无人机、手持设备、小型机器人）
   - 振动环境（越野机器人、无人机）
   - 长期部署（需要高寿命、低维护）
   - 需要高覆盖率检测小物体（如电线、树枝）
   - 前方场景重建（建筑测绘、矿山扫描）
   
   E. 技术趋势
   
   激光雷达的发展趋势：
   - 机械式雷达：逐步向高线束（128线+）和低成本方向发展
   - 固态雷达：快速成为主流，特别是机器人和低成本自动驾驶领域
   - 混合方案：部分厂商推出"半固态"方案（单轴旋转+MEMS扫描）
   - 芯片化：激光雷达OPA（光学相控阵）技术，完全无机械部件
   - 融合趋势：雷达+IMU+相机融合成为标配
   
   未来3-5年，固态雷达（如Livox）预计将占据中低端市场主导地位，
   而高端自动驾驶仍会使用机械式或混合式方案，直到固态技术完全成熟。
   
   2.2 什么是点云？用生活中的例子来理解
   - 想象你在黑暗中用激光笔扫描一个房间，每次激光打到物体表面，你就记录下这个点的位置
   - 扫描成千上万次后，这些点连在一起，就能"画"出房间的3D形状
   - 这些密密麻麻的点，就是"点云"
   
   类比理解：
   * 如果把照片比作"2D像素的集合"（每个像素记录颜色）
   * 那么点云就是"3D点的集合"（每个点记录空间位置）
   * 照片告诉你"那里有什么颜色"，点云告诉你"那里有什么物体，距离多远"
   
   2.2 PointCloud2数据为什么人类无法直接阅读？
   
   原因1：二进制编码存储
   - PointCloud2使用紧凑的二进制格式存储，不是文本格式
   - 就像你打开一个.jpg图片的原始数据，看到的是乱码
   - 例如：一个点的坐标(1.234, 5.678, 9.012)被编码成12个字节的二进制数据
   
   原因2：数据量巨大
   - 一帧点云可能包含50,000到100,000个点
   - 每个点有多个属性：x, y, z坐标、强度、时间戳等
   - 总数据量：50,000点 × 每点16字节 = 800KB（单帧！）
   - 即使转成文本，也是上万行数字，人眼无法理解
   
   原因3：缺乏直观的组织结构
   - 这些点是无序的3D空间散点，没有"从左到右、从上到下"的阅读顺序
   - 不像CSV表格那样有行列结构，可以逐行阅读
   - 就像把一个雕塑打散成沙粒，你无法从沙粒直接"看出"原来的雕塑形状
   
   2.3 通俗理解PointCloud2的数据结构
   
   可以把PointCloud2想象成一个"特殊的Excel表格"：
   
   | 点编号 | X坐标(m) | Y坐标(m) | Z坐标(m) | 强度 | 时间戳(ns) |
   |-------|---------|---------|---------|-----|-----------|
   | 1     | 1.234   | 2.456   | 0.123   | 100 | 123456789 |
   | 2     | 1.567   | 2.890   | 0.145   | 95  | 123456790 |
   | ...   | ...     | ...     | ...     | ... | ...       |
   | 50000 | 5.678   | 8.901   | 1.234   | 87  | 123789012 |
   
   但是：
   - 这个"表格"有5万到10万行
   - 数据以二进制紧密打包，不是文本
   - 坐标是相对雷达的，需要转换才能理解空间关系
   
   2.4 如何"看懂"点云数据？
   
   方法1：可视化工具（推荐）
   - 使用PCL Viewer、CloudCompare等工具
   - 将数万个点绘制成3D图像，人眼可以直观看到形状
   - 就像把散落的沙粒重新组装成雕塑
   
   方法2：转换为可读格式
   - 本脚本将PointCloud2转换为PCD文件（ASCII格式）
   - PCD文件虽然也很大，但至少可以用文本编辑器打开查看
   - 可以看到每个点的具体坐标值（尽管仍然难以理解整体形状）
   
   方法3：统计分析
   - 不直接看每个点，而是看统计信息
   - 如：点云中心位置、密度、范围、最近障碍物距离等
   - 就像不数人群中的每个人，而是看"大约有多少人"
   
   2.5 Livox雷达特点
   - 固态激光雷达，采用非重复扫描模式（覆盖更均匀）
   - 高分辨率和高精度的3D测距（误差通常<2cm）
   - 视场角广（通常70°-100°），适合机器人导航和环境感知
   
   2.6 在机器人中的应用
   - 3D环境建模：构建周围环境的三维模型（如建筑物、树木）
   - 障碍物检测：识别前方障碍物的位置和形状（如行人、车辆）
   - 地形分析：评估地面的可通行性（如楼梯、坡道、坑洼）
   - SLAM：同时定位与地图构建（机器人知道自己在哪里）
   
   2.7 PointCloud2消息的技术细节
   - header: 时间戳和坐标系信息
   - width × height: 点云的组织结构（无序点云height=1，width=点数）
   - fields: 每个点包含哪些数据（x,y,z,intensity等）
   - point_step: 一个点占用多少字节（如16字节）
   - row_step: 一行数据占用多少字节
   - data: 实际的点云数据（二进制格式，这就是人类无法阅读的部分）
   - is_dense: 所有点是否都有效（无NaN或Inf值）

数据保存策略：
============
- IMU数据保存为CSV文件：便于数据分析和可视化
- 点云数据保存为PCD文件：标准的点云存储格式
- 自动管理文件数量：超过10个文件时自动删除最旧的文件
- 文件命名包含时间戳：便于识别和管理

作者：SDK Demo
日期：2026-01-27
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os
import csv
from datetime import datetime
from pathlib import Path
import struct


class LivoxDataRecorder(Node):
    """
    Livox雷达数据记录节点
    
    功能：
    1. 订阅IMU和点云数据
    2. 实时显示数据统计信息
    3. 保存数据到本地文件系统
    4. 自动管理文件数量（最多保存10个）
    """
    
    def __init__(self):
        super().__init__('livox_data_recorder')
        
        # 创建数据保存目录
        self.script_dir = self._find_source_directory()
        self.data_dir = self.script_dir / 'livox_data'
        self.imu_dir = self.data_dir / 'imu'
        self.pointcloud_dir = self.data_dir / 'pointcloud'
        
        # 确保目录存在
        self.imu_dir.mkdir(parents=True, exist_ok=True)
        self.pointcloud_dir.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info(f'数据保存目录: {self.data_dir}')
        
        # 文件保存上限
        self.max_files = 50
        
        # 数据统计计数器
        self.imu_count = 0
        self.pointcloud_count = 0
        self.imu_save_count = 0
        self.pointcloud_save_count = 0
        
        # 订阅IMU话题
        self.imu_subscription = self.create_subscription(
            Imu,
            '/livox/imu',
            self.imu_callback,
            10
        )
        self.get_logger().info('已订阅话题: /livox/imu')
        
        # 订阅点云话题
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            10
        )
        self.get_logger().info('已订阅话题: /livox/lidar')
        
        # 创建定时器，每5秒显示一次统计信息
        self.timer = self.create_timer(5.0, self.display_statistics)
        
    def _find_source_directory(self):
        """智能定位源代码目录
        
        通过分析当前文件路径，自动定位到ROS2工作空间的src目录。
        适用于从install目录运行或直接从src目录运行的情况。
        
        Returns:
            Path: 源代码目录路径
        """
        current_file = Path(__file__).resolve()
        
        # 情况1: 如果当前在install目录运行
        # 路径类似: /path/to/workspace/install/pkg/lib/.../file.py
        # 需要找到workspace根目录，然后定位到src/pkg/pkg/
        if 'install' in current_file.parts:
            # 向上查找直到找到包含install目录的父目录（工作空间根目录）
            path = current_file
            while path.parent != path:
                # 检查当前目录是否同时包含install和src目录（工作空间根目录特征）
                if (path / 'install').exists() and (path / 'src').exists():
                    # 找到工作空间根目录
                    workspace_root = path
                    # 构造源代码路径: workspace/src/sdk_demo/sdk_demo/
                    source_dir = workspace_root / 'src' / 'sdk_demo' / 'sdk_demo'
                    if source_dir.exists():
                        return source_dir
                    break
                # 继续向上查找父目录
                path = path.parent
        
        # 情况2: 如果当前已经在src目录运行（开发时直接运行）
        # 路径类似: /path/to/workspace/src/pkg/pkg/file.py
        if 'src' in current_file.parts:
            # 当前文件的父目录就是我们要的目录（src/pkg/pkg/）
            return current_file.parent
        
        # 备用方案: 返回当前文件所在目录
        return current_file.parent
    
    def imu_callback(self, msg):
        """
        IMU数据回调函数
        
        IMU消息 (sensor_msgs/msg/Imu) 包含：
        - header: 时间戳和坐标系信息
        - orientation: 四元数表示的方向 (x, y, z, w)
        - angular_velocity: 角速度 (x, y, z) 单位: rad/s
        - linear_acceleration: 线性加速度 (x, y, z) 单位: m/s²
        - covariance: 协方差矩阵，表示测量的不确定性
        """
        self.imu_count += 1
        
        # 每接收100条IMU数据保存一次（IMU频率通常较高，避免频繁保存）
        if self.imu_count % 100 == 0:
            self.save_imu_data(msg)
            
        # 每收到10条数据显示一次（避免输出过多）
        if self.imu_count % 10 == 0:
            self.get_logger().info(
                f'IMU数据 #{self.imu_count} - '
                f'角速度: ({msg.angular_velocity.x:.3f}, '
                f'{msg.angular_velocity.y:.3f}, '
                f'{msg.angular_velocity.z:.3f}) rad/s, '
                f'加速度: ({msg.linear_acceleration.x:.3f}, '
                f'{msg.linear_acceleration.y:.3f}, '
                f'{msg.linear_acceleration.z:.3f}) m/s²'
            )
    
    def pointcloud_callback(self, msg):
        """
        点云数据回调函数
        
        PointCloud2消息 (sensor_msgs/msg/PointCloud2) 包含：
        - header: 时间戳和坐标系信息
        - height, width: 点云的组织结构（无序点云height=1）
        - fields: 每个点的字段定义（x, y, z, intensity等）
        - is_bigendian: 字节序
        - point_step: 单个点的字节数
        - row_step: 一行的字节数
        - data: 实际的点云数据（二进制格式）
        - is_dense: 是否所有点都有效（无NaN或Inf）
        """
        self.pointcloud_count += 1
        
        # 计算点云中的点数
        point_count = msg.width * msg.height
        
        # 每接收10帧点云保存一次（点云数据量大，不宜频繁保存）
        if self.pointcloud_count % 10 == 0:
            self.save_pointcloud_data(msg)
        
        # 显示点云信息
        self.get_logger().info(
            f'点云数据 #{self.pointcloud_count} - '
            f'点数: {point_count}, '
            f'尺寸: {msg.width}x{msg.height}, '
            f'坐标系: {msg.header.frame_id}'
        )
    
    def save_imu_data(self, msg):
        """
        保存IMU数据到CSV文件
        
        CSV格式便于在Excel或Python中进行数据分析
        每行包含一次IMU测量的所有信息
        """
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        filename = self.imu_dir / f'imu_{timestamp_str}.csv'
        
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # 写入表头
                writer.writerow([
                    'timestamp_sec', 'timestamp_nanosec',
                    'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                    'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                    'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'
                ])
                
                # 写入数据
                writer.writerow([
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ])
            
            self.imu_save_count += 1
            self.get_logger().info(f'IMU数据已保存: {filename.name}')
            
            # 管理文件数量
            self.manage_file_limit(self.imu_dir, '.csv')
            
        except Exception as e:
            self.get_logger().error(f'保存IMU数据失败: {str(e)}')
    
    def save_pointcloud_data(self, msg):
        """
        保存点云数据到PCD文件
        
        PCD (Point Cloud Data) 是PCL库的标准点云格式
        支持ASCII和二进制两种编码方式，这里使用ASCII便于查看
        """
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        filename = self.pointcloud_dir / f'pointcloud_{timestamp_str}.pcd'
        
        try:
            # 从PointCloud2消息中提取点数据
            points = []
            for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
                points.append(point)
            
            if len(points) == 0:
                self.get_logger().warn('点云为空，跳过保存')
                return
            
            # 写入PCD文件（ASCII格式）
            with open(filename, 'w') as f:
                # PCD文件头
                f.write('# .PCD v0.7 - Point Cloud Data file format\n')
                f.write('VERSION 0.7\n')
                f.write('FIELDS x y z\n')
                f.write('SIZE 4 4 4\n')
                f.write('TYPE F F F\n')
                f.write('COUNT 1 1 1\n')
                f.write(f'WIDTH {len(points)}\n')
                f.write('HEIGHT 1\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                f.write(f'POINTS {len(points)}\n')
                f.write('DATA ascii\n')
                
                # 写入点数据
                for point in points:
                    f.write(f'{point[0]} {point[1]} {point[2]}\n')
            
            self.pointcloud_save_count += 1
            self.get_logger().info(
                f'点云数据已保存: {filename.name} ({len(points)} 个点)'
            )
            
            # 管理文件数量
            self.manage_file_limit(self.pointcloud_dir, '.pcd')
            
        except Exception as e:
            self.get_logger().error(f'保存点云数据失败: {str(e)}')
    
    def manage_file_limit(self, directory, extension):
        """
        管理目录中的文件数量，确保不超过设定的上限
        
        策略：按文件修改时间排序，删除最旧的文件
        这样可以保证磁盘空间不会被无限占用
        
        参数：
            directory: 目录路径
            extension: 文件扩展名（如 '.csv' 或 '.pcd'）
        """
        try:
            # 获取所有指定类型的文件
            files = sorted(
                directory.glob(f'*{extension}'),
                key=lambda x: x.stat().st_mtime
            )
            
            # 如果文件数量超过上限，删除最旧的文件
            while len(files) > self.max_files:
                oldest_file = files.pop(0)
                oldest_file.unlink()
                self.get_logger().info(f'已删除旧文件: {oldest_file.name}')
                
        except Exception as e:
            self.get_logger().error(f'管理文件数量失败: {str(e)}')
    
    def display_statistics(self):
        """
        定时显示数据接收和保存的统计信息
        
        帮助用户了解节点的运行状态
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('数据统计信息:')
        self.get_logger().info(f'  IMU数据接收: {self.imu_count} 条')
        self.get_logger().info(f'  IMU数据保存: {self.imu_save_count} 个文件')
        self.get_logger().info(f'  点云数据接收: {self.pointcloud_count} 帧')
        self.get_logger().info(f'  点云数据保存: {self.pointcloud_save_count} 个文件')
        self.get_logger().info(f'  数据目录: {self.data_dir}')
        self.get_logger().info('=' * 60)


def main(args=None):
    """
    主函数：初始化ROS2节点并启动数据记录
    
    使用方法：
    1. 确保Livox雷达已连接并正常工作
    2. 运行此脚本: python3 17_lidar_demo.py
    3. 数据将自动保存到脚本所在目录的 livox_data 文件夹中
    4. 按 Ctrl+C 停止记录
    
    数据文件说明：
    - livox_data/imu/*.csv: IMU数据文件
    - livox_data/pointcloud/*.pcd: 点云数据文件
    - 文件名包含时间戳，便于识别
    - 自动保持最新的10个文件
    """
    rclpy.init(args=args)
    
    recorder = LivoxDataRecorder()
    
    try:
        recorder.get_logger().info('Livox数据记录节点已启动')
        recorder.get_logger().info('正在等待数据...')
        recorder.get_logger().info('按 Ctrl+C 停止记录')
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('用户中断，正在停止...')
    finally:
        # 显示最终统计
        recorder.get_logger().info('=' * 60)
        recorder.get_logger().info('最终统计:')
        recorder.get_logger().info(f'  总共接收IMU数据: {recorder.imu_count} 条')
        recorder.get_logger().info(f'  总共保存IMU文件: {recorder.imu_save_count} 个')
        recorder.get_logger().info(f'  总共接收点云数据: {recorder.pointcloud_count} 帧')
        recorder.get_logger().info(f'  总共保存点云文件: {recorder.pointcloud_save_count} 个')
        recorder.get_logger().info('=' * 60)
        
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
