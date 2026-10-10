# 复现与技术说明

[返回项目首页](../README.md)

本文说明游戏插件、ROS 桥接程序的实现与运行方式。源码入口：[游戏侧插件](../cs16_sensors_udp.sma)、[ROS 桥接脚本](../cs_udp_sensors_bridge.py)。

## 关键实现

### 虚拟 LiDAR 与坐标转换

游戏插件按方位角和俯仰角调用 `trace_line`，通过场景碰撞结果构造扫描点。默认参数为 32 线、水平步长 2°、扫描间隔 0.07 s，最大量程为代码中的 50 m。扫描间隔为调度配置值，最大量程通过源码常量设置。

GoldSrc 单位按 `0.0254 m/unit` 换算为米。机体系约定为 X 向前、Y 向左、Z 向上；射线先从机体系变换到世界系，命中点再投回机体系。每个点携带 `ring` 与按方位角生成的帧内相对 `time`。

### 模拟 IMU

插件从玩家视角基向量生成姿态四元数，以相邻姿态估算体坐标角速度，以速度差分估算线加速度，并包含重力项、限幅、低速抑制与低通处理。`imu_rate_hz` 默认为 100，调度设置范围为 10-200 Hz。

这是一种从游戏运动状态构造的近似传感器输入。地面、水中及低速处理带有游戏场景假设，不等同于真实 IMU 的噪声、偏置与物理响应。

### UDP 与 ROS 消息适配

点云通过 `FRAME_BEGIN` / `FRAME_END` 标记组帧，支持按包大小合并多行点；默认包大小预算为 1200 字节。ROS 侧解析点数据并发布 `PointCloud2`，支持异常行统计、距离过滤，以及可选的帧超时发布。

点云默认使用 32 字节的 XYZIRT 字段布局：`x/y/z`、`intensity`、`ring`、`time` 及对齐填充。模拟强度固定为 1.0。IMU 单独接收并发布为 `sensor_msgs/Imu`。

## 运行准备

1. 准备可运行本地服务器的 CS1.6 环境与 AMX Mod X，启用插件所需的 `engine`、`fakemeta`、`sockets` 模块及对应编译包含文件。
2. ROS 侧准备 ROS Noetic、Python 3，以及 `rospy`、`std_msgs`、`sensor_msgs`；建图与可视化需要另外配置 FAST-LIO2 和 RViz。
3. 游戏端与 ROS 接收端之间允许 UDP 9999、10000 端口通信。

### 1. 配置并加载游戏插件

- 将 `cs16_sensors_udp.sma` 中的 `SERVER_IP` 和 `IMU_IP` 都改为 ROS 接收机器的可达地址；只修改其中一处会造成一路传感器没有数据。
- 使用与本机 AMX Mod X 配套的编译工具编译，生成 `cs16_sensors_udp.amxx`。
- 将编译结果放入 `addons/amxmodx/plugins/`，在 `addons/amxmodx/configs/plugins.ini` 中添加 `cs16_sensors_udp.amxx`，然后加载或重启本地服务器。
- 当前 `find_carrier_player()` 只选择玩家 1，且默认要求玩家存活。进入本地地图后，确认该玩家存在并存活。

采样参数对应以下服务器 cvar，示例值与插件默认值一致：

```text
lidar_n_scan 32
lidar_deg_step 2
lidar_interval_s 0.07
imu_rate_hz 100
```

LiDAR 和 IMU 的循环间隔在各自启动函数中读取；运行中修改间隔后，应重新加载插件使调度生效。

### 2. 启动 ROS 桥接

在已加载 ROS Noetic 环境的终端中启动 ROS master：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

另开一个终端，在仓库目录运行：

```bash
source /opt/ros/noetic/setup.bash
python3 cs_udp_sensors_bridge.py \
  _lidar_ip:=0.0.0.0 _lidar_port:=9999 \
  _imu_ip:=0.0.0.0 _imu_port:=10000 \
  _lidar_topic:=/velodyne_points _imu_topic:=/cs/imu \
  _xyz_only:=false
```

桥接脚本启动日志会显示监听地址、端口、frame 和 topic。游戏开始发送数据后，检查：

```bash
rostopic list
rostopic hz /velodyne_points
rostopic hz /cs/imu
rostopic echo -n 1 /velodyne_points/fields
```

### 3. 接入 FAST-LIO2 与 RViz

在自己的 FAST-LIO2 环境中配置输入点云与 IMU topic，检查预处理器是否识别 `ring` 和 `time`，并使扫描线数、时间单位、传感器坐标与外参设置一致。

FAST-LIO2 工程、启动文件与外参配置需要单独准备。先确认 ROS 侧两路数据有效，再启动外部建图进程。也可以先在 RViz 查看 `cs_lidar` 坐标系下的原始点云，检查轴向与尺度。

## 仿真与时间约定

- 扫描在一次 `do_scan()` 调用中使用同一玩家姿态；点内时间按方位角合成，不能当作真实逐点运动采样或精确去畸变的验证基准。
- 点云帧头与 IMU 消息使用 ROS 接收时刻。当前协议没有传输统一的源端绝对时间，也没有实现跨机器时钟同步。
- 游戏侧 `imu_fix_rxpi` 默认启用，ROS 侧另有默认关闭的 `apply_rz_pi`。这些是不同轴向的修正开关，不能互相替代；接入算法前需检查姿态、角速度、加速度和外参的一致性。
- UDP 发送没有序号、重传或完整性校验。帧标记可帮助组帧，不能保证检测所有丢点；桥接端的 `frame_timeout` 默认是 0，即关闭超时发布。
- 仓库展示的是虚拟场景建图联调，不提供真实机器人部署或定位精度基准结论。

## 常见排查

| 现象 | 优先检查 |
| --- | --- |
| 点云和 IMU 都没有数据 | 游戏端目标地址、两路 UDP 端口、防火墙、插件是否成功加载、玩家 1 是否存活 |
| 只有一路传感器有数据 | `SERVER_IP` 与 `IMU_IP` 是否都已修改，对应接收端口是否一致 |
| 点云出现镜像、旋转或尺度异常 | 游戏单位到米的换算、FLU 坐标约定、姿态修正开关及算法外参 |
| ROS 有数据但建图不正常 | FAST-LIO2 预处理点型、`ring/time` 字段与时间单位、IMU 轴向及时间处理 |
| 帧不完整或解析器报异常行 | 桥接日志、UDP 包大小、丢包与两端文本格式；需要时启用 `_frame_timeout` 兜底 |
