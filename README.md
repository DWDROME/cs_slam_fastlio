# CS1.6 → ROS：基于 GoldSrc 的虚拟 LiDAR / IMU 与 SLAM 建图

**让经典 FPS 游戏场景成为机器人 SLAM 的虚拟试验场。**

[![CS1.6 游戏画面与 FAST-LIO2 建图结果同屏展示](assets/cs16-fastlio2-demo.jpg)](https://www.bilibili.com/video/BV1hYpWztEcS/)

*演示视频约 2:13 处截图：左侧为 CS1.6 场景与桥接终端，右侧为 RViz 中的建图结果。点击图片观看视频。*

[**▶ 观看建图演示（Bilibili）**](https://www.bilibili.com/video/BV1hYpWztEcS/) · [复现与技术说明](docs/technical-notes.md)

本项目在《Counter-Strike 1.6》的 GoldSrc 引擎中实现虚拟 LiDAR 与 IMU，通过 UDP 将传感器数据实时传输至 ROS Noetic，并接入 FAST-LIO2 进行建图联调与可视化。

利用游戏引擎已有的地图几何、碰撞检测和玩家运动状态，构建从游戏场景到机器人感知算法的完整数据通路。

## 系统架构

```text
CS1.6 / GoldSrc
  └─ AMX Mod X：虚拟 LiDAR + IMU
          │ UDP：点云 9999 / IMU 10000
          ▼
ROS Noetic：cs_udp_sensors_bridge.py
  ├─ sensor_msgs/PointCloud2 → /velodyne_points
  └─ sensor_msgs/Imu         → /cs/imu
          ▼
FAST-LIO2 → RViz
```

系统分为游戏侧传感器插件与 ROS 侧桥接程序；FAST-LIO2 单独配置接入。

## 核心实现

### 游戏侧：[cs16_sensors_udp.sma](cs16_sensors_udp.sma)

- **虚拟 LiDAR**：通过 `trace_line` 对场景碰撞几何进行多线射线扫描，可配置扫描线数、角分辨率和扫描间隔。
- **虚拟 IMU**：根据玩家姿态与运动状态生成四元数、角速度和线加速度，结合滤波与限幅处理。
- **坐标转换**：在 GoldSrc 世界坐标与机器人机体系之间变换，将游戏单位统一换算为米。
- **UDP 传输**：批量打包点云，以 `FRAME_BEGIN` / `FRAME_END` 标记帧边界；IMU 使用独立端口发送。

### ROS 侧：[cs_udp_sensors_bridge.py](cs_udp_sensors_bridge.py)

- **标准消息**：发布 `sensor_msgs/PointCloud2` 与 `sensor_msgs/Imu`，供 ROS 工具及建图进程使用。
- **点云适配**：采用 XYZIRT 字段布局，保留 `ring`、`time` 等信息。
- **接收处理**：支持帧组装、异常数据统计、距离过滤和可选的超时发布。

## 默认配置

| 参数 | 默认值 |
| --- | --- |
| LiDAR 扫描线数 | 32 |
| 水平角分辨率 | 2° |
| 扫描间隔 | 0.07 s |
| 最大量程（源码常量） | 50 m |
| IMU 采样率配置 | 100 Hz |
| 点云 / IMU UDP 端口 | 9999 / 10000 |
| 点云 / IMU topic | `/velodyne_points` / `/cs/imu` |
| ROS 发行版 | Noetic |

扫描间隔与 IMU 采样率为调度配置值。

## 从游戏场景到建图输入

这个项目将射线扫描、姿态与运动状态估算、坐标系约定、UDP 分帧和 ROS 消息适配串联起来，让 CS1.6 地图成为可以接入机器人 SLAM 系统的虚拟环境。

工程中的主要工作包括：

- 将游戏碰撞结果转换成具有米制尺度与机体系约定的多线点云。
- 为建图输入组织 `ring`、帧内时间与模拟惯性数据。
- 打通游戏端、网络接收、ROS 发布与 FAST-LIO2 / RViz 的联调链路。

## 复现入口

准备 CS1.6 + AMX Mod X 与 ROS Noetic，设置游戏插件的接收端 IP，加载插件并运行桥接脚本，再接入自己的 FAST-LIO2 配置。

具体的依赖、启动命令、坐标与时间约定、常见问题见 [复现与技术说明](docs/technical-notes.md)。输入由游戏实时生成，无需额外下载 rosbag 或数据集。

## 项目定位

本仓库提供游戏侧传感器插件与 ROS 桥接程序，FAST-LIO2 为外部建图组件。虚拟传感器采用简化模型，适合数据通路调试、算法接入探索与建图演示；模型及时间处理约定见[技术说明](docs/technical-notes.md#仿真与时间约定)。
