# ROBOCON MID-360 从零技术导览

这份导览面向第一次接触 ROS 2、点云、惯导和机器人仿真的读者。它不把本仓库理解成一条“启动 Gazebo 后就会自动打球”的脚本，而是把它拆成一条可以逐段检查的数据与控制链：**机器人和传感器产生消息，状态估计把消息变成位姿，地图模块把位姿和点云变成空间表示，感知模块把相机结果变成可用条件，控制模块在条件满足时发布动作。**

仓库的主要入口是 `README.md`，源码集中在 `src/`，而这篇文档解释每个子包为什么存在、输入和输出是什么，以及这些模块怎样连成双机器人篮球演示。

## 1. 先建立全局图像

可以把系统看成一条从光线到动作的流水线：

```text
Gazebo 场地、两台机器人、篮球
    │
    ├── Livox 风格激光雷达 ──> /robot1/livox/lidar ──> FAST-LIO2
    ├── IMU ───────────────> /robot1/livox/imu ─────┘
    ├── RGB-D 相机 ─────────> RGB、depth、camera_info
    └── 机器人底盘状态 ─────> robot_state_publisher / TF
                                      │
                         /cloud_registered、里程计、TF
                                      │
              点云地图 / PCD / 二维占据栅格 / 固定地图定位
                                      │
                pose_valid、map_locked、target_valid、preflight_ready
                                      │
                       姿态目标桥和比赛状态机
                                      │
               /robot1/cmd_vel_chassis、/robot2/cmd_vel_chassis
```

这里有两个很重要的工程观念。

第一，**消息不是状态**。例如，收到一帧点云并不等于已经知道机器人在地图中的位置；收到一个目标框也不等于允许投篮。仓库因此把输入新鲜度、地图锁定、目标稳定性和动作反馈做成显式条件。

第二，**坐标系不是可有可无的标签**。相机、IMU、雷达、车体和地图都必须通过 TF 变换树对齐。只要 `map -> odom -> base_link` 中有一段错位，点云会漂浮、栅格会拖影，后续“向目标移动”的方向也会错误。

## 2. ROS、ROS 2 与本仓库的通信模型

### 2.1 ROS 2 到底解决什么

ROS 2 不是控制算法，而是一套机器人软件中间件。它解决的是不同进程如何发现彼此、如何交换消息、如何声明参数、如何启动和如何记录诊断。在本项目里：

- `rclpy` 用于 Python 节点，例如目标门、位姿桥和篮球演示控制器；
- `rclcpp` 用于 C++ 节点，例如点云质量地图、二维栅格和 ICP 扫描匹配；
- `ros2 launch` 用 Python launch 文件把多个节点按同一组参数启动；
- `colcon build --symlink-install` 把各个 ROS 2 包构建为一个 workspace；
- `source install/setup.bash` 让当前 shell 找到构建后的节点、launch 和资源文件。

ROS 是这套生态的名称；ROS 1 与 ROS 2 的运行方式不同。ROS 1 通常依赖中心化的 `roscore`/master 做节点发现，ROS 2 基于 DDS（Data Distribution Service）进行分布式发现和发布订阅。因而 ROS 2 节点在同一网络和同一 Domain 内可以直接发现彼此，不需要单独启动 master。本项目使用 ROS 2 Humble，并通过 `ROS_DOMAIN_ID` 隔离不同实验会话：同一 Domain 的 Gazebo、RViz、FAST-LIO2 和控制器能互相发现，不同 Domain 则不会串到一起。录制命令中使用的 `ROS_DOMAIN_ID=192` 就是这层隔离的具体值。

一个 ROS 2 节点通常包含四类接口：

| 接口 | 作用 | 本仓库中的例子 |
|---|---|---|
| Topic | 持续、异步的数据流 | `/livox/lidar`、`/cloud_registered`、`/mid360/occupancy_grid` |
| Service | 一次请求对应一次应答 | `/gazebo/set_entity_state` 用于演示中的篮球实体状态更新 |
| Parameter | 启动时或运行时配置节点 | `scan_voxel`、`max_depth`、`min_confidence` |
| TF | 持续发布坐标系之间的刚体变换 | `map -> odom -> base_link -> imu_link -> lidar_mid360` |

### 2.2 Topic、消息和 QoS

Topic 可以理解为有名字的数据通道；消息类型定义了通道内每一条数据的字段。例如：

- `sensor_msgs/msg/Imu` 有角速度、线加速度和时间戳；
- `nav_msgs/msg/Odometry` 有位置、姿态、线速度和角速度；
- `sensor_msgs/msg/PointCloud2` 是通用点云容器；
- `livox_ros_driver2/msg/CustomMsg` 是 Livox 风格的专用点云消息；
- `sensor_msgs/msg/Image` 承载 RGB 图像或深度图；
- `std_msgs/msg/Bool` 承载简单状态，例如 `map_locked`；
- `diagnostic_msgs/msg/DiagnosticArray` 承载可读的运行健康信息。

传感器通常使用 `qos_profile_sensor_data`。它的设计重点是“尽量取最新数据，而不为旧帧阻塞系统”；定位和视觉链路一般更关心当前帧是否新鲜，而不是等待一帧已经过期的旧数据。项目的输入守卫也会检查时间戳和数据本身，而不是只相信 topic 存在。

### 2.3 Namespace：为什么双车不会串话

两辆车使用相同的模型、相同的传感器插件，却不能共用 topic 名。双车 launch 通过 namespace 把它们隔离为：

```text
/robot1/livox/lidar
/robot1/livox/imu
/robot1/cmd_vel_chassis
/robot1/simulated_rgbd_camera/...

/robot2/livox/lidar
/robot2/livox/imu
/robot2/cmd_vel_chassis
/robot2/simulated_rgbd_camera/...
```

`gazebo_mid360_dual.launch.py` 为每台车调用同一个 Xacro 机器人描述，同时传入各自的雷达、IMU、底盘和相机 topic。当前双车地图演示由 robot1 的雷达接入 FAST-LIO2；robot2 仍保留独立 RGB-D、IMU、雷达和底盘接口，供协作和感知展示使用。

## 3. 机器人模型、URDF、Xacro 与 Gazebo

### 3.1 URDF 和 Xacro

URDF 描述机器人由哪些 link 和 joint 构成：`base_link` 是车体参考系，四个轮子、IMU、雷达和相机通过 joint 连接到车体。Xacro 是 URDF 的宏语言，允许在 launch 时传入参数并生成完整 XML，避免为 robot1 和 robot2 维护两套近乎相同的模型。

`src/robocon_mid360_simulation/urdf/robocon25_mid360_robot.xacro` 中的关键结构包括：

- 四个连续关节轮与 `libgazebo_ros_planar_move.so` 平面移动插件；
- `imu_link` 和 `libgazebo_ros_imu_sensor.so`，输出 `/livox/imu`；
- `lidar_mid360` 与自定义 Livox 多射线插件，输出 `/livox/lidar`；
- 可选 `camera_rgbd_link` 与 `libgazebo_ros_camera.so`，输出彩色图、深度图和相机内参；
- 可选 Gazebo pose topic，便于定位链路的对齐与误差分析；
- 球盘、进球导向件、发射器、传感器桅杆等几何部件，使篮球任务中的功能位置可见。

在 ROS 中，URDF 由 `robot_state_publisher` 转换为 TF。Gazebo 同时加载碰撞、质量、惯性和插件，这样同一个模型既有可视化树，也能参与物理和传感器更新。

### 3.2 Gazebo Classic 的角色

Gazebo Classic 在此提供三件事：世界几何、碰撞/动力学和传感器插件。`robocon25_candidate.world` 提供球场、篮筐和篮球；`spawn_entity.py` 在指定位置放入两台机器人。双车 launch 中 robot1 起点约为 `(-3.0, -1.35, 0.18)`，robot2 起点约为 `(2.2, 1.10, 0.18)`。

场景启动时会看到 `gzserver` 和可选的 `gzclient`：前者负责物理和世界状态，后者是可见 GUI。ROS 节点通过 `gazebo_ros` 插件与 server 交换消息。

## 4. MID-360、点云与 Livox CustomMsg

### 4.1 雷达一帧数据包含什么

激光雷达通过发射光束并测量返回时间得到距离。把距离和每束光的方向结合起来，就得到雷达坐标系中的三维点 `(x, y, z)`；反射率可作为强度信息。连续多帧点云加上位姿，就可以重建环境结构。

本项目采用 Livox 风格扫描模式，而不是把雷达抽象为固定栅格相机。关键输入 topic 是：

```text
/robot1/livox/lidar    livox_ros_driver2/msg/CustomMsg
/robot1/livox/imu      sensor_msgs/msg/Imu
```

`CustomMsg` 的两个时间字段格外关键：

- `timebase`：一包点云的基准时间；
- `offset_time`：包内每个点相对 `timebase` 的时间偏移，单位为纳秒。

机器人运动期间，不同点并非在同一时刻采到。若只保留消息头时间，算法会把一帧内扫到的所有点当成同步点，快速移动时墙面会弯曲或重影。FAST-LIO2 使用包内时间偏移与 IMU 轨迹对点做去畸变，因此项目的接口合同明确要求保留每一个 `offset_time`。

### 4.2 自定义 Livox 插件如何生成消息

`livox_points_plugin.cpp` 读取 MID-360 扫描模式 CSV，在 Gazebo 中创建射线并以碰撞结果构造点。它同时发布：

- `/robot1/livox/lidar`：FAST-LIO2 使用的 `CustomMsg`；
- `/robot1/livox/lidar/pointcloud`：可选的 `PointCloud2` 镜像，便于通用工具查看。

插件把 Gazebo 仿真时间转换为 `timebase`，并在单个更新窗口里为每个有效回波计算单调的 `offset_time`。无回波样本不会写入 Livox 包。`lidar_samples` 决定每包尝试的扫描样本数，`lidar_downsample` 决定抽样间隔；两者共同决定点密度、CPU 负载和消息大小。

双车 launch 提供密度保护：当样本数很高且下采样不足时，启动器会阻止在 WSL 中一次分配过多射线。正常可视化采用 `lidar_samples:=30000`、`lidar_downsample:=1`；更高密度应把样本数、下采样、帧率和内存一起考虑，而不是只提高一个数字。

### 4.3 输入守卫：先确认输入健康，再让下游使用

`mid360_localization_contract/input_validation.py` 会统计：

- 点数、有限点数和非有限点数；
- `offset_time` 是否单调；
- 包内时间跨度；
- IMU 的时间戳是否为零或倒退；
- IMU 的加速度、角速度是否有限；
- 点云和 IMU 是否在预期的新鲜度窗口内。

对应 ROS 节点发布 `/mid360/input_valid` 和诊断数组。这样，下游不需要从复杂的点云内容中猜测“当前传感器是否可用”，而是消费一个带诊断依据的布尔条件。

## 5. IMU：为什么点云定位必须关心惯性

IMU（惯性测量单元）通常输出两组量：

- 角速度 `ω = (ωx, ωy, ωz)`：描述转得多快；
- 线加速度 `a = (ax, ay, az)`：描述加速及重力方向的合成量。

如果只积分 IMU，误差会快速累积，因为陀螺和加速度计都存在偏置；如果只依靠一帧点云，快速旋转或特征稀少时又不够稳定。FAST-LIO2 将两者组合：IMU 在相邻雷达帧之间提供连续运动预测，点云配准提供几何校正。

常见的状态可以概念化为：

```text
x = {R, p, v, bg, ba, g}
R: 姿态旋转矩阵或等价四元数
p: 位置
v: 速度
bg、ba: 陀螺与加速度计偏置
g: 重力方向/大小
```

在时间间隔 `Δt` 内，预测步骤用陀螺更新姿态、用去重力后的加速度更新速度和位置；随后用点到地图几何残差修正该预测。这个组合正是“LiDAR-Inertial Odometry”的含义。

## 6. FAST-LIO2：从雷达和 IMU 到局部里程计

### 6.1 它在系统里的位置

FAST-LIO2 是本项目的 LiDAR-IMU 前端。它输入 Livox `CustomMsg` 和 IMU，输出局部里程计、注册点云以及可选地图相关话题。双车地图 launch 中，robot1 的输入被重映射为：

```text
/livox/lidar  -> /robot1/livox/lidar
/livox/imu    -> /robot1/livox/imu
/Odometry     -> /mid360/local_odometry
```

也就是说，FAST-LIO2 的内部话题名称被保留，而外部系统统一读取 `/mid360/local_odometry`。这降低了上游算法替换时对比赛控制和地图模块的影响。

### 6.2 FAST-LIO2 的核心思路

FAST-LIO2 的典型流程可以按四步理解：

1. **IMU 预测**：从上一状态积分到当前雷达扫描时刻；
2. **逐点去畸变**：利用每个点的 `offset_time` 将点转换到一致时刻；
3. **最近邻几何约束**：在增量地图中找局部平面或几何邻域，形成点到平面的残差；
4. **迭代误差状态更新**：不断线性化并更新状态，使几何残差与不确定性共同收敛。

FAST-LIO2 使用增量 kd-tree 管理地图邻域，因此能够在地图持续扩展时快速查询近邻。仓库保留的 `vendor_fast_lio` 是该前端的 ROS 2 集成来源；项目自己的职责是把消息、帧、诊断和控制接口规范化。

### 6.3 当前配置如何读

`fast_lio_simulation.yaml` 和 `fast_lio_mapping_simulation.yaml` 有两类用途。

基础配置中：

- `lidar_type: 1`：选择 Livox 消息路径；
- `scan_line: 4`、`scan_rate: 10`、`timestamp_unit: 3`：描述输入扫描节奏和时间单位；
- `blind: 0.5`：过滤过近回波；
- `point_filter_num: 3`：降低前端点数；
- `filter_size_surf: 0.5`、`filter_size_map: 0.5`：使用 0.5 m 体素滤波控制运算量；
- `max_iteration: 3`：限制每次更新的迭代次数；
- `time_sync_en: false`：不启用软件时间同步；
- `extrinsic_est_en: false`：不在本配置中在线估计雷达-IMU 外参。

地图叠加配置额外开启 `scan_publish_en: true` 和 `dense_publish_en: true`。这是因为后面的质量地图需要较密的 `/cloud_registered`，而不是只依赖稀疏表面点。

### 6.4 里程计不是全局地图坐标

FAST-LIO2 的局部里程计适合连续跟踪，但初始原点往往是启动位置，不天然等于比赛场地坐标。因此系统把“局部运动”和“全局地图定位”分开：

```text
FAST-LIO2 局部估计      /mid360/local_odometry
固定地图锚定后的估计    /mid360/localization_odometry
```

这也是为什么后续会出现 `odom` 与 `map` 两层坐标系。

## 7. TF、四元数与固定地图定位

### 7.1 坐标树的职责划分

项目的标准 TF 树是：

```text
map -> odom -> base_link -> imu_link -> lidar_mid360
```

含义如下：

- `map`：固定场地地图坐标；
- `odom`：局部连续里程计坐标；
- `base_link`：机器人车体中心；
- `imu_link`、`lidar_mid360`：传感器安装位置。

`mid360_pose_bridge` 负责将 FAST-LIO2 位姿转成唯一的 `odom -> base_link`；`mid360_map_odom_anchor` 负责 `map -> odom`。单一所有者能防止两个节点同时发布同一段 TF 而相互覆盖。

### 7.2 四元数为什么要转成 yaw

ROS 的姿态通常存为四元数 `(x, y, z, w)`。平面篮球底盘主要关注偏航角 yaw，但不能直接把四元数的 `z` 当作 yaw。项目的 `quaternion_to_yaw` 先归一化，再使用：

```text
yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
```

把目标 `(target_x, target_y)` 与当前位置 `(x, y)` 相减，可得到：

```text
distance      = hypot(target_x - x, target_y - y)
target_heading = atan2(target_y - y, target_x - x)
heading_error  = wrap(target_heading - yaw)
```

`robocon_pose_command_bridge` 将这些量封装为版本化 JSON 姿态目标。发布前必须同时满足位姿存在、`pose_valid` 为真、`map_locked` 为真，并且位姿年龄未超过 `max_pose_age_sec`。

### 7.3 PCD 与 ICP 固定地图定位

PCD 是 Point Cloud Data 文件，常用于保存点云地图。`mid360_map_localizer` 读取地图 PCD，同时订阅局部点云和 `/initialpose`。其 ICP（Iterative Closest Point）思路是：

1. 使用 `/initialpose` 作为初值；
2. 对当前扫描和地图各自体素下采样；
3. 为扫描点寻找地图近邻；
4. 计算刚体变换，使两组点的对应距离减小；
5. 迭代到收敛，再检查质量门；
6. 接受变换后发布 `map -> odom` 修正。

代码中的典型门槛包括 `voxel_leaf_size=0.12 m`、`max_fitness_score=0.40`、最大平移跳变 `0.80 m`、最大 yaw 跳变 `25°`。ICP 不收敛、fitness 不合格或候选修正突变时，不应将该修正写进锚点。

定位状态机位于 `mid360_localization_contract/tracking.py`：

```text
UNINITIALIZED  没有锚点
RELOCALIZING   已收到初始位姿或请求重新定位，等待有效校正
TRACKING       锚点已接受，局部里程计新鲜且有效
LOST           已有锚点，但当前位姿或里程计不再满足条件
```

只有 `TRACKING` 才让 `/mid360/map_locked` 为真。

## 8. 从注册点云到地图：3D 质量地图、PCD 和 2D 占据栅格

### 8.1 `/cloud_registered` 是什么

注册点云是经过当前位姿变换后的点云。它与原始雷达坐标系点云不同：前者可以被累积到统一地图中，后者只描述传感器这一瞬间看到的局部结构。RViz 中“点云越来越完整”的视觉效果来自连续注册和累积。

`mid360_map_tools/fastlio_cloud_mapper_node.cpp` 接收 `/cloud_registered`，处理流程为：

1. 按距离和高度范围筛点；
2. 用 `scan_voxel` 体素下采样单帧扫描；
3. 用半径邻居数移除离群点；
4. 根据里程计速度、位移跳变、Z 方向跳变和扫描-地图重叠度评估帧；
5. 把接受的点写入 `map_voxel` 体素哈希表；
6. 发布过滤扫描、可靠扫描和累积地图；
7. 按需保存 PCD 与 YAML 元数据。

常用默认值为 `scan_voxel=0.08 m`、`map_voxel=0.12 m`、半径过滤 `0.20 m` 且至少 2 个邻居。质量门还包括最大速度 `2.5 m/s`、最大短时位移跳变 `0.55 m`、最大 Z 跳变 `0.35 m`、以及在地图积累后检查扫描-地图重叠。

### 8.2 为什么还要做二维栅格

3D 点云适合看立体结构，但平面导航和快速查看更适合 2D 占据栅格。`pointcloud_occupancy_grid_node.cpp` 将高度范围内的点投影到网格：

```text
世界坐标点 (x, y, z)
      │
      ├─ 过滤 z_min 到 z_max
      ├─ 根据 resolution 映射到格子索引 (ix, iy)
      ├─ 端点格子增加 occupied log-odds
      └─ 从机器人位置到端点做 raycast，路径格子降低 free log-odds
```

每个格子维护一个有界对数几率值。默认配置中，命中占据点增加 `12`，射线经过的自由格降低 `4`，高于 `occupied_threshold=45` 的格子显示为占据，低于 `free_threshold=-12` 的格子显示为自由，介于两者之间显示为未知或不确定。`occupied_cell_inflation` 可把障碍附近格子扩张，给导航留下安全余量。

双车可视化 launch 使用 `0.08 m` 分辨率、`24 m x 12 m` 画布，原点 `(-12, -6)`，每帧发布一次。RViz 的二维 PCD/占据图展示的正是这条降维链，而不是一张静态图片。

## 9. RGB-D 与深度图：颜色、距离和可视化并不是同一件事

### 9.1 RGB-D 相机输出

启用 `enable_rgbd:=true` 后，Xacro 中的深度传感器插件发布：

```text
/robotN/simulated_rgbd_camera/image_raw
/robotN/simulated_rgbd_camera/depth/image_raw
/robotN/simulated_rgbd_camera/camera_info
```

彩色图像可用于检测篮筐、篮板或篮球；深度图每个像素记录沿相机光轴的距离；`camera_info` 给出焦距、主点和图像几何。知道像素 `(u, v)`、深度 `Z` 和内参 `(fx, fy, cx, cy)` 后，可反投影为：

```text
X = (u - cx) Z / fx
Y = (v - cy) Z / fy
Z = depth(u, v)
```

这让二维检测框可以与三维距离结合。例如，检测器报告篮筐框，同时从其中心或有效区域取得深度，就能形成“类别 + 置信度 + 距离”的观测。

### 9.2 为什么原始深度图常常看起来是黑的

深度的常见编码是 `32FC1`（单位通常为米的浮点数）或 `16UC1`（毫米整数）。它们不是直接给人看的 RGB 图；如果显示器按 0--255 灰度直接解释很小的浮点字节，画面会几乎全黑。

项目的 `depth_visualizer.py` 专门做显示转换：

1. 识别 `32FC1` 或 `16UC1`；
2. 将 16 位毫米值换算为米；
3. 忽略非有限值和小于最小距离的像素；
4. 将 `[min_depth, max_depth]` 映射到 `mono8`；
5. 输出 `/robotN/simulated_rgbd_camera/depth/image_visualized`。

映射方向采用“近亮远暗”：

```text
gray = 255 * (max_depth - clamp(depth, min_depth, max_depth))
             / (max_depth - min_depth)
```

双车 launch 对两个机器人各启动一个可视化器，使用 `min_depth=0.2 m`、`max_depth=12.0 m`。只想录相机，不想显示 RViz 的三维空视口时，可运行：

```bash
bash tools/run_rgbd_image_view.sh robot1 depth
bash tools/run_rgbd_image_view.sh robot2 rgb
```

该脚本用 `rqt_image_view` 打开指定 topic。

### 9.3 目标检测和目标门

`robocon_camera_yolo_adapter` 预留了 YOLO 类检测入口，配置中包含阈值 `conf_thres=0.60`、`iou_thres=0.45` 与目标类别 `basket`、`hoop`、`backboard`。更关键的是，比赛控制不直接消费一个原始检测框，而通过 `robocon_perception_adapter/target_gate.py` 统一验证 JSON 观测：

```json
{
  "confidence": 0.92,
  "distance_m": 3.1,
  "stable": true,
  "observed_at_ns": 123456789,
  "target_type": "hoop"
}
```

目标门检查置信度、距离范围、观测年龄和稳定性。当前默认条件为置信度至少 `0.70`、距离在 `1.0--10.0 m`、年龄不超过 `0.50 s` 且 `stable=true`。通过后发布 `/robocon/perception/target_valid`，并发布带原因、年龄和数值的状态 JSON。

这种设计把“看见了一个框”和“允许执行投篮动作”分成两个层级，便于替换检测器，也便于定位失败原因。

## 10. 比赛控制：从布尔条件到状态机和动作

`robocon_game_supervisor` 把比赛流程建模为显式状态机：

```text
BOOT -> PREFLIGHT -> WAIT_START -> ACTIVE
                         │             │
                         ├-> PAUSED    ├-> RECOVERY
                         └-> FAULT     ├-> ESTOP
                                       └-> FINISHED
```

机器人任务状态进一步描述传球和投篮过程：

```text
IDLE
 -> RECEIVER_READY
 -> PASS_ARMED
 -> PASS_EXECUTED
 -> RECEIPT_CONFIRMED
 -> PREPARING_SHOT
 -> SHOT_EXECUTED
```

`FireShot` 前的 `SafetySnapshot` 同时检查：

- 位姿是否有效；
- 地图是否锁定；
- 位姿与目标观测是否新鲜；
- 目标是否有效；
- 篮球是否在位；
- 机构是否健康；
- 队友是否处于安全状态。

动作请求有版本、任务 ID、消息序列、发送者和过期控制。动作反馈不是简单的“延时后自动成功”；状态机根据反馈进入下一步、恢复或紧急停止。这样的结构有两个好处：一是上层策略可以只关心高层动作名，例如 `NavigateToPose`、`PreparePass`、`ExecutePass`、`PrepareShot`、`FireShot`；二是底层控制、通信和机构反馈可以独立替换。

## 11. 双车运球、传球和投篮演示是怎样组织的

### 11.1 底盘运动

篮球演示控制器向两台车的 `/robotN/cmd_vel_chassis` 发布 `geometry_msgs/msg/Twist`。其中 `linear.x`、`linear.y` 控制平面移动，`angular.z` 控制偏航。演示中的 `_drive_to_xy` 计算当前位置到目标点的向量，按距离自适应限制速度，并以约 `0.08 m` 的位置容差判断到位。

robot1 的运球目标被设置为起点前方约 `2.40 m`，所以画面中 robot1 不再只做很短的位移。robot2 在投篮准备阶段移动到 `(4.90, -0.55)` 附近的 staging point。

### 11.2 篮球传球曲线

为了让传球的起止速度平滑，控制器没有用线性位置插值，而使用 smoothstep：

```text
s(u) = u² (3 - 2u),  0 <= u <= 1
```

水平方向为起点到接球队前方的插值，竖直方向加上：

```text
z = 0.30 + 1.20 sin(π s)
```

这形成一条中间抬高、两端平滑的传球弧线。控制器持续从 `/gazebo/model_states` 读取两车位置，并通过 `/gazebo/set_entity_state` 写入篮球模型位置。服务调用有 future 保护：上一个请求尚未完成时不再排入新请求，避免 Gazebo 在负载高时把旧请求堆积并以错误顺序显示。

### 11.3 投篮抛体曲线

投篮开始点来自 robot2 前方的持球位置，目标点为篮筐位置。当前控制器设置：

```text
目标篮筐坐标  (6.44, -0.19, 2.43)
最高点高度    3.40 m
重力加速度    9.81 m/s²
```

给定起点高度 `sz` 和最高点 `apex_z`，上升时间为：

```text
t_up = sqrt(2 (apex_z - sz) / g)
v_z0 = g t_up
```

球的垂直位置为：

```text
z(t) = sz + v_z0 t - 1/2 g t²
```

水平方向在总飞行时间内从起点线性插值到篮筐。控制器记录起点、篮筐、最高点、飞行时间和重力，以 JSON 事件写入 `demo_events.jsonl`；结束时把成功状态、两车位移、最终篮球误差和得分写入 `success_summary.json`。

这条机制的价值在于可重复的视觉流程和完整事件记录：运球、接球、投篮准备、抛体开始、进球和完成都拥有明确事件，而不是把展示结果藏在一段视频里。

## 12. RViz：如何阅读四种画面

RViz 是 ROS 的三维可视化工具。它不生成传感器数据，只订阅 topic 并按 TF 把数据放到正确空间位置。

| 画面 | 主要 topic | 应该观察什么 |
|---|---|---|
| Gazebo 场景 | Gazebo 世界状态 | 两台车、篮球、篮筐和任务过程 |
| 三维点云 | `/cloud_registered`、`/mid360/quality_map`、`/path` | 注册扫描是否随运动逐步覆盖场地结构 |
| 二维地图 | `/mid360/occupancy_grid`、`/path` | 障碍、自由空间和路径投影是否随点云更新 |
| RGB-D | robot1 或 robot2 的 RGB 与 `depth/image_visualized` | 彩色内容与近亮远暗深度关系是否同步 |

RViz 中最常见的“什么都看不到”并不一定代表 topic 没有数据。先检查三件事：Fixed Frame 是否与消息 frame 一致；topic 是否选择正确 namespace；显示项是否启用且队列大小足够。点云密度还会受 `lidar_samples`、`lidar_downsample`、FAST-LIO2 注册扫描发布与 RViz Point Size 共同影响。

## 13. 运行、构建与检查的最小路径

在 Ubuntu 22.04 + ROS 2 Humble 环境中，基本构建步骤是：

```bash
source /opt/ros/humble/setup.bash
cd /path/to/Robocon-mid360-autonomy-stack
export LIVOX_SDK2_ROOT=/path/to/Livox-SDK2/install
colcon build --symlink-install --cmake-args -DLIVOX_SDK2_ROOT="$LIVOX_SDK2_ROOT"
source install/setup.bash
```

检查 package 清单：

```bash
python3 tools/validate_project.py
```

启动双车地图与 RGB-D：

```bash
ros2 launch robocon_mid360_simulation gazebo_mid360_dual_mapping.launch.py \
  use_gui:=true enable_rgbd:=true lidar_samples:=30000 lidar_downsample:=1
```

在另一个已经 `source install/setup.bash` 的终端检查数据流：

```bash
ros2 topic hz /robot1/livox/lidar
ros2 topic hz /cloud_registered
ros2 topic hz /mid360/occupancy_grid
ros2 topic list | grep -E 'robot[12]/simulated_rgbd|cloud_registered|occupancy_grid|path'
```

这些命令分别回答三个不同问题：原始雷达有没有来、FAST-LIO2 注册点云有没有来、二维地图有没有随点云更新。把它们混成一个“系统是否正常”的问题，排障会非常困难。

## 14. 常见现象与系统化排查顺序

### 14.1 RViz 点云稀疏

按从源到显示的顺序检查：

1. `/robot1/livox/lidar` 的频率与每包点数；
2. `/cloud_registered` 是否持续发布；
3. FAST-LIO2 是否启用了注册扫描发布的 mapping profile；
4. `scan_voxel`、`map_voxel` 和半径过滤是否过强；
5. RViz 的 Fixed Frame、PointCloud2 topic、Point Size 和队列设置；
6. 电脑负载、Gazebo real-time factor 与当前 `lidar_samples/downsample` 组合。

### 14.2 二维地图看起来不变化

二维栅格只会在接收点云、完成高度过滤、坐标投影和 log-odds 更新后变化。检查 `/mid360/occupancy_grid` 的频率，确认输入是 `/cloud_registered`，再确认 map 节点的分辨率、区域范围和 `publish_every_n_clouds`。如果机器人几乎不动，连续扫描也会投影到近似相同格子，画面变化自然有限。

### 14.3 深度图黑或没有画面

先确认 `enable_rgbd:=true`；再分别查看 raw depth 和 `depth/image_visualized`。raw 深度是数值图，不适合直接按 RGB 显示；使用可视化 topic 或 `run_rgbd_image_view.sh`。如果 robot1 有图但 robot2 没有，先检查命名空间是否选成 `/robot2/simulated_rgbd_camera/...`。

### 14.4 控制器动作看起来跳变

双车演示中，底盘与篮球是不同链路：底盘通过 `Twist` 运动，篮球由状态服务逐步更新。事件日志 `demo_events.jsonl` 和最终 `success_summary.json` 用来检查状态序列、球的起止位置、飞行时间与成功判定，而不是只靠肉眼判断。

## 15. 推荐的源码阅读顺序

阅读大型机器人仓库时，先读启动入口，再读通信合同，最后读算法实现，效率最高。建议顺序：

1. `README.md`：系统总览、构建与演示入口；
2. `src/robocon_mid360_simulation/launch/gazebo_mid360_dual_mapping.launch.py`：双车、FAST-LIO2、点云地图、栅格和深度可视化如何一起启动；
3. `src/mid360_localization_contract/INTERFACE_CONTRACT.md`：话题、坐标系和状态条件；
4. `src/mid360_localization_contract/.../input_guard.py`、`tracking.py`：输入健康与地图锁定逻辑；
5. `src/mid360_map_tools/src/`：体素地图和二维 log-odds 栅格；
6. `src/mid360_map_localizer/src/mid360_scan_matcher_node.cpp`：ICP 固定地图定位；
7. `src/robocon_perception_adapter/.../target_gate.py`：视觉结果如何变成控制条件；
8. `src/robocon_pose_command_bridge/.../core.py`：四元数、目标方向和命令门；
9. `src/robocon_game_supervisor/`：比赛状态机和动作协议；
10. `src/robocon_mid360_simulation/scripts/basketball_demo_controller.py`：双车运球、传球和投篮演示的完整事件流程。

## 16. 最后把知识点连起来

如果只记住一条主线，可以记成下面这句话：

> MID-360 给出带包内时间的三维回波，IMU 给出连续运动信息，FAST-LIO2 用两者估计局部运动；TF 和固定地图把局部运动放入场地坐标；点云模块把环境投影成 3D/2D 地图；RGB-D 与检测器给出目标观测；目标门、位姿门和比赛状态机决定何时可以发布动作；双车演示把这些接口放到一个可观察、可记录的篮球流程中。

这也是该仓库最值得学习的地方：它没有把雷达、视觉、地图和控制写成一个不可拆分的大程序，而是用 ROS 2 topic、TF、参数、状态机和诊断信息把每段责任拆开。理解这些边界后，读者可以替换其中任意一层，例如换检测器、换里程计前端、换地图表示或换底盘控制器，同时仍能沿着相同的接口检查系统。
