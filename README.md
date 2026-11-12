# hero_chassis_controller

## Overview

`hero_chassis_controller` 是一个 [ros_control] 架构下的的麦轮（mecanum）英雄底盘控制器插件，实现为
`controller_interface::Controller<hardware_interface::EffortJointInterface>`。它订阅 `/cmd_vel`，
经麦轮运动学解算得到四轮目标转速，再由各自的 PID 闭环算出力矩指令下发给 `EffortJointInterface`；
同时用轮速反馈做正运动学解算，积分出里程计并发布 `odom → base_link` 的 TF。

主要功能：

- 从配置文件读取底盘参数、四个轮的几何位姿以及每个轮子的 PID 参数；
- 麦轮逆运动学：用雅可比矩阵把底盘速度 `(vx, vy, wz)` 映射为四轮目标转速；
- 轮速闭环：[control_toolbox/Pid] 由转速误差计算力矩指令；
- 速度斜坡与限幅：底盘速度、轮速均有限幅，可选按固定加速度提速；
- `/cmd_vel` 超时保护：超过 `timeout` 未收到指令时清零输出；
- `frame_mode: world` 时，把世界系（odom）下的速度指令旋转到底盘系再解算；
- 正运动学（最小二乘）＋中点积分里程计，发布 `nav_msgs/Odometry` 与 TF。

**Keywords:** ros_control, mecanum, hero chassis, chassis controller, odometry, PID

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) Noetic（middleware for robotics）
- [Eigen]（linear algebra library，ROS 自带 `libeigen3-dev`）
- `controller_interface`、`hardware_interface`、`pluginlib`、`control_toolbox`、`realtime_tools`
- `roscpp`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、`angles`、`geometry_msgs`、`nav_msgs`
- 运行仿真还需要 `rm_description_for_task`、`rm_gazebo`（提供 hero 模型与 `librm_robot_hw_sim.so`），
  以及 `controller_manager`、`joint_state_controller`、`robot_state_publisher`

用 `rosdep` 安装依赖：

```bash
rosdep install --from-paths src --ignore-src
```

#### Building

把仓库克隆到 catkin 工作空间并编译（本工程使用 `catkin_tools`）：

```bash
cd /root/catkin_ws/src
git clone git@github.com:Seedofking/hero_chassis_controller.git
cd /root/catkin_ws

source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src
catkin build hero_chassis_controller
source devel/setup.bash
```

本机在 Docker 容器 `ros_noetic` 中开发，宿主目录 `/home/seed/catkin_ws` 挂载为容器内 `/root/catkin_ws`，

```bash
source /root/catkin_ws/env.sh          # 容器内
```

### Running in Docker

本工程的编译与运行都在容器内完成

```bash
docker exec -it ros_noetic bash
source /root/catkin_ws/env.sh
```

容器以 host 网络运行、直通 `/dev/dri`，Gazebo/RViz 可直接显示在宿主的桌面上。

### Unit Tests

本包当前没有单元测试（`CMakeLists.txt` 中的 gtest 段落保持注释状态）。

## Usage

### 起 Gazebo 仿真并加载控制器

```bash
roslaunch hero_chassis_controller run_simulation_and_load_controller.launch
```

### 仅加载控制器

```bash
roslaunch hero_chassis_controller load_controller.launch
```

### 发送速度指令

```bash
rostopic pub -r 50 /cmd_vel geometry_msgs/Twist "{ linear: {x: 1, y: 1}, angular: {z: 1}}"
```

停止发指令后，超过 `timeout` 秒控制器会自动把速度指令清零。

### 查看里程计

```bash
rostopic echo /odom
rosrun rviz rviz    # Add → Odometry，或直接把 Fixed Frame 设为 odom
rosrun plotjuggler plotjuggler
```

## Config files

* **config/controllers.yaml** 控制器参数：底盘模式与限幅、四个轮的关节名/几何位姿/PID，
  以及 `joint_state_controller` 的发布频率。整份 yaml 以 `controller:` 为顶层命名空间，
  由 launch 用 `<rosparam file="..."/>` 加载，因此参数实际位于 `/controller/<控制器名>/...` 下
  （控制器名为 `controller/hero_chassis_controller`，即命名空间 `/controller/hero_chassis_controller`）。

## Launch files

* **run_simulation_and_load_controller.launch** 启动完整的仿真环境：

    * 通过 `<include>` 调用 `rm_description_for_task/launch/hero.launch`，参数在 include 中写死：

        - **`load_chassis`** 是否加载底盘模型。取值：`true`。
        - **`paused`** Gazebo 是否以暂停状态启动。取值：`false`。
        - **`roller_type`** 麦轮辊子碰撞体类型，`simple`（球体）或 `realistic`（网格）。取值：`simple`。

    * 加载 `controllers.yaml`，用 `controller_manager/spawner` 启动
      `controller/hero_chassis_controller` 与 `controller/joint_state_controller`；
    * 启动 `robot_state_publisher`（由 `/joint_states` 发布底盘到四个轮的 TF）。

* **load_controller.launch** 只加载参数、spawn 两个控制器并启动 `robot_state_publisher`，
  不包含仿真。适用于外部已提供硬件接口/controller_manager 的场合。

## Controller

### hero_chassis_controller

控制器插件 `hero_chassis_controller::HeroChassisController`，由 `controller_manager` 加载，不是独立节点。
硬件层提供四个轮的 `EffortJointInterface`（hero 模型通过 `transmission` 声明），
并在 `update()` 中按以下顺序执行：

1. 从实时缓冲区读取最新的 `/cmd_vel`；
2. 超时保护：`timeout` 内未更新则指令清零；
3. `frame_mode: world` 时把世界系速度旋转到底盘系；
4. 速度斜坡与底盘速度限幅；
5. 麦轮逆运动学得到四轮目标转速，再做轮速限幅；
6. 每轮 PID 由转速误差计算力矩指令；
7. 正运动学得到底盘实际速度，做中点积分更新里程计；
8. 按 `publish_rate` 发布 `/odom` 与 `odom → base_link` TF。

#### Subscribed Topics

* **`/cmd_vel`** ([geometry_msgs/Twist])

  底盘目标速度，`linear.x` / `linear.y` / `angular.z` 参与解算（其余字段忽略）。
  订阅建立在根节点句柄上，是全局话题

#### Published Topics

* **`/odom`** ([nav_msgs/Odometry])

  由轮速反馈正运动学解算并积分得到的里程计，`frame_id` 为 `odom`，`child_frame_id` 为 `base_link`，
  发布频率由 `publish_rate` 控制，使用 `realtime_tools::RealtimePublisher` 在实时线程中发布。

#### Transformations

* **`odom` → `base_link`**：与 `/odom` 同频广播，位姿来自积分里程计。
  `base_link` → 各轮的 TF 由 `robot_state_publisher` 根据 `/joint_states` 发布。

#### Services

无。

#### Parameters

参数位于 `/controller/hero_chassis_controller/` 下（数值为 `config/controllers.yaml` 中的当前取值）：

* **`publish_rate`** (double, default: 100)

  `/odom` 与 TF 的发布频率，单位 Hz。

* **`timeout`** (double, default: 0.5)

  `/cmd_vel` 超时时间，单位 s；超时后速度指令清零。

* **`frame_mode`** (string, default: "world")

  速度指令所在坐标系。`base`：指令即底盘系速度；`world`：指令是世界系（odom）速度，
  控制器用 TF 把它旋转到底盘系后再解算。TF 查询失败时会清零指令。

* **`use_velocity_ramp`** (bool, default: false)

  是否对速度指令做斜坡处理。

* **`chassis_velocity_ramp_rate`** (double, default: 2)

  斜坡加速度，单位 (m/s)/s 与 rad/s²；仅在 `use_velocity_ramp: true` 时生效。

* **`wheel_max_velocity`** (double, default: 50)

  轮子最大转速，单位 rad/s；逆运动学结果按最大分量等比缩放限幅。

* **`chassis_max_velocity`** (double, default: 3.9)

  底盘最大速度，单位 m/s（作用于 `(vx, vy, wz)` 的最大分量，等比缩放）。

* **`wheel_radius`** (double, default: 0.07625)

  麦轮半径，单位 m，用于速度到转速的换算。

* **`joints`** (string[], 长度必须为 4)

  四个轮的关节名，顺序为 `left_front`、`right_front`、`left_back`、`right_back`，
  必须与 URDF 中的关节名一致。

* **`wheel/<joint_name>/pid`** (namespace)

  每个轮的 PID 参数，交给 [control_toolbox/Pid] 的 `init()` 读取：
  `p`、`i`、`d`、`i_clamp_max`、`i_clamp_min`、`antiwindup`、`publish_state`。

* **`wheel/<joint_name>/pose`** (namespace)

  该轮相对 `base_link` 的安装位置与辊子角度：
  `x` (m)、`y` (m)、`roller_angle` (rad)。
  控制器据此构造逆运动学雅可比矩阵，取值范围与正负号需与 URDF 保持一致。

* **`wheel_base` / `wheel_track`** (double, default: 0.4)

  仅在 yaml 中作为几何说明保留；控制器实际使用的是 `wheel/<joint_name>/pose` 里的 `x` / `y`。

## Bugs & Feature Requests

Please report bugs and request features using
the [Issue Tracker](https://github.com/Seedofking/hero_chassis_controller/issues).


[ROS]: http://www.ros.org

[rviz]: http://wiki.ros.org/rviz

[Eigen]: http://eigen.tuxfamily.org

[ros_control]: http://wiki.ros.org/ros_control

[control_toolbox/Pid]: http://wiki.ros.org/control_toolbox

[geometry_msgs/Twist]: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html

[nav_msgs/Odometry]: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
