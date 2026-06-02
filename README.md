# panda_leader — GELLO 主臂（leader）读取与处理

从 [`gello_software`](../gello_software-main) 中**抽取并精简**出来的最小子集，只保留：

1. 通过 Dynamixel（U2D2/USB）**读取** GELLO 主臂的关节角；
2. 用标定参数把原始舵机角度**映射到 Franka Panda 关节空间**（含夹爪归一化到 `[0,1]`）；
3. 把处理后的关节向量**交给你自己的 Panda 接口**。

不包含：完整 `gello` 包、ROS、follower 机器人驱动、重力补偿、ZMQ、相机、数据采集。
跨平台（Windows `COMx` / Linux `/dev/...` / macOS）。

## 安装

```bash
pip install -r requirements.txt
```

依赖仅 `numpy` 和 `dynamixel-sdk`。

## 目录结构

```
panda_leader/
├── gello_leader/
│   ├── dynamixel_driver.py   # 只读 Dynamixel 驱动（后台线程轮询，脉冲→弧度）
│   ├── calibration.py        # 标定参数 dataclass + JSON 读写
│   └── leader_arm.py         # 标定公式：raw → Panda 关节 + 夹爪[0,1]
├── scripts/
│   ├── calibrate.py          # 标定：检测 joint_offsets / 夹爪开合角
│   └── read_leader.py        # 验证：循环打印处理后的关节角
├── examples/
│   ├── teleop_panda.py            # read → 处理 → 发送 Panda（接口为待填桩）
│   └── teleop_panda_ros1_moveit.py # read → ROS1/MoveIt → 控制 Franka（开箱即用）
└── configs/
    └── panda_calib.example.json
```

## 使用流程

### 1. 设置电机 ID

用 Dynamixel Wizard 把每个舵机 ID 设为 `1..7`（基座→腕部），夹爪通常为 `8`。

### 2. 标定

把 GELLO 摆到 Panda 已知姿势（弧度）`0 0 0 -1.57 0 1.57 0` 并保持不动：

```bash
# Windows（端口换成你的 COM 号）
python scripts/calibrate.py --port COM5 ^
  --start-joints 0 0 0 -1.57 0 1.57 0 ^
  --joint-signs 1 1 1 1 1 -1 1 ^
  --gripper --out configs/panda_calib.json
```

```bash
# Linux
python scripts/calibrate.py --port /dev/ttyUSB0 \
  --start-joints 0 0 0 -1.57 0 1.57 0 \
  --joint-signs 1 1 1 1 1 -1 1 \
  --gripper --out configs/panda_calib.json
```

结果会写入 `configs/panda_calib.json`。无夹爪用 `--no-gripper`。

**夹爪行程标定**（启用 `--gripper` 时自动进行）：

1. 臂关节 offset 检测完成后，提示将夹爪**完全张开**，按回车记录 `open_deg`（映射为 `gripper_01 = 0`）；
2. 再提示**完全闭合**，按回车记录 `close_deg`（映射为 `gripper_01 = 1`）。

仅重标夹爪、保留已有臂标定：

```bash
python scripts/calibrate.py --port /dev/ttyUSB0 \
  --gripper-only --calib configs/panda_calib.json
```

JSON 中 `gripper` 字段为 `[舵机ID, 张开角度deg, 闭合角度deg]`，运行时线性映射到 `[0, 1]`。

### 3. 验证读取

```bash
python scripts/read_leader.py --calib configs/panda_calib.json
```

在标定姿势下，臂关节应接近 `0 0 0 -1.57 0 1.57 0`，夹爪在 `[0,1]` 内变化。
若某一轴方向相反，把 `configs/panda_calib.json` 里该轴的 `joint_signs` 取反即可。

### 4. 接入你的 Panda

编辑 `examples/teleop_panda.py` 中的 `PandaInterface`，把三个方法换成你自己的
机器人 API（libfranka / franka_ros2 / Polymetis / 自建桥接）：

```python
robot.command_joint_positions(q_arm)  # 7 关节弧度，与 Franka getJointPositions 同序
robot.command_gripper(gripper_01)     # 0=张开, 1=闭合；width = 0.09 * (1 - g)
```

然后运行：

```bash
python examples/teleop_panda.py --calib configs/panda_calib.json
```

### 5. 直接用 ROS1 + MoveIt 控制 Franka（开箱即用）

`examples/teleop_panda_ros1_moveit.py` 已经把 Panda 接口实现为 ROS1 版本：机械臂通过
MoveIt 所管理的控制器下发，夹爪通过 `franka_gripper` 的 action 控制。

**额外依赖**（来自 ROS 环境，不在 `requirements.txt`）：
`rospy`、`actionlib`、`moveit_commander`（仅 `--mode moveit` 需要）、
`control_msgs`、`trajectory_msgs`、`sensor_msgs`、`franka_gripper`。
运行前先 `source /opt/ros/<distro>/setup.bash` 和你的 catkin 工作空间 `setup.bash`。

先在其他终端启动机器人与 MoveIt（示例，按你的实际 launch 调整）：

```bash
roslaunch franka_control franka_control.launch robot_ip:=<机器人IP>
roslaunch panda_moveit_config <你的moveit>.launch        # 启动 MoveIt + 关节轨迹控制器
roslaunch franka_gripper franka_gripper.launch robot_ip:=<机器人IP>
```

然后运行遥操作脚本：

```bash
# 推荐：trajectory 模式，平滑连续跟随（直接给 MoveIt 管理的关节轨迹控制器发单点目标）
python examples/teleop_panda_ros1_moveit.py --calib configs/panda_calib.json \
  --mode trajectory --hz 50 --gripper-hz 1

# 或：moveit 模式，每周期用 moveit_commander 规划并执行（更安全但偏顿，建议降频）
python examples/teleop_panda_ros1_moveit.py --calib configs/panda_calib.json \
  --mode moveit --hz 8
```

常用参数：

- `--mode {trajectory,moveit}`：机械臂下发方式（见上）。
- `--arm-id panda`：关节命名前缀（`panda` / `fr3` …）。
- `--move-group panda_arm`：MoveIt 规划组（`moveit` 模式）。
- `--traj-action`：`trajectory` 模式的 action 名，默认
  `/position_joint_trajectory_controller/follow_joint_trajectory`（用 effort 控制器时改成
  `/effort_joint_trajectory_controller/...`）。
- `--gripper-mode {move,grasp,none}`：夹爪连续跟随用 `move`，带力抓取用 `grasp`。
- `--hz` / `--gripper-hz`：机械臂与夹爪控制频率（默认 50 Hz / 1 Hz，相互独立）。
- `--max-delta`：每步单关节最大变化(rad)，限制跳变。
- `--max-start-delta`：主/从起始差异过大则拒绝启动（安全）。

## 在你自己的代码里直接调用

```python
from gello_leader import LeaderArm, LeaderCalibration

calib = LeaderCalibration.load("configs/panda_calib.json")
with LeaderArm(calib, alpha=1.0) as leader:
    while True:
        arm_rad, gripper_01 = leader.get_arm_and_gripper()
        # ... 你的滤波/限速/IK/下发 ...
```

## 标定公式（与 GELLO 完全一致）

```
raw_rad[i] = present_position_pulse[i] / 2048 * pi
q[i]       = joint_signs[i] * (raw_rad[i] - joint_offsets[i])
gripper    = clip((q_grip - open_rad) / (close_rad - open_rad), 0, 1)
```
