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
│   └── teleop_panda.py       # read → 处理 → 发送 Panda（接口为待填桩）
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
