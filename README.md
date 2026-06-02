# GELLO → Franka Panda 遥操作（ROS1）

GELLO 主臂读数经标定后，通过 ROS1 控制 Franka Panda（机械臂 + 夹爪）。

```bash
pip install -r requirements.txt
source /opt/ros/<distro>/setup.bash && source <catkin_ws>/devel/setup.bash
```

## 需要先启动（各开一个终端）

```bash
roslaunch franka_control franka_control.launch robot_ip:=<IP>
roslaunch panda_moveit_config <moveit>.launch          # MoveIt + 关节轨迹控制器
roslaunch franka_gripper franka_gripper.launch robot_ip:=<IP>
```

## 指令

**首次标定**（GELLO 摆姿势 `0 0 0 -1.57 0 1.57 0` 不动 → 测臂 offset；再夹爪张开/闭合各按一次回车）：

```bash
python scripts/calibrate.py --port /dev/ttyUSB0 \
  --start-joints 0 0 0 -1.57 0 1.57 0 \
  --joint-signs 1 1 1 1 1 -1 1 \
  --gripper --out configs/panda_calib.json
```

**只重标夹爪**：

```bash
python scripts/calibrate.py --port /dev/ttyUSB0 \
  --gripper-only --calib configs/panda_calib.json
```

**遥操作**（推荐 `trajectory`）：

```bash
python examples/teleop_panda_ros1_moveit.py --calib configs/panda_calib.json
```

## 可调参数（`teleop_panda_ros1_moveit.py`）

| 参数 | 默认 | 说明 |
|------|------|------|
| `--calib` | （必填） | 标定 JSON |
| `--hz` | `50` | 机械臂控制频率 |
| `--gripper-hz` | `1` | 夹爪下发频率 |
| `--mode` | `trajectory` | `trajectory` 连续跟随；`moveit` 每步规划（建议 `--hz 8`） |
| `--alpha` | `1.0` | 主臂平滑，`1` 关闭 |
| `--max-delta` | `0.05` | 每步单关节最大变化 (rad) |
| `--max-start-delta` | `0.8` | 启动时主从差异上限 (rad)，超限拒绝启动 |
| `--arm-id` | `panda` | 关节名前缀 |
| `--move-group` | `panda_arm` | MoveIt 规划组（`moveit` 模式） |
| `--traj-action` | 见下 | 轨迹 action；默认 `/position_joint_trajectory_controller/follow_joint_trajectory` |
| `--joint-states-topic` | `/franka_state_controller/joint_states` | 读当前关节角 |
| `--gripper-mode` | `move` | `move` 连续宽度 / `grasp` 开合力抓取 / `none` 不控夹爪 |
| `--gripper-max-width` | `0.08` | 夹爪最大开口 (m) |
| `--gripper-speed` | `0.1` | 夹爪速度 |
| `--gripper-force` | `20` | `grasp` 模式夹持力 (N) |

示例：

```bash
python examples/teleop_panda_ros1_moveit.py --calib configs/panda_calib.json \
  --mode trajectory --hz 50 --gripper-hz 1 --max-delta 0.05
```
