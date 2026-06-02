"""读取 GELLO 主臂 -> 通过 ROS1 + MoveIt 控制 Franka Panda。

数据流：
    GELLO (Dynamixel) --LeaderArm--> 关节角(rad) + 夹爪[0,1]
        |
        v  (本脚本)
    ROS1: MoveIt MoveGroupCommander  -> 机械臂 7 关节
          franka_gripper action      -> 夹爪开合

机械臂支持两种下发方式（--mode）：

  * moveit      : 用 moveit_commander 的 MoveGroupCommander 规划并执行到目标关节角。
                  碰撞检测/限位最完整，但每个周期都要规划，频率低、动作偏顿，
                  适合慢速、安全优先的跟随。
  * trajectory  : 直接给 MoveIt 所管理的关节轨迹控制器
                  (position/effort_joint_trajectory_controller) 发送单点
                  FollowJointTrajectory 目标。无规划、平滑、适合连续遥操作。
                  这正是 MoveIt 执行轨迹时所用的同一个控制器接口。

夹爪通过 franka_gripper 的 action 控制（--gripper-mode）：
  * move        : franka_gripper/move，按目标宽度连续跟随（推荐遥操作）。
  * grasp       : franka_gripper/grasp，带抓取力（开/合切换，不适合连续跟随）。
  * none        : 不控制夹爪。

依赖（来自 ROS 环境，不在 requirements.txt 里）：
    rospy, actionlib, moveit_commander(=moveit mode),
    control_msgs, trajectory_msgs, sensor_msgs, franka_gripper

先在另一个终端启动 Franka 与 MoveIt，例如：
    roslaunch franka_control franka_control.launch robot_ip:=<IP>
    roslaunch panda_moveit_config <moveit>.launch        # 或对应的 moveit launch
    roslaunch franka_gripper franka_gripper.launch robot_ip:=<IP>

然后运行：
    python examples/teleop_panda_ros1_moveit.py --calib configs/panda_calib.json \
        --mode trajectory --hz 50 --gripper-hz 1
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gello_leader import LeaderArm, LeaderCalibration  # noqa: E402

# Franka Hand 最大开口宽度（米）。GELLO 约定 width = MAX_OPEN * (1 - gripper)。
PANDA_MAX_OPEN_M = 0.08


def _import_ros():
    """延迟导入 ROS 依赖，给出友好的错误提示。"""
    try:
        import rospy  # noqa: F401
        import actionlib  # noqa: F401
        from control_msgs.msg import (  # noqa: F401
            FollowJointTrajectoryAction,
            FollowJointTrajectoryGoal,
            GripperCommandAction,
            GripperCommandGoal,
        )
        from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: F401
        from sensor_msgs.msg import JointState  # noqa: F401
    except Exception as e:  # pragma: no cover
        raise SystemExit(
            "无法导入 ROS1 依赖，请先 `source /opt/ros/<distro>/setup.bash` 以及你的 "
            f"catkin 工作空间 setup.bash 后再运行。原始错误：{e}"
        )


class FrankaMoveItInterface:
    """通过 ROS1 把目标关节角/夹爪下发给 Franka。

    与 examples/teleop_panda.py 中的 PandaInterface 接口保持一致：
        get_joint_positions() -> np.ndarray(7,)
        command_joint_positions(q_arm)
        command_gripper(gripper_01)
    """

    def __init__(
        self,
        mode: str = "trajectory",
        arm_id: str = "panda",
        move_group: str = "panda_arm",
        traj_action: str = "",
        joint_states_topic: str = "/franka_state_controller/joint_states",
        gripper_mode: str = "move",
        gripper_max_width: float = PANDA_MAX_OPEN_M,
        gripper_speed: float = 0.1,
        gripper_force: float = 20.0,
        plan_time: float = 0.2,
    ) -> None:
        import rospy
        import actionlib
        from sensor_msgs.msg import JointState

        self._rospy = rospy
        self._mode = mode
        self._gripper_mode = gripper_mode
        self._gripper_max_width = float(gripper_max_width)
        self._gripper_speed = float(gripper_speed)
        self._gripper_force = float(gripper_force)

        # 7 个机械臂关节名，顺序与 Franka getJointPositions 一致。
        self._joint_names = [f"{arm_id}_joint{i}" for i in range(1, 8)]

        rospy.init_node("gello_teleop_ros1", anonymous=True, disable_signals=True)

        # --- 机械臂控制 ---
        if mode == "moveit":
            import moveit_commander

            moveit_commander.roscpp_initialize(sys.argv)
            self._mc = moveit_commander
            self._group = moveit_commander.MoveGroupCommander(move_group)
            self._group.set_max_velocity_scaling_factor(0.3)
            self._group.set_max_acceleration_scaling_factor(0.3)
            self._group.set_planning_time(float(plan_time))
            self._joint_names = list(self._group.get_active_joints())
            rospy.loginfo("MoveIt group '%s' joints: %s", move_group, self._joint_names)
        elif mode == "trajectory":
            from control_msgs.msg import FollowJointTrajectoryAction

            if not traj_action:
                traj_action = "/position_joint_trajectory_controller/follow_joint_trajectory"
            self._traj_client = actionlib.SimpleActionClient(
                traj_action, FollowJointTrajectoryAction
            )
            rospy.loginfo("等待轨迹控制器 action：%s", traj_action)
            if not self._traj_client.wait_for_server(rospy.Duration(10.0)):
                raise SystemExit(f"等待 {traj_action} 超时，确认控制器已启动。")
            # trajectory 模式下从 joint_states 读当前关节角。
            self._last_js = None
            rospy.Subscriber(joint_states_topic, JointState, self._on_joint_state)
            rospy.loginfo("订阅关节状态：%s", joint_states_topic)
            self._wait_for_joint_state()
        else:
            raise SystemExit(f"未知 mode: {mode!r}（可选 moveit / trajectory）")

        # --- 夹爪控制 ---
        self._grip_client = None
        if gripper_mode == "move":
            try:
                from franka_gripper.msg import MoveAction
            except Exception as e:
                raise SystemExit(f"导入 franka_gripper 失败：{e}")
            self._grip_client = actionlib.SimpleActionClient(
                "/franka_gripper/move", MoveAction
            )
            self._grip_action = "move"
        elif gripper_mode == "grasp":
            try:
                from franka_gripper.msg import GraspAction
            except Exception as e:
                raise SystemExit(f"导入 franka_gripper 失败：{e}")
            self._grip_client = actionlib.SimpleActionClient(
                "/franka_gripper/grasp", GraspAction
            )
            self._grip_action = "grasp"
        elif gripper_mode == "none":
            self._grip_action = "none"
        else:
            raise SystemExit(f"未知 gripper-mode: {gripper_mode!r}")

        if self._grip_client is not None:
            rospy.loginfo("等待夹爪 action：/franka_gripper/%s", self._grip_action)
            if not self._grip_client.wait_for_server(rospy.Duration(10.0)):
                rospy.logwarn("夹爪 action 未就绪，将忽略夹爪指令。")
                self._grip_client = None

        self._grip_busy = False  # grasp 模式下避免重复发同一目标
        self._last_grip_cmd = None

    # ------------------------------------------------------------------ #
    # joint_states (trajectory mode)                                     #
    # ------------------------------------------------------------------ #
    def _on_joint_state(self, msg) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self._last_js = np.array(
                [name_to_pos[n] for n in self._joint_names], dtype=float
            )
        except KeyError:
            pass  # 该消息可能只含夹爪关节，忽略

    def _wait_for_joint_state(self, timeout: float = 10.0) -> None:
        rospy = self._rospy
        t0 = rospy.Time.now()
        rate = rospy.Rate(50)
        while self._last_js is None and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > timeout:
                raise SystemExit("等待 joint_states 超时，确认 franka_state_controller 已运行。")
            rate.sleep()

    # ------------------------------------------------------------------ #
    # Public API (与 PandaInterface 一致)                                 #
    # ------------------------------------------------------------------ #
    def get_joint_positions(self) -> np.ndarray:
        if self._mode == "moveit":
            return np.array(self._group.get_current_joint_values(), dtype=float)
        if self._last_js is None:
            self._wait_for_joint_state()
        return np.array(self._last_js, dtype=float)

    def command_joint_positions(self, q_arm: np.ndarray) -> None:
        q_arm = np.asarray(q_arm, dtype=float).tolist()
        if self._mode == "moveit":
            self._group.set_joint_value_target(q_arm)
            # 阻塞执行到目标；遥操作时建议降低 --hz。
            self._group.go(wait=True)
            self._group.stop()
        else:
            self._send_trajectory_point(q_arm)

    def _send_trajectory_point(self, q_arm, time_from_start: float = 0.1) -> None:
        from control_msgs.msg import FollowJointTrajectoryGoal
        from trajectory_msgs.msg import JointTrajectoryPoint

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self._joint_names
        point = JointTrajectoryPoint()
        point.positions = list(q_arm)
        point.velocities = [0.0] * len(q_arm)
        point.time_from_start = self._rospy.Duration.from_sec(time_from_start)
        goal.trajectory.points.append(point)
        goal.trajectory.header.stamp = self._rospy.Time(0)  # 立即执行
        # 不等待结果，连续覆盖发送以实现平滑跟随。
        self._traj_client.send_goal(goal)

    def command_gripper(self, gripper_01: float) -> None:
        if self._grip_client is None or self._grip_action == "none":
            return
        g = float(np.clip(gripper_01, 0.0, 1.0))
        width = self._gripper_max_width * (1.0 - g)

        if self._grip_action == "move":
            from franka_gripper.msg import MoveGoal

            # 宽度变化很小则跳过，减少抖动与指令风暴。
            if self._last_grip_cmd is not None and abs(width - self._last_grip_cmd) < 0.002:
                return
            self._last_grip_cmd = width
            goal = MoveGoal(width=width, speed=self._gripper_speed)
            self._grip_client.send_goal(goal)
        elif self._grip_action == "grasp":
            from franka_gripper.msg import GraspGoal

            close = g > 0.5
            if self._last_grip_cmd == close:
                return
            self._last_grip_cmd = close
            goal = GraspGoal()
            goal.width = 0.0 if close else self._gripper_max_width
            goal.speed = self._gripper_speed
            goal.force = self._gripper_force
            goal.epsilon.inner = 0.08
            goal.epsilon.outer = 0.08
            self._grip_client.send_goal(goal)

    def shutdown(self) -> None:
        if self._mode == "moveit":
            try:
                self._group.stop()
                self._mc.roscpp_shutdown()
            except Exception:
                pass


def main() -> None:
    p = argparse.ArgumentParser(description="GELLO leader -> ROS1/MoveIt -> Franka Panda")
    p.add_argument("--calib", required=True, help="标定 JSON 路径")
    p.add_argument("--hz", type=float, default=50.0, help="机械臂控制频率（moveit 模式建议 5~10）")
    p.add_argument(
        "--gripper-hz",
        type=float,
        default=1.0,
        help="夹爪下发频率（与机械臂解耦，默认 1 Hz）",
    )
    p.add_argument("--alpha", type=float, default=1.0, help="主臂平滑系数（1=关闭）")
    p.add_argument(
        "--mode",
        choices=["trajectory", "moveit"],
        default="trajectory",
        help="机械臂下发方式：trajectory(平滑遥操作) 或 moveit(规划执行)",
    )
    p.add_argument("--arm-id", default="panda", help="关节命名前缀（panda / fr3 等）")
    p.add_argument("--move-group", default="panda_arm", help="MoveIt 规划组名")
    p.add_argument(
        "--traj-action",
        default="",
        help="trajectory 模式的 FollowJointTrajectory action 名"
        "（默认 /position_joint_trajectory_controller/follow_joint_trajectory）",
    )
    p.add_argument(
        "--joint-states-topic",
        default="/franka_state_controller/joint_states",
        help="trajectory 模式读取当前关节角的话题",
    )
    p.add_argument(
        "--gripper-mode",
        choices=["move", "grasp", "none"],
        default="move",
        help="夹爪控制方式",
    )
    p.add_argument("--gripper-max-width", type=float, default=PANDA_MAX_OPEN_M)
    p.add_argument("--gripper-speed", type=float, default=0.1)
    p.add_argument("--gripper-force", type=float, default=20.0)
    p.add_argument(
        "--max-delta",
        type=float,
        default=0.05,
        help="每步每个关节最大变化(rad)，限制跳变（与 GELLO run_env 类似）",
    )
    p.add_argument(
        "--max-start-delta",
        type=float,
        default=0.8,
        help="主/从起始差异超过该值(rad)则拒绝启动",
    )
    args = p.parse_args()

    _import_ros()

    calib = LeaderCalibration.load(args.calib)
    arm_period = 1.0 / args.hz
    gripper_period = 1.0 / args.gripper_hz

    robot = FrankaMoveItInterface(
        mode=args.mode,
        arm_id=args.arm_id,
        move_group=args.move_group,
        traj_action=args.traj_action,
        joint_states_topic=args.joint_states_topic,
        gripper_mode=args.gripper_mode,
        gripper_max_width=args.gripper_max_width,
        gripper_speed=args.gripper_speed,
        gripper_force=args.gripper_force,
    )

    with LeaderArm(calib, alpha=args.alpha) as leader:
        # --- 安全检查：启动前比对主/从起始差异 ---
        arm, gripper = leader.get_arm_and_gripper()
        current = robot.get_joint_positions()
        if arm.shape != current.shape:
            raise SystemExit(
                f"自由度不匹配：主臂 {arm.shape} vs 机器人 {current.shape}"
            )
        gap = np.abs(arm - current)
        if gap.max() > args.max_start_delta:
            worst = int(np.argmax(gap))
            raise SystemExit(
                f"主/从在关节 {worst} 差异过大 (delta {gap[worst]:.3f} > "
                f"{args.max_start_delta})。请先把 GELLO 摆到与机器人接近的姿势。"
            )

        print(
            f"遥操作启动（mode={args.mode}, 臂 {args.hz}Hz, 夹爪 {args.gripper_hz}Hz）。"
            "Ctrl+C 停止。"
        )
        last_gripper_send = 0.0
        try:
            while True:
                arm, gripper = leader.get_arm_and_gripper()
                current = robot.get_joint_positions()

                # 限制单步跳变。
                delta = arm - current
                biggest = np.abs(delta).max()
                if biggest > args.max_delta:
                    delta = delta / biggest * args.max_delta
                robot.command_joint_positions(current + delta)

                if gripper is not None:
                    now = time.monotonic()
                    if now - last_gripper_send >= gripper_period:
                        robot.command_gripper(gripper)
                        last_gripper_send = now

                time.sleep(arm_period)
        except KeyboardInterrupt:
            print("\nstopped.")
        finally:
            robot.shutdown()


if __name__ == "__main__":
    main()
