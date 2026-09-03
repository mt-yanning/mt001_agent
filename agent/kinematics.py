"""运动学工具：直接复用 Arachne 的 AuboI5Kinematics，不自己写 IK。

★ 架构文档 1.3 的结论：IK 是白送的，别抄 autopick 那份手写实现。
  arachne_operator.real_hardware_acceptance_test.AuboI5Kinematics 提供
    fk(q)                    正运动学
    solve_position(...)      3 自由度位置 IK
    solve_pose(...)          6 自由度全位姿 IK（阻尼最小二乘）
  纯 numpy、无 ROS 依赖，agent_bridge 自己就是这么用的。

本模块只补两件 AuboI5Kinematics 没管的事：
  1. 坐标系换算。它算的是 aubo_base_link → tool0，而我们关心的是
     机器人 base_link 下的位置。机械臂安装偏移取自 autopick：
     ARM_MOUNT_XYZ=(0.22, 0, 0.105)、ARM_MOUNT_RPY=(0, 0, π/2)。
  2. 关节名换算。bridge 的 status 用规范名（shoulder_joint），
     而 mock / URDF 用带前缀的名字（aubo_shoulder_joint）。

v0.1 还不动机械臂，这里先把地基打好，v0.5 的 reach_to 直接用。
"""

from __future__ import annotations

import math
from typing import Any, Sequence

# 关节顺序（bridge 的规范名，索引 0-5）
CANONICAL_JOINTS = (
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)
PREFIXED_JOINTS = tuple(f"aubo_{name}" for name in CANONICAL_JOINTS)

# 机械臂在底盘上的安装位姿（base_link → aubo_base_link）
ARM_MOUNT_XYZ = (0.22, 0.0, 0.105)
ARM_MOUNT_RPY = (0.0, 0.0, math.pi / 2.0)

# mock 与 autopick 共用的 HOME 姿态
HOME_POSE = (
    -1.5707963267949,
    0.201570428261868,
    1.65970467002488,
    0.485178041391533,
    1.67675136677345,
    0.76432946885334,
)


class KinematicsUnavailable(RuntimeError):
    """没装 ROS2 / 找不到 arachne_operator 时抛出。"""


def _import_backend() -> Any:
    try:
        from arachne_operator.real_hardware_acceptance_test import AuboI5Kinematics
    except ImportError as exc:
        raise KinematicsUnavailable(
            "import 不到 AuboI5Kinematics。请先 source Arachne 的 install/setup.bash："
            "  source /root/autodl-tmp/Arachne/install/setup.bash"
            f"（原始错误：{exc}）"
        )
    return AuboI5Kinematics()


def joint_dict_to_vector(joints: dict[str, float]) -> list[float]:
    """把 bridge status 里的 arm_joints 字典按标准顺序摊成 6 元列表。"""
    missing = [name for name in CANONICAL_JOINTS if name not in joints]
    if missing:
        raise ValueError(f"关节角不完整，缺少：{missing}")
    return [float(joints[name]) for name in CANONICAL_JOINTS]


def vector_to_joint_dict(vector: Sequence[float], *, prefixed: bool = False) -> dict[str, float]:
    """反向转换。prefixed=True 时用 aubo_ 前缀名（发 JointTrajectory 时需要）。"""
    names = PREFIXED_JOINTS if prefixed else CANONICAL_JOINTS
    if len(vector) != len(names):
        raise ValueError(f"需要 {len(names)} 个关节角，收到 {len(vector)} 个")
    return {name: float(value) for name, value in zip(names, vector)}


class ArmKinematics:
    """AuboI5Kinematics 的薄包装，补上 base_link ↔ aubo_base_link 换算。"""

    def __init__(self) -> None:
        import numpy as np   # 延迟 import：没装 numpy 时也能 import 本模块
        self._np = np
        self._backend = _import_backend()
        self._mount = self._transform(ARM_MOUNT_XYZ, ARM_MOUNT_RPY)
        self._mount_inv = np.linalg.inv(self._mount)

    def _transform(self, xyz: Sequence[float], rpy: Sequence[float]):
        np = self._np
        roll, pitch, yaw = rpy
        rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        matrix = np.eye(4)
        matrix[:3, :3] = rz @ ry @ rx
        matrix[:3, 3] = xyz
        return matrix

    # -- FK ---------------------------------------------------------------
    def tool_in_arm_base(self, q: Sequence[float]):
        """末端在 aubo_base_link 下的 4x4 位姿。"""
        return self._backend.fk(self._np.array(list(q), dtype=float))

    def tool_in_base_link(self, q: Sequence[float]):
        """末端在机器人 base_link 下的 4x4 位姿。"""
        return self._mount @ self.tool_in_arm_base(q)

    def tool_position(self, q: Sequence[float]) -> tuple[float, float, float]:
        matrix = self.tool_in_base_link(q)
        return float(matrix[0, 3]), float(matrix[1, 3]), float(matrix[2, 3])

    # -- IK ---------------------------------------------------------------
    def solve_position(
        self,
        target_in_base_link: Sequence[float],
        q_start: Sequence[float] | None = None,
        *,
        tolerance: float = 0.01,
        damping: float = 0.08,
        max_iterations: int = 200,
        max_step: float = 0.08,
    ) -> tuple[bool, list[float], float]:
        """给 base_link 下的目标点，解出 6 个关节角。

        返回 (是否收敛, 关节角, 末端残差米)。★不收敛时也会返回目前最好的解，
        但 ok=False —— 上层必须据此判断该不该动，不要闷头执行。
        """
        np = self._np
        target = np.array(list(target_in_base_link), dtype=float)
        target_homogeneous = np.append(target, 1.0)
        target_in_arm = (self._mount_inv @ target_homogeneous)[:3]
        start = np.array(list(q_start if q_start is not None else HOME_POSE), dtype=float)
        ok, q, error, _iterations = self._backend.solve_position(
            start,
            target_in_arm,
            tolerance=tolerance,
            damping=damping,
            max_iterations=max_iterations,
            max_step=max_step,
        )
        return bool(ok), [float(v) for v in q], float(error)
