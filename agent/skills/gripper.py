"""夹爪与安全技能。

夹爪是三个环境（mock / Gazebo / 真机）都订阅同一条话题
/arachne/gripper/command 的少数工具之一，所以这个技能到处都能用。

确认方式：读 gripper_status。mock 的格式是 "mock ms42dc position=0.60"，
open→0.0，close→0.6。读不到就老实说"没能确认"，不假装成功
—— 这正是 autopick 缺的那一环。
"""

from __future__ import annotations

import time

from .base import ExecutionContext, Skill, SkillResult, register, sleep_with_deadline

OPEN_POSITION = 0.0
CLOSE_POSITION = 0.6
POSITION_TOLERANCE = 0.15


@register
class GripperSkill(Skill):
    name = "gripper"
    description = "张开或闭合夹爪。command 只能是 open 或 close。"
    args_schema = {
        "command": {
            "type": "string",
            "desc": "open=张开，close=闭合",
            "required": True,
            "enum": ["open", "close"],
        },
    }

    def execute(self, ctx: ExecutionContext, **kwargs) -> SkillResult:
        command = kwargs["command"]
        expected = OPEN_POSITION if command == "open" else CLOSE_POSITION

        before = ctx.bridge.gripper_position()
        result = ctx.bridge.call("gripper", command=command)
        if not result["ok"]:
            return SkillResult(False, f"bridge 拒绝了夹爪指令：{result['message']}")

        # 等它动完再确认，最多等 3 秒
        deadline = time.monotonic() + 3.0
        position = None
        while time.monotonic() < deadline:
            sleep_with_deadline(0.3)
            position = ctx.bridge.gripper_position()
            if position is not None and abs(position - expected) <= POSITION_TOLERANCE:
                return SkillResult(
                    True,
                    f"夹爪已{'张开' if command == 'open' else '闭合'}（position={position:.2f}）",
                    {"command": command, "position": position, "before": before},
                )

        if position is None:
            # 拿不到反馈：指令确实发出去了，但没有证据说明它执行了
            return SkillResult(
                True,
                f"夹爪指令 {command} 已发出，但读不到 gripper_status，无法确认执行结果",
                {"command": command},
                verified=False,
            )
        return SkillResult(
            False,
            f"发出 {command} 后 3 秒，夹爪位置仍是 {position:.2f}（期望 {expected:.2f}），判定没动",
            {"command": command, "position": position, "expected": expected},
        )


@register
class SafeStopSkill(Skill):
    name = "safe_stop"
    description = "立刻安全停止：底盘归零、机械臂归零、夹爪停。任何不确定的情况都可以调它。"
    args_schema: dict = {}

    def check_preconditions(self, ctx: ExecutionContext, **kwargs) -> None:
        # safe_stop 不受运动开关门控，任何时候都该能调 —— 这是 bridge 的设计意图
        return

    def execute(self, ctx: ExecutionContext, **kwargs) -> SkillResult:
        ctx.bridge.safe_stop()
        return SkillResult(True, "已发出 safe_stop")


@register
class GetStateSkill(Skill):
    name = "get_state"
    description = "读取机器人当前状态（底盘位姿、夹爪、安全开关）。只读，不会让机器人动。"
    args_schema: dict = {}

    def check_preconditions(self, ctx: ExecutionContext, **kwargs) -> None:
        return   # 只读工具不受运动开关限制

    def execute(self, ctx: ExecutionContext, **kwargs) -> SkillResult:
        ctx.bridge.call("get_robot_state")
        try:
            x, y, yaw = ctx.bridge.odom()
            pose = f"位姿 ({x:.2f}, {y:.2f})，朝向 {round(yaw * 57.2958)}°"
            data = {"x": x, "y": y, "yaw": yaw}
        except Exception as exc:
            pose = f"里程计不可用（{exc}）"
            data = {}
        position = ctx.bridge.gripper_position()
        if position is not None:
            pose += f"，夹爪 position={position:.2f}"
            data["gripper_position"] = position
        return SkillResult(True, pose, data)
