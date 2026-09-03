"""导航技能：navigate_to / turn_to。

原语组合思路（VoxPoser §3.2「组合原语」/ BUMBLE「参数化技能」）：
    navigate_to(x, y)
      = 反复执行 [读 odom → 算误差 → 发一小段 base_velocity → 实测确认]
        直到到位、超时、或判定卡死

为什么用 base_velocity 而不是 base_relative / base_turn：
    后两者只是把 JSON 转发给 grasp_task_server，仿真下没人接 = 静默失败，
    而 bridge 照样返回 ok:true。base_velocity 是三环境唯一完整闭环的运动工具。

死人开关：bridge 的 base_velocity 到 duration_sec 到期会自动归零（上限 2s）。
所以我们发的是一连串短指令，而不是发一次就撒手 —— 这反而是好事：
Agent 崩溃时机器人最多再走 2 秒就停。
"""

from __future__ import annotations

import math

from ..errors import StaleStateError
from ..utils import wrap_angle
from .base import ExecutionContext, PreconditionFailed, Skill, SkillResult, register, sleep_with_deadline

# bridge 的默认限速（launch 参数 max_base_linear_x / max_base_angular_z）
BRIDGE_MAX_LINEAR = 0.20      # m/s
BRIDGE_MAX_ANGULAR = 0.35     # rad/s
BRIDGE_MAX_DURATION = 2.0     # s


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class _BaseMotionSkill(Skill):
    """底盘运动技能的公共部分：一个"发一小步 → 实测确认"的闭环。"""

    def _tick(
        self,
        ctx: ExecutionContext,
        *,
        linear_x: float,
        angular_z: float,
        duration: float,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], dict]:
        """发一小段速度并等它走完，返回 (发之前的位姿, 走完的位姿, bridge回执data)。"""
        before = ctx.bridge.odom()
        result = ctx.bridge.call(
            "base_velocity",
            linear_x=round(linear_x, 4),
            angular_z=round(angular_z, 4),
            duration_sec=round(duration, 3),
        )
        if not result["ok"]:
            raise RuntimeError(f"bridge 拒绝了 base_velocity：{result['message']}")

        # ★ 从回执里读【实际生效】的值。bridge 的限幅是静默的，
        #   发 linear_x=100 会被悄悄夹到 0.20，不读回执就会算错预期位移。
        data = result.get("data") or {}
        actual_duration = float(data.get("duration_sec", duration))
        sleep_with_deadline(actual_duration + 0.25)   # 多等一点，让 odom 更新上来
        after = ctx.bridge.odom()
        return before, after, data

    @staticmethod
    def _moved(before, after) -> tuple[float, float]:
        """返回 (平移距离, 转角变化绝对值)。"""
        dist = math.hypot(after[0] - before[0], after[1] - before[1])
        turn = abs(wrap_angle(after[2] - before[2]))
        return dist, turn


@register
class NavigateTo(_BaseMotionSkill):
    name = "navigate_to"
    description = (
        "让底盘走到场景里的一个坐标点 (x, y)。会自己先转向再前进，"
        "并且每一步都读里程计确认真的动了。只管底盘，不动机械臂。"
    )
    args_schema = {
        "x": {"type": "number", "desc": "目标点的 x 坐标（米，世界坐标系）", "required": True},
        "y": {"type": "number", "desc": "目标点的 y 坐标（米，世界坐标系）", "required": True},
        "tolerance": {"type": "number", "desc": "到位判定半径，默认 0.15 米", "required": False},
    }

    def check_preconditions(self, ctx: ExecutionContext, **kwargs) -> None:
        super().check_preconditions(ctx, **kwargs)
        # 里程计必须是新鲜的，否则闭环无从谈起
        try:
            ctx.bridge.odom(max_age_sec=2.0)
        except StaleStateError as exc:
            raise PreconditionFailed(
                f"{exc}。mock/真机的里程计是 /odom，Gazebo 是 /gz/odom —— "
                "Gazebo 下要用 ros2 run 并加 -p odom_topic:=/gz/odom"
            )

    def execute(self, ctx: ExecutionContext, **kwargs) -> SkillResult:
        target_x = kwargs["x"]
        target_y = kwargs["y"]
        tolerance = kwargs.get("tolerance") or ctx.get("nav_position_tolerance", 0.15)

        yaw_tolerance = ctx.get("nav_yaw_tolerance", 0.12)          # rad
        tick_duration = min(ctx.get("nav_tick_duration", 0.6), BRIDGE_MAX_DURATION)
        max_linear = min(ctx.get("nav_max_linear", 0.18), BRIDGE_MAX_LINEAR)
        max_angular = min(ctx.get("nav_max_angular", 0.30), BRIDGE_MAX_ANGULAR)
        k_linear = ctx.get("nav_k_linear", 0.8)
        k_angular = ctx.get("nav_k_angular", 1.2)
        timeout = ctx.get("nav_timeout_sec", 90.0)
        stall_limit = int(ctx.get("nav_stall_ticks", 3))

        start = ctx.bridge.odom()
        start_distance = math.hypot(target_x - start[0], target_y - start[1])
        ctx.log(
            f"[navigate_to] 出发点 ({start[0]:.2f}, {start[1]:.2f})，"
            f"目标 ({target_x:.2f}, {target_y:.2f})，直线距离 {start_distance:.2f} m"
        )

        elapsed = 0.0
        stalled = 0
        ticks = 0

        while True:
            x, y, yaw = ctx.bridge.odom()
            dx, dy = target_x - x, target_y - y
            distance = math.hypot(dx, dy)
            if distance <= tolerance:
                ctx.bridge.safe_stop()
                return SkillResult(
                    True,
                    f"到达目标附近，当前 ({x:.2f}, {y:.2f})，残差 {distance:.2f} m，共 {ticks} 步",
                    {"x": x, "y": y, "yaw": yaw, "distance": distance, "ticks": ticks},
                )

            if elapsed > timeout:
                ctx.bridge.safe_stop()
                return SkillResult(
                    False,
                    f"超时（{timeout:.0f}s）未到达，还差 {distance:.2f} m",
                    {"x": x, "y": y, "distance": distance, "ticks": ticks},
                )

            heading_error = wrap_angle(math.atan2(dy, dx) - yaw)

            if abs(heading_error) > yaw_tolerance:
                # 先转向：原地转，不前进
                angular = _clamp(k_angular * heading_error, max_angular)
                if abs(angular) < 0.06:                      # 太小的速度驱动不动底盘
                    angular = math.copysign(0.06, heading_error)
                before, after, _ = self._tick(
                    ctx, linear_x=0.0, angular_z=angular, duration=tick_duration
                )
                _, turned = self._moved(before, after)
                expected = abs(angular) * tick_duration
                progressed = turned > expected * 0.25
                ctx.log(
                    f"[navigate_to] 第{ticks + 1}步 转向 {math.degrees(heading_error):+.0f}° "
                    f"实测转了 {math.degrees(turned):.1f}°"
                )
            else:
                # 再前进
                linear = _clamp(k_linear * distance, max_linear)
                linear = max(linear, 0.06)
                angular = _clamp(k_angular * heading_error, max_angular * 0.5)
                before, after, _ = self._tick(
                    ctx, linear_x=linear, angular_z=angular, duration=tick_duration
                )
                moved, _ = self._moved(before, after)
                expected = linear * tick_duration
                progressed = moved > expected * 0.25
                ctx.log(
                    f"[navigate_to] 第{ticks + 1}步 前进 期望 {expected:.3f} m "
                    f"实测 {moved:.3f} m，剩余 {distance:.2f} m"
                )

            ticks += 1
            elapsed += tick_duration + 0.25

            # ★ 静默失败检测：bridge 回了 ok，里程计却纹丝不动
            if progressed:
                stalled = 0
            else:
                stalled += 1
                if stalled >= stall_limit:
                    ctx.bridge.safe_stop()
                    return SkillResult(
                        False,
                        f"连续 {stalled} 步发出指令但里程计几乎没变化，判定底盘没动。"
                        "常见原因：执行层（mock/Gazebo）没起来、odom 话题名不对、"
                        "或者 bridge 的运动开关没打开",
                        {"x": x, "y": y, "distance": distance, "ticks": ticks},
                    )


@register
class TurnTo(_BaseMotionSkill):
    name = "turn_to"
    description = "让底盘原地转到某个朝向（角度，单位度，世界坐标系，0 度是 +x 方向）。"
    args_schema = {
        "yaw_deg": {"type": "number", "desc": "目标朝向，单位度", "required": True},
    }

    def execute(self, ctx: ExecutionContext, **kwargs) -> SkillResult:
        target_yaw = math.radians(kwargs["yaw_deg"])
        yaw_tolerance = ctx.get("nav_yaw_tolerance", 0.12)
        tick_duration = min(ctx.get("nav_tick_duration", 0.6), BRIDGE_MAX_DURATION)
        max_angular = min(ctx.get("nav_max_angular", 0.30), BRIDGE_MAX_ANGULAR)
        k_angular = ctx.get("nav_k_angular", 1.2)
        timeout = ctx.get("nav_timeout_sec", 45.0)

        elapsed = 0.0
        stalled = 0
        while True:
            _, _, yaw = ctx.bridge.odom()
            error = wrap_angle(target_yaw - yaw)
            if abs(error) <= yaw_tolerance:
                ctx.bridge.safe_stop()
                return SkillResult(
                    True,
                    f"转到 {math.degrees(yaw):.1f}°，残差 {math.degrees(abs(error)):.1f}°",
                    {"yaw_deg": math.degrees(yaw)},
                )
            if elapsed > timeout:
                ctx.bridge.safe_stop()
                return SkillResult(False, f"转向超时，还差 {math.degrees(abs(error)):.1f}°")

            angular = _clamp(k_angular * error, max_angular)
            if abs(angular) < 0.06:
                angular = math.copysign(0.06, error)
            before, after, _ = self._tick(
                ctx, linear_x=0.0, angular_z=angular, duration=tick_duration
            )
            _, turned = self._moved(before, after)
            elapsed += tick_duration + 0.25
            if turned > abs(angular) * tick_duration * 0.25:
                stalled = 0
            else:
                stalled += 1
                if stalled >= 3:
                    ctx.bridge.safe_stop()
                    return SkillResult(False, "连续 3 步发出转向指令但里程计朝向没变，判定底盘没动")
