#!/usr/bin/env python3
"""链路自检：不接 LLM，用写死的逻辑跑一遍全链路。

这是把「架构信息.txt 组合 A 的手工六步」自动化的版本。目的是在接 LLM 之前，
先证明这条链路是通的：

    本脚本 → /arachne/agent/command → agent_bridge → /cmd_vel
          → mock/Gazebo 执行 → /odom → status → 本脚本读回来确认

跑法（先起好 mock 和 bridge）：
    cd ~/mt001_agent && python3 scripts/check_link.py
    python3 scripts/check_link.py --distance 0.5   # 自定义前进距离
    python3 scripts/check_link.py --skip-gripper

任何一步失败都会告诉你最可能的原因。全绿 = 第二个里程碑达成。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from agent import skills as skill_lib          # noqa: E402
from agent.config import load_settings          # noqa: E402
from agent.errors import BridgeError            # noqa: E402
from agent.perception import GodViewPerception  # noqa: E402
from agent.primitives import connect            # noqa: E402

RESULTS: list[tuple[str, bool, str]] = []


def record(name: str, ok: bool, detail: str = "") -> bool:
    RESULTS.append((name, ok, detail))
    print(f"{'✅' if ok else '❌'} {name}" + (f" —— {detail}" if detail else ""))
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description="mt001_agent 链路自检（不接 LLM）")
    parser.add_argument("--distance", type=float, default=0.4, help="向前走多远，单位米")
    parser.add_argument("--skip-gripper", action="store_true", help="跳过夹爪测试")
    args = parser.parse_args()

    settings = load_settings()

    with connect(
        command_topic=settings["command_topic"],
        status_topic=settings["status_topic"],
        event_topic=settings["event_topic"],
        tools_topic=settings["tools_topic"],
    ) as bridge:
        # --- 1. 连上 bridge ------------------------------------------------
        try:
            bridge.wait_ready(timeout=10.0)
        except BridgeError as exc:
            record("连接 agent_bridge", False, str(exc))
            return 1
        record("连接 agent_bridge", True, "收到 status")

        # --- 2. 工具清单 ---------------------------------------------------
        tools = [t.get("name") for t in bridge.tools(timeout=3.0)]
        record("拿到工具清单", bool(tools), f"{len(tools)} 个：{tools}")
        if "aubo_teach" in tools:
            record("aubo_teach 已被拉黑", False, "原语层过滤失效，请检查 FORBIDDEN_TOOLS")
        else:
            record("aubo_teach 已被拉黑", True, "LLM 看不到这个危险工具")

        # --- 3. 两把安全锁 -------------------------------------------------
        allowed, reason = bridge.motion_allowed()
        record("运动开关", allowed, reason)

        # --- 4. 只读指令走通 -----------------------------------------------
        result = bridge.call("get_robot_state")
        record("get_robot_state", result["ok"], result["message"])

        # --- 5. 里程计新鲜 -------------------------------------------------
        try:
            x0, y0, yaw0 = bridge.odom(max_age_sec=2.0)
            record("读到里程计", True, f"当前 ({x0:.2f}, {y0:.2f}), 朝向 {math.degrees(yaw0):.0f}°")
        except BridgeError as exc:
            record("读到里程计", False,
                   f"{exc}。mock/真机是 /odom，Gazebo 是 /gz/odom —— "
                   "Gazebo 下要 ros2 run 并加 -p odom_topic:=/gz/odom")
            return summarize()

        if not allowed:
            print("\n⚠ 运动开关没开，后面的运动测试跳过。")
            return summarize()

        # --- 6. 闭环导航（核心）--------------------------------------------
        target_x = x0 + args.distance * math.cos(yaw0)
        target_y = y0 + args.distance * math.sin(yaw0)
        print(f"\n▶ 让 navigate_to 走到正前方 {args.distance} m 处 "
              f"({target_x:.2f}, {target_y:.2f})……")
        ctx = skill_lib.ExecutionContext(
            bridge=bridge,
            perception=GodViewPerception(settings.get("world_path") or None),
            settings=settings,
        )
        nav = skill_lib.get_skill("navigate_to")
        nav_result = nav.run(ctx, x=target_x, y=target_y)
        record("navigate_to 闭环", nav_result.ok, nav_result.message)

        x1, y1, _ = bridge.odom()
        moved = math.hypot(x1 - x0, y1 - y0)
        record("里程计确实变化了", moved > args.distance * 0.5,
               f"实测位移 {moved:.3f} m（目标 {args.distance:.2f} m）")

        # --- 7. 夹爪 --------------------------------------------------------
        if not args.skip_gripper:
            gripper = skill_lib.get_skill("gripper")
            for command in ("close", "open"):
                gripper_result = gripper.run(ctx, command=command)
                record(f"夹爪 {command}", gripper_result.ok, gripper_result.message)

        # --- 8. 安全停止 ----------------------------------------------------
        stop_result = bridge.call("safe_stop")
        record("safe_stop", stop_result["ok"], stop_result["message"])

    return summarize()


def summarize() -> int:
    passed = sum(1 for _, ok, _ in RESULTS if ok)
    total = len(RESULTS)
    print(f"\n{'=' * 68}\n自检结果：{passed}/{total} 通过")
    failed = [name for name, ok, _ in RESULTS if not ok]
    if failed:
        print("失败项：" + "、".join(failed))
        print("\n排查顺序：")
        print("  1) ros2 topic list | grep arachne   ← bridge 起来了吗")
        print("  2) ros2 topic echo /cmd_vel          ← 指令发出去了吗")
        print("  3) ros2 topic echo /odom             ← 执行层在跑吗")
        print("  4) bridge 的 launch 参数带了 motion_enabled:=true confirm_agent_motion:=true 吗")
        return 1
    print("全部通过 ✅ —— 链路是通的，可以接 LLM 了：python3 -m agent.main")
    return 0


if __name__ == "__main__":
    sys.exit(main())
