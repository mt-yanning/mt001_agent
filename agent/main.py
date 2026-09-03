"""mt001_agent v0.1 主程序：人话 → LLM 选一个技能 → 执行 → 读 odom 确认。

跑法（先起 mock 和 bridge，见 README）：
    python3 -m agent.main                      # 交互模式
    python3 -m agent.main -i "向前走一米"       # 单条指令
    python3 -m agent.main -i "..." --dry-run    # 只看决策不执行
    python3 -m agent.main -i "..." --yes        # 不逐条确认，直接执行

安全约定：
  · 任何异常、Ctrl-C、正常退出，都会走 safe_stop（见 primitives.connect）
  · aubo_teach 在原语层就被拉黑，LLM 根本看不到它
  · 默认每一步执行前都要你按回车确认，--yes 才跳过
"""

from __future__ import annotations

import argparse
import sys
import time

from . import skills as skill_lib
from .brain import Brain, Decision
from .config import load_settings
from .errors import BrainError, BridgeError
from .perception import GodViewPerception
from .primitives import connect


def describe_robot_state(bridge) -> str:
    """把机器人状态写成给 LLM 看的一段人话。"""
    lines = []
    try:
        x, y, yaw = bridge.odom()
        lines.append(f"底盘位姿：x={x:.2f}, y={y:.2f}, 朝向={yaw * 57.2958:.0f}°")
    except Exception as exc:
        lines.append(f"底盘位姿：读不到（{exc}）")
    position = bridge.gripper_position()
    if position is not None:
        lines.append(f"夹爪：position={position:.2f}（0.0 是张开，0.6 是闭合）")
    snapshot = bridge.status()
    lines.append(
        f"安全开关：motion_enabled={snapshot.get('motion_enabled')}, "
        f"confirm_agent_motion={snapshot.get('confirm_agent_motion')}"
    )
    active = snapshot.get("active") or {}
    if active.get("command"):
        lines.append(f"正在执行：{active['command']}，剩余 {active.get('remaining_sec', 0):.1f}s")
    return "\n".join(lines)


def run_once(bridge, brain, perception, settings, instruction: str, *,
             auto_yes: bool, dry_run: bool) -> bool:
    """处理一条人话指令。返回是否成功。"""
    print(f"\n{'=' * 68}\n指令：{instruction}\n{'=' * 68}")

    robot_state = describe_robot_state(bridge)
    scene = perception.summary_for_llm()
    schemas = skill_lib.all_schemas()

    print("正在请大脑决策……")
    started = time.monotonic()
    try:
        decision: Decision = brain.decide(
            instruction,
            skills=schemas,
            robot_state=robot_state,
            scene=scene,
            log=print,
        )
    except BrainError as exc:
        print(f"❌ 决策失败：{exc}")
        return False
    print(f"（耗时 {time.monotonic() - started:.1f}s）")

    print(f"\n【大脑的推理】{decision.think}")
    print(f"【决定调用】{decision.skill}({decision.args})")

    if decision.is_refusal:
        print("\n⚠ 大脑主动拒答：现有技能或场景信息不足以完成这条指令。")
        return False

    if decision.skill not in skill_lib.REGISTRY:
        print(f"\n❌ 大脑编了一个不存在的技能 {decision.skill!r}，已拦下。"
              f"可用技能：{sorted(skill_lib.REGISTRY)}")
        return False

    if dry_run:
        print("\n(--dry-run，不执行)")
        return True

    if not auto_yes:
        answer = input("\n执行吗？[y/N] ").strip().lower()
        if answer not in ("y", "yes"):
            print("已取消。")
            return False

    ctx = skill_lib.ExecutionContext(bridge=bridge, perception=perception, settings=settings)
    skill = skill_lib.get_skill(decision.skill)
    print()
    result = skill.run(ctx, **decision.args)
    print(f"\n{result}")
    return result.ok


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="mt001_agent v0.1 单步 Agent")
    parser.add_argument("-i", "--instruction", help="要执行的一句人话；不给则进交互模式")
    parser.add_argument("--yes", action="store_true", help="不逐条确认，直接执行")
    parser.add_argument("--dry-run", action="store_true", help="只出决策，不真的执行")
    args = parser.parse_args(argv)

    settings = load_settings()
    perception = GodViewPerception(settings.get("world_path") or None)

    try:
        brain = Brain(settings)
    except BrainError as exc:
        print(f"❌ 大脑初始化失败：{exc}")
        return 1

    with connect(
        command_topic=settings["command_topic"],
        status_topic=settings["status_topic"],
        event_topic=settings["event_topic"],
        tools_topic=settings["tools_topic"],
    ) as bridge:
        try:
            bridge.wait_ready(timeout=10.0)
        except BridgeError as exc:
            print(f"❌ {exc}")
            return 1

        allowed, reason = bridge.motion_allowed()
        print(f"✅ 已连上 agent_bridge。{reason}")
        if not allowed:
            print("   （只读技能仍可用，但机器人不会动）")
        print(f"✅ 世界模型来源：{perception.source}，{len(perception.objects())} 个物体")
        print(f"✅ 技能库：{sorted(skill_lib.REGISTRY)}")
        print(f"✅ 大脑：{brain.model} @ {brain.base_url}")

        if args.instruction:
            ok = run_once(bridge, brain, perception, settings, args.instruction,
                          auto_yes=args.yes, dry_run=args.dry_run)
            return 0 if ok else 1

        print("\n进入交互模式。输入人话指令，quit 退出。")
        while True:
            try:
                instruction = input("\n> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not instruction:
                continue
            if instruction.lower() in ("quit", "exit", "q", "退出"):
                break
            try:
                run_once(bridge, brain, perception, settings, instruction,
                         auto_yes=args.yes, dry_run=args.dry_run)
            except KeyboardInterrupt:
                print("\n⚠ 已中断，正在 safe_stop……")
                bridge.safe_stop()
        return 0


if __name__ == "__main__":
    sys.exit(main())
