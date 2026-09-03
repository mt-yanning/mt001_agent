"""技能层基类。

★ 这一层是本项目的核心工程量，也是论文的主要贡献点。

为什么需要它：bridge 公布的那 10 个"工具"其实是【执行器】(actuator)，
不是 BUMBLE 意义上的【技能】(skill)。base_velocity 只是"以 0.1 m/s 走
1 秒"，它不知道"走到桌子那边"是什么意思。中间这层参数化技能库是缺的、
需要我们造的。

每个技能必须回答三个问题（对应文献里的三个机制）：
  1. 前置条件满足吗？        —— IALP 的 grounding / SayCan 的 affordance
  2. 怎么用原语把它做出来？   —— VoxPoser / Code as Policies 的原语组合
  3. 到底做成了没有？        —— ConceptAgent / RoBridge 的闭环判成败

第 3 条尤其重要：bridge 对多个工具是静默转发，ok:true 不代表机器人动了。
所以 verify() 必须靠实测状态变化来判断，这是我们相对 autopick 的核心改进。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable

from ..errors import PreconditionFailed  # noqa: F401  供技能层 import

if TYPE_CHECKING:                      # 只在类型检查时需要，运行时不拉 rclpy
    from ..primitives import BridgeClient


@dataclass
class SkillResult:
    """技能执行结果。ok 是【实测确认】后的结论，不是 bridge 的回执。"""

    ok: bool
    message: str
    data: dict[str, Any] = field(default_factory=dict)
    verified: bool = True   # False = 指令发出去了，但没能力确认到底做没做成

    def __str__(self) -> str:
        mark = "✅" if self.ok else "❌"
        tail = "" if self.verified else "（⚠ 未能实测确认）"
        return f"{mark} {self.message}{tail}"


@dataclass
class ExecutionContext:
    """技能执行时能拿到的一切。"""

    bridge: "BridgeClient"
    perception: Any = None
    settings: dict[str, Any] = field(default_factory=dict)
    log: Callable[[str], None] = print

    def get(self, key: str, default: Any = None) -> Any:
        return self.settings.get(key, default)


class Skill:
    """所有技能的基类。"""

    name: str = ""
    description: str = ""
    # 参数表：{参数名: {"type": "number"/"string", "desc": ..., "required": bool, "enum": [...]}}
    args_schema: dict[str, dict[str, Any]] = {}

    # -- 给 LLM 看的自描述 ------------------------------------------------
    @classmethod
    def schema_for_llm(cls) -> dict[str, Any]:
        return {
            "skill": cls.name,
            "description": cls.description,
            "args": {
                key: {
                    "type": spec.get("type", "string"),
                    "desc": spec.get("desc", ""),
                    "required": bool(spec.get("required", False)),
                    **({"enum": spec["enum"]} if "enum" in spec else {}),
                }
                for key, spec in cls.args_schema.items()
            },
        }

    # -- 参数校验 ---------------------------------------------------------
    @classmethod
    def validate(cls, args: dict[str, Any]) -> dict[str, Any]:
        """校验并归一化 LLM 给的参数。参数不对就抛异常，绝不带着错参数往下发。"""
        cleaned: dict[str, Any] = {}
        for key, spec in cls.args_schema.items():
            if key not in args or args[key] is None:
                if spec.get("required"):
                    raise ValueError(f"技能 {cls.name} 缺少必填参数 {key}")
                continue
            value = args[key]
            kind = spec.get("type", "string")
            if kind == "number":
                try:
                    value = float(value)
                except (TypeError, ValueError):
                    raise ValueError(f"参数 {key} 应该是数字，收到 {value!r}")
            elif kind == "string":
                value = str(value)
                if "enum" in spec and value not in spec["enum"]:
                    raise ValueError(
                        f"参数 {key} 只能是 {spec['enum']} 之一，收到 {value!r}"
                    )
            cleaned[key] = value
        unknown = set(args) - set(cls.args_schema)
        if unknown:
            raise ValueError(f"技能 {cls.name} 不认识这些参数：{sorted(unknown)}")
        return cleaned

    # -- 三段式执行 -------------------------------------------------------
    def check_preconditions(self, ctx: ExecutionContext, **kwargs: Any) -> None:
        """不满足就抛 PreconditionFailed。默认只检查两把安全锁。"""
        allowed, reason = ctx.bridge.motion_allowed()
        if not allowed:
            raise PreconditionFailed(reason)

    def execute(self, ctx: ExecutionContext, **kwargs: Any) -> SkillResult:
        raise NotImplementedError

    def run(self, ctx: ExecutionContext, **kwargs: Any) -> SkillResult:
        """对外统一入口：校验参数 → 查前置条件 → 执行 → 出事必 safe_stop。"""
        try:
            cleaned = self.validate(kwargs)
        except ValueError as exc:
            return SkillResult(False, f"参数错误：{exc}")

        try:
            self.check_preconditions(ctx, **cleaned)
        except PreconditionFailed as exc:
            return SkillResult(False, f"前置条件不满足：{exc}")

        try:
            return self.execute(ctx, **cleaned)
        except KeyboardInterrupt:
            ctx.bridge.safe_stop()
            raise
        except Exception as exc:   # 任何意外都先把机器人停住，再报错
            ctx.bridge.safe_stop()
            return SkillResult(False, f"执行中出错（已 safe_stop）：{exc}")


# ---------------------------------------------------------------------------
# 技能注册表
# ---------------------------------------------------------------------------
REGISTRY: dict[str, type[Skill]] = {}


def register(cls: type[Skill]) -> type[Skill]:
    if not cls.name:
        raise ValueError(f"{cls.__name__} 没有设置 name")
    REGISTRY[cls.name] = cls
    return cls


def get_skill(name: str) -> Skill:
    key = str(name).strip()
    if key not in REGISTRY:
        raise KeyError(f"没有名为 {key!r} 的技能，可用的有：{sorted(REGISTRY)}")
    return REGISTRY[key]()


def all_schemas() -> list[dict[str, Any]]:
    return [cls.schema_for_llm() for cls in REGISTRY.values()]


def sleep_with_deadline(seconds: float) -> None:
    """普通 sleep，单独包一层是为了以后好插打断逻辑。"""
    if seconds > 0:
        time.sleep(seconds)
