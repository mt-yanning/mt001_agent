"""技能库。import 本包即完成所有技能的注册。"""

from .base import (  # noqa: F401
    ExecutionContext,
    PreconditionFailed,
    Skill,
    SkillResult,
    all_schemas,
    get_skill,
    REGISTRY,
)
from . import gripper as _gripper   # noqa: F401  触发注册
from . import navigate as _navigate  # noqa: F401  触发注册

__all__ = [
    "ExecutionContext",
    "PreconditionFailed",
    "Skill",
    "SkillResult",
    "all_schemas",
    "get_skill",
    "REGISTRY",
]
