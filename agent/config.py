"""配置加载：默认值 → config/settings.yaml → .env / 环境变量，后者覆盖前者。

API Key 只从环境变量或 .env 读，绝不写进 yaml、绝不提交（.gitignore 已覆盖）。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parent.parent

DEFAULTS: dict[str, Any] = {
    # ---- LLM ----
    # 任何 OpenAI 兼容接口都行：DeepSeek / 通义千问 / 智谱 / Kimi / vLLM 本地部署
    "llm_base_url": "https://api.deepseek.com/v1",
    "llm_model": "deepseek-chat",
    "llm_temperature": 0.0,      # 决策要可复现，别让它发挥
    "llm_timeout_sec": 60.0,
    "llm_max_retries": 1,        # JSON 解析失败后重试次数

    # ---- 导航闭环 ----
    "nav_position_tolerance": 0.15,   # m，到位判定半径
    "nav_yaw_tolerance": 0.12,        # rad，约 7°
    "nav_tick_duration": 0.6,         # s，每小步时长（bridge 上限 2.0）
    "nav_max_linear": 0.18,           # m/s（bridge 上限 0.20）
    "nav_max_angular": 0.30,          # rad/s（bridge 上限 0.35）
    "nav_k_linear": 0.8,
    "nav_k_angular": 1.2,
    "nav_timeout_sec": 90.0,
    "nav_stall_ticks": 3,             # 连续几步没动就判定静默失败

    # ---- 话题（Gazebo 下 bridge 需要 -p odom_topic:=/gz/odom，不在这里改）----
    "command_topic": "/arachne/agent/command",
    "status_topic": "/arachne/agent/status",
    "event_topic": "/arachne/agent/event",
    "tools_topic": "/arachne/agent/tools",

    # ---- 场景 ----
    "world_path": "",   # 留空则自动找 arachne_showroom.sdf
}

# 环境变量名 → 配置键
ENV_MAP = {
    "MT001_LLM_BASE_URL": ("llm_base_url", str),
    "MT001_LLM_MODEL": ("llm_model", str),
    "MT001_LLM_TEMPERATURE": ("llm_temperature", float),
    "ARACHNE_WORLD": ("world_path", str),
}


def load_dotenv(path: Path | None = None) -> None:
    """把 .env 里的 KEY=VALUE 读进 os.environ（不覆盖已有的）。"""
    target = path or (PROJECT_ROOT / ".env")
    try:
        text = target.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key, value = key.strip(), value.strip().strip('"').strip("'")
        os.environ.setdefault(key, value)


def load_settings(path: Path | None = None) -> dict[str, Any]:
    settings = dict(DEFAULTS)

    target = path or (PROJECT_ROOT / "config" / "settings.yaml")
    try:
        import yaml   # ROS2 环境自带；没有就跳过，用默认值
        text = target.read_text(encoding="utf-8")
        loaded = yaml.safe_load(text) or {}
        if isinstance(loaded, dict):
            settings.update(loaded)
    except Exception:
        pass

    load_dotenv()
    for env_key, (key, caster) in ENV_MAP.items():
        raw = os.environ.get(env_key)
        if raw:
            try:
                settings[key] = caster(raw)
            except ValueError:
                pass
    return settings


def api_key() -> str:
    """按优先级找 API Key。找不到返回空串，由调用方报错。"""
    load_dotenv()
    for name in ("MT001_LLM_API_KEY", "DEEPSEEK_API_KEY", "OPENAI_API_KEY", "DASHSCOPE_API_KEY"):
        value = os.environ.get(name)
        if value:
            return value.strip()
    return ""
