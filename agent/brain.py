"""大脑层：把人话变成一次技能调用。

v0.1 是【单步】大脑 —— 不分解任务、不记忆、一次只出一个技能调用。
任务分解（SayCan / VoxPoser）、两步决策（BUMBLE）、失败反思恢复
（ConceptAgent）都是 v0.5 的事，这里先把最小闭环跑通。

结构化输出格式借鉴 ManipLVM-R1 的 think + answer：强制模型先写推理再给答案，
既提升决策质量，也让我们能看懂它为什么这么决定（调试时非常有用）。

只用标准库发 HTTP，不引入 openai / requests 依赖 —— AutoDL 上少装一个包
就少一个坑。任何 OpenAI 兼容接口都能用。
"""

from __future__ import annotations

import json
import re
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from .config import PROJECT_ROOT, api_key, load_settings
from .errors import BrainError

PROMPT_PATH = PROJECT_ROOT / "config" / "prompts" / "single_step.txt"


@dataclass
class Decision:
    """大脑的一次决策。"""

    think: str
    skill: str
    args: dict[str, Any] = field(default_factory=dict)
    raw: str = ""

    @property
    def is_refusal(self) -> bool:
        """skill=none 表示大脑主动拒答（做不到 / 目标不存在）。

        这是 DynaMem「负结果拒答」思想的最简版本：宁可说做不到，
        也不要瞎编一个坐标然后把机器人开到墙里去。
        """
        return self.skill.strip().lower() in ("none", "", "null")


def extract_json(text: str) -> dict[str, Any]:
    """从模型回复里抠出第一个完整的 JSON 对象。

    模型经常会画蛇添足加上 ```json 代码块或前后缀说明，所以不能直接 json.loads。
    """
    cleaned = re.sub(r"^\s*```(?:json)?|```\s*$", "", text.strip(), flags=re.MULTILINE)
    try:
        return json.loads(cleaned)
    except json.JSONDecodeError:
        pass
    # 括号配平扫描，取第一个完整对象
    depth = 0
    start = -1
    in_string = False
    escape = False
    for index, char in enumerate(cleaned):
        if in_string:
            if escape:
                escape = False
            elif char == "\\":
                escape = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == "{":
            if depth == 0:
                start = index
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0 and start >= 0:
                try:
                    return json.loads(cleaned[start:index + 1])
                except json.JSONDecodeError:
                    start = -1
    raise BrainError(f"模型没有输出可解析的 JSON，原文：{text[:300]}")


def to_decision(payload: dict[str, Any], raw: str = "") -> Decision:
    skill = payload.get("skill", payload.get("tool", ""))
    args = payload.get("args", payload.get("arguments", {}))
    if not isinstance(args, dict):
        raise BrainError(f"args 必须是对象，收到 {args!r}")
    return Decision(
        think=str(payload.get("think", payload.get("reason", ""))).strip(),
        skill=str(skill).strip(),
        args=args,
        raw=raw,
    )


class Brain:
    """调用 OpenAI 兼容接口的单步决策大脑。"""

    def __init__(self, settings: dict[str, Any] | None = None) -> None:
        self.settings = settings or load_settings()
        self.base_url = str(self.settings["llm_base_url"]).rstrip("/")
        self.model = str(self.settings["llm_model"])
        self.temperature = float(self.settings.get("llm_temperature", 0.0))
        self.timeout = float(self.settings.get("llm_timeout_sec", 60.0))
        self.max_retries = int(self.settings.get("llm_max_retries", 1))
        self.prompt_template = self._load_prompt()

    @staticmethod
    def _load_prompt(path: Path | None = None) -> str:
        target = path or PROMPT_PATH
        try:
            return target.read_text(encoding="utf-8")
        except OSError as exc:
            raise BrainError(f"读不到提示词模板 {target}：{exc}")

    # ------------------------------------------------------------------
    def build_prompt(
        self,
        instruction: str,
        *,
        skills: list[dict[str, Any]],
        robot_state: str,
        scene: str,
    ) -> str:
        return self.prompt_template.format(
            skills=json.dumps(skills, ensure_ascii=False, indent=1),
            robot_state=robot_state,
            scene=scene,
            instruction=instruction,
        )

    def _post(self, messages: list[dict[str, str]]) -> str:
        key = api_key()
        if not key:
            raise BrainError(
                "没找到 API Key。请把它放进环境变量或 mt001_agent/.env，例如：\n"
                "  export MT001_LLM_API_KEY=sk-xxxx\n"
                "（.env 已在 .gitignore 里，不会被提交）"
            )
        body = json.dumps({
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature,
            "stream": False,
        }).encode("utf-8")
        request = urllib.request.Request(
            f"{self.base_url}/chat/completions",
            data=body,
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {key}",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", "ignore")[:300]
            raise BrainError(f"LLM 接口返回 {exc.code}：{detail}")
        except urllib.error.URLError as exc:
            raise BrainError(
                f"连不上 {self.base_url}：{exc.reason}。"
                "如果在 AutoDL 上，确认这个域名在可访问范围内。"
            )
        try:
            return payload["choices"][0]["message"]["content"]
        except (KeyError, IndexError):
            raise BrainError(f"LLM 返回结构不认识：{json.dumps(payload)[:300]}")

    def decide(
        self,
        instruction: str,
        *,
        skills: list[dict[str, Any]],
        robot_state: str,
        scene: str,
        log: Callable[[str], None] = lambda _msg: None,
    ) -> Decision:
        prompt = self.build_prompt(
            instruction, skills=skills, robot_state=robot_state, scene=scene
        )
        messages = [
            {"role": "system", "content": "你是严谨的机器人决策模块，只输出 JSON。"},
            {"role": "user", "content": prompt},
        ]
        last_error = ""
        for attempt in range(self.max_retries + 1):
            if attempt:
                log(f"[brain] 第 {attempt} 次重试，上次的问题：{last_error}")
                messages.append({
                    "role": "user",
                    "content": f"你上次的输出无法使用：{last_error}。请只输出符合格式的 JSON。",
                })
            raw = self._post(messages)
            try:
                return to_decision(extract_json(raw), raw)
            except BrainError as exc:
                last_error = str(exc)
                messages.append({"role": "assistant", "content": raw})
        raise BrainError(f"连续 {self.max_retries + 1} 次都没拿到可用决策：{last_error}")


class ScriptedBrain:
    """离线替身：不联网，按预设脚本返回决策。

    用途：① 单元测试；② 在还没配好 API Key 时先验证 ROS 链路通不通
    （scripts/check_link.py 就是这么用的）。
    """

    def __init__(self, decisions: list[Decision]) -> None:
        self._decisions = list(decisions)
        self.calls: list[str] = []

    def decide(self, instruction: str, **_kwargs: Any) -> Decision:
        self.calls.append(instruction)
        if not self._decisions:
            raise BrainError("ScriptedBrain 的脚本已经用完了")
        return self._decisions.pop(0)
