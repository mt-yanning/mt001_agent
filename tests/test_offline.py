#!/usr/bin/env python3
"""离线测试：不需要 ROS2、不需要网络、不需要机器人。

Mac 上就能跑：
    cd ~/github仓库/robot/mt001_agent && python3 tests/test_offline.py

覆盖三件事：
  1. 感知桩能读出物体、中文别名能命中、找不到时老实返回 None
  2. 大脑的 JSON 抽取足够抗造（代码块、前后缀废话、嵌套对象）
  3. 技能的参数校验会拦住错参数（这是不让 LLM 把坏参数发给机器人的最后一道闸）
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from agent import skills as skill_lib          # noqa: E402
from agent.brain import extract_json, to_decision  # noqa: E402
from agent.errors import BrainError            # noqa: E402
from agent.perception import GodViewPerception  # noqa: E402
from agent.utils import clamp, wrap_angle      # noqa: E402


class TestPerception(unittest.TestCase):
    def setUp(self) -> None:
        self.perception = GodViewPerception()

    def test_loads_objects(self) -> None:
        objects = self.perception.objects()
        self.assertGreaterEqual(len(objects), 10)
        names = {o.name for o in objects}
        self.assertIn("work_table", names)
        self.assertIn("pick_bottle", names)
        self.assertNotIn("showroom_floor", names)   # 地面不是可交互目标

    def test_chinese_alias(self) -> None:
        self.assertEqual(self.perception.find("桌子").name, "work_table")
        self.assertEqual(self.perception.find("把瓶子拿来").name, "pick_bottle")
        self.assertEqual(self.perception.find("inspection_crate").name, "inspection_crate")

    def test_unknown_object_returns_none(self) -> None:
        # 找不到就得说找不到，不能瞎给一个 —— 这是 v1.0 拒答能力的地基
        self.assertIsNone(self.perception.find("一头大象"))


class TestBrainParsing(unittest.TestCase):
    def test_plain_json(self) -> None:
        decision = to_decision(extract_json('{"think":"a","skill":"gripper","args":{"command":"open"}}'))
        self.assertEqual(decision.skill, "gripper")
        self.assertEqual(decision.args["command"], "open")

    def test_markdown_fenced(self) -> None:
        raw = '好的。\n```json\n{"think":"走过去","skill":"navigate_to","args":{"x":1.5,"y":0.2}}\n```\n完成。'
        decision = to_decision(extract_json(raw), raw)
        self.assertEqual(decision.skill, "navigate_to")
        self.assertAlmostEqual(decision.args["x"], 1.5)

    def test_nested_braces(self) -> None:
        raw = '{"think":"里面有 {括号} 和 \\"引号\\"","skill":"safe_stop","args":{}}'
        decision = to_decision(extract_json(raw), raw)
        self.assertEqual(decision.skill, "safe_stop")

    def test_refusal(self) -> None:
        decision = to_decision(extract_json('{"think":"场景里没有大象","skill":"none","args":{}}'))
        self.assertTrue(decision.is_refusal)

    def test_garbage_raises(self) -> None:
        with self.assertRaises(BrainError):
            extract_json("我不知道该怎么办。")


class TestSkillValidation(unittest.TestCase):
    def test_registry(self) -> None:
        self.assertIn("navigate_to", skill_lib.REGISTRY)
        self.assertIn("gripper", skill_lib.REGISTRY)
        # 危险工具绝不能作为技能暴露给大脑
        self.assertNotIn("aubo_teach", skill_lib.REGISTRY)

    def test_missing_required_arg(self) -> None:
        navigate = skill_lib.REGISTRY["navigate_to"]
        with self.assertRaises(ValueError):
            navigate.validate({"x": 1.0})          # 少了 y

    def test_unknown_arg(self) -> None:
        navigate = skill_lib.REGISTRY["navigate_to"]
        with self.assertRaises(ValueError):
            navigate.validate({"x": 1.0, "y": 2.0, "speed": 99})

    def test_enum_enforced(self) -> None:
        gripper = skill_lib.REGISTRY["gripper"]
        self.assertEqual(gripper.validate({"command": "open"})["command"], "open")
        with self.assertRaises(ValueError):
            gripper.validate({"command": "半开"})

    def test_number_coercion(self) -> None:
        navigate = skill_lib.REGISTRY["navigate_to"]
        cleaned = navigate.validate({"x": "1.5", "y": 2})   # LLM 常把数字写成字符串
        self.assertIsInstance(cleaned["x"], float)
        self.assertAlmostEqual(cleaned["x"], 1.5)

    def test_schema_for_llm(self) -> None:
        schemas = {s["skill"]: s for s in skill_lib.all_schemas()}
        self.assertTrue(schemas["navigate_to"]["args"]["x"]["required"])
        self.assertFalse(schemas["navigate_to"]["args"]["tolerance"]["required"])


class TestUtils(unittest.TestCase):
    def test_wrap_angle(self) -> None:
        import math
        self.assertAlmostEqual(wrap_angle(3 * math.pi), math.pi, places=6)
        self.assertAlmostEqual(wrap_angle(-3 * math.pi / 2), math.pi / 2, places=6)

    def test_clamp(self) -> None:
        self.assertEqual(clamp(5.0, 0.2), 0.2)
        self.assertEqual(clamp(-5.0, 0.2), -0.2)
        self.assertEqual(clamp(0.1, 0.2), 0.1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
