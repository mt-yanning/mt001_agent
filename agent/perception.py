"""感知层（v0.1 是桩）：上帝视角世界模型。

★ 诚实声明：这里【不做任何真实感知】。物体坐标是从 Gazebo 的世界文件
  arachne_showroom.sdf 里直接读出来的，等于给了 Agent 一双"上帝之眼"。

  这是 v0.1 的有意简化，理由见架构文档 1.4：当前 Gazebo 环境
  with_ee_camera:=false 且 ros_gz_bridge 没桥接任何图像话题，物理上就
  拿不到图像。v1.0 会把这一层换成"末端相机 → VLM（Set-of-Mark）→ 定位"，
  接口形状保持不变，替换时上层技能与大脑都不用改。

  参考：BUMBLE 的开放世界 RGB-D 感知 / DynaMem 的深度反投影建图，都是
  这一层将来要长成的样子。
"""

from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

# ---------------------------------------------------------------------------
# 兜底世界模型：arachne_showroom.sdf 里 16 个模型的坐标（从架构审查抄录）。
# SDF 解析不到时用它，保证没有 Arachne 源码的机器上也能跑通离线测试。
# ---------------------------------------------------------------------------
FALLBACK_MODELS: dict[str, tuple[float, float, float]] = {
    "demo_pedestal":     (-1.20, -1.40, 0.25),
    "soft_target_box":   ( 1.00, -1.50, 0.18),
    "marker_column":     (-2.00,  1.40, 0.45),
    "start_lane_marker": (-2.70,  0.00, 0.046),
    "slalom_marker_a":   (-1.40,  2.10, 0.28),
    "slalom_marker_b":   (-0.40,  1.55, 0.28),
    "slalom_marker_c":   ( 0.60,  2.10, 0.28),
    "work_table":        ( 2.45,  1.70, 0.42),
    "foam_block_a":      ( 2.25,  1.58, 0.91),
    "foam_block_b":      ( 2.60,  1.82, 0.89),
    "inspection_crate":  ( 3.45,  0.25, 0.22),
    "pick_target_pad":   ( 3.40, -2.35, 0.047),
    "pick_bottle":       ( 3.40, -2.35, 0.27),
    "pick_ball":         ( 3.85, -2.05, 0.16),
}

# 给 LLM 用的中文别名。人说"桌子"，Agent 要知道指的是 work_table。
ALIASES: dict[str, list[str]] = {
    "work_table":       ["桌子", "工作台", "台子"],
    "pick_bottle":      ["瓶子", "水瓶", "饮料"],
    "pick_ball":        ["球", "小球"],
    "inspection_crate": ["筐", "检查筐", "箱子", "收纳筐"],
    "foam_block_a":     ["泡沫块A", "泡沫块 A", "方块A"],
    "foam_block_b":     ["泡沫块B", "泡沫块 B", "方块B"],
    "slalom_marker_a":  ["桩A", "第一个桩"],
    "slalom_marker_b":  ["桩B", "第二个桩"],
    "slalom_marker_c":  ["桩C", "第三个桩"],
    "start_lane_marker": ["起点", "起始线"],
    "demo_pedestal":    ["台座", "演示台"],
    "marker_column":    ["标记柱", "柱子"],
    "soft_target_box":  ["软箱", "软目标箱"],
}

# 障碍物半径（autopick 手抄的那份，用于 v0.5 的可行性检查）
OBSTACLE_RADIUS: dict[str, float] = {
    "demo_pedestal": 0.58,
    "soft_target_box": 0.45,
    "marker_column": 0.38,
    "slalom_marker_a": 0.32,
    "slalom_marker_b": 0.32,
    "slalom_marker_c": 0.32,
    "work_table": 0.62,
    "inspection_crate": 0.50,
}


@dataclass
class SceneObject:
    name: str
    x: float
    y: float
    z: float
    aliases: list[str] = field(default_factory=list)
    radius: float | None = None

    def describe(self) -> str:
        alias = f"（{'/'.join(self.aliases)}）" if self.aliases else ""
        return f"{self.name}{alias} 位于 ({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"


class GodViewPerception:
    """从 SDF 世界文件读物体位姿的"感知"。"""

    def __init__(self, world_path: str | os.PathLike[str] | None = None) -> None:
        self.world_path = Path(world_path) if world_path else self._guess_world_path()
        self._objects: dict[str, SceneObject] | None = None
        self.source = "unknown"

    # -- 定位 world 文件 -------------------------------------------------
    @staticmethod
    def _guess_world_path() -> Path | None:
        """按优先级找 arachne_showroom.sdf。"""
        env = os.environ.get("ARACHNE_WORLD")
        if env:
            return Path(env)
        root = os.environ.get("ARACHNE_ROOT")
        candidates = []
        if root:
            candidates.append(Path(root) / "src/arachne_demo/worlds/arachne_showroom.sdf")
        candidates += [
            Path("/root/autodl-tmp/Arachne/src/arachne_demo/worlds/arachne_showroom.sdf"),
            Path.home() / "github仓库/robot/Arachne/src/arachne_demo/worlds/arachne_showroom.sdf",
        ]
        for candidate in candidates:
            try:
                if candidate.is_file():
                    return candidate
            except OSError:
                continue   # 权限不足等情况直接跳过，别让感知层把整个 Agent 拖崩
        return None

    # -- 解析 ------------------------------------------------------------
    def _parse_sdf(self, path: Path) -> dict[str, SceneObject]:
        """读 <model name="X"> 下的第一个 <pose>，取前三个数当 xyz。"""
        objects: dict[str, SceneObject] = {}
        tree = ET.parse(path)
        world = tree.getroot().find("world")
        if world is None:
            return objects
        for model in world.findall("model"):
            name = model.get("name")
            if not name:
                continue
            pose = model.find("pose")
            if pose is None or not (pose.text or "").strip():
                continue
            parts = re.split(r"\s+", pose.text.strip())
            if len(parts) < 3:
                continue
            try:
                x, y, z = (float(parts[i]) for i in range(3))
            except ValueError:
                continue
            objects[name] = SceneObject(
                name=name, x=x, y=y, z=z,
                aliases=ALIASES.get(name, []),
                radius=OBSTACLE_RADIUS.get(name),
            )
        return objects

    def _load(self) -> dict[str, SceneObject]:
        if self._objects is not None:
            return self._objects
        objects: dict[str, SceneObject] = {}
        try:
            readable = bool(self.world_path) and self.world_path.is_file()
        except OSError:
            readable = False
        if readable:
            try:
                objects = self._parse_sdf(self.world_path)
                self.source = f"SDF: {self.world_path}"
            except (ET.ParseError, OSError):
                objects = {}
        if not objects:
            objects = {
                name: SceneObject(
                    name=name, x=x, y=y, z=z,
                    aliases=ALIASES.get(name, []),
                    radius=OBSTACLE_RADIUS.get(name),
                )
                for name, (x, y, z) in FALLBACK_MODELS.items()
            }
            self.source = "内置兜底表（没找到 SDF 文件）"
        # 地面和相机锚点不是可交互目标，去掉
        for junk in ("showroom_floor", "camera_view_anchor", "ground_plane", "sun"):
            objects.pop(junk, None)
        self._objects = objects
        return objects

    # -- 对外接口 --------------------------------------------------------
    def objects(self) -> list[SceneObject]:
        return sorted(self._load().values(), key=lambda o: o.name)

    def find(self, query: str) -> SceneObject | None:
        """按名字或中文别名找物体。找不到返回 None —— 上层应据此拒答，不要瞎猜。"""
        objects = self._load()
        text = query.strip()
        if text in objects:
            return objects[text]
        lowered = text.lower()
        for name, obj in objects.items():
            if name.lower() == lowered:
                return obj
        for obj in objects.values():
            for alias in obj.aliases:
                if alias in text or text in alias:
                    return obj
        for name, obj in objects.items():
            if lowered and lowered in name.lower():
                return obj
        return None

    def summary_for_llm(self, limit: int = 20) -> str:
        lines = [obj.describe() for obj in self.objects()[:limit]]
        return "\n".join(lines) if lines else "（场景里没有已知物体）"
