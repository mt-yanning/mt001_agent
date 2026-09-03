"""不依赖 ROS2 的小工具。"""

from __future__ import annotations

import math


def wrap_angle(angle: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, limit: float) -> float:
    """对称限幅到 [-limit, limit]。"""
    return max(-limit, min(limit, value))
