"""
原语层：封装与 arachne_agent_bridge 的全部 ROS2 通信
设计依据：
1、Code as Policies §III -- 把控制基元封装成 API 供上层调用
"""

from __future__ import annotations # 类型注解延迟求值

import json        # JSON 编解码,agent 和 bridge 完全靠 JSON 通信
import math        # atan2、hypot、radians —— 后面算距离和角度要用
import threading   # 锁 + 条件变量,做多线程同步
import time        # time.monotonic() 单调时钟
from contextlib import contextmanager  # 装饰器,把一个普通生成器函数变成上下文管理器
from typing import Any, Iterator       # 类型注解

import rclpy                                          # ROS2 的 Python 库。包含节点、发布器、订阅器、参数、日志等一整套。
from rclpy.executors import SingleThreadedExecutor    # 所有 ROS2 节点的基类
from rclpy.node import Node                           # 执行器
from std_msgs.msg import String                       # ROS2 标准消息类型：一个字符串字段 

from .errors import BridgeError, StaleStateError, ToolForbidden  # noqa: F401
from .utils import wrap_angle  # noqa: F401


# 使用不可变集合禁用 aubo 工具（安全红线）
FORBIDDEN_TOOLS = frozenset({"aubo_teach", "teach"})

# bridge 公布的 10 个工具里，本项目允许使用的 9 个
KNOWN_TOOLS = frozenset({
    "get_robot_state",
    "safe_stop",
    "arm_stop",
    "base_velocity",
    "base_relative",
    "base_turn",
    "arm_cartesian_jog",
    "arm_joint_jog",
    "gripper",
})

class BridgeClient(Node):
    """
    一个 rclpy 节点，负责收发 agent_bridge 的四条话题
    """

    def __init__(
        self,
        *,
        node_name: str = "mt001_agent_client",
        command_topic: str = "/arachne/agent/command",
        status_topic: str = "/arachne/agent/status",
        event_topic: str = "/arachne/agent/event",
        tools_topic: str = "/arachne/agent/tools",
    ) -> None:
        super().__init__(node_name)
        
        #  用锁实现并发原语
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock) # 设置一把锁+唤醒机制

        #  存放从 bridge 收到的所有信息
        self._status: dict[str, Any] | None = None
        self._status_stamp: float = 0.0           # 本地收到 status 的时刻
        self._tools: list[dict[str, Any]] | None = None
        self._events: list[dict[str, Any]] = []   # 只留最近 200 条
        self._event_seq: int = 0                  # 收到的事件总数，用于定位新事件

        self._command_pub = self.create_publisher(String, command_topic, 10) # 参数：消息类型, 发布者话题名, 队列深度
        self.create_subscription(String, status_topic, self._on_status, 10)  # 参数：消息类型，订阅者话题名，回调函数，队列深度
        self.create_subscription(String, event_topic, self._on_event, 10)
        self.create_subscription(String, tools_topic, self._on_tools, 1)

    
    # 订阅回调
    def _on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._cv:             # 加锁写缓存
            self._status = data    # 覆盖，只保留最新状态
            self._status_stamp = time.monotonic()
            self._cv.notify_all()

    def _on_event(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._cv:
            self._event_seq += 1            # 全局事件计数器
            data["_seq"] = self._event_seq  # 记录事件数量
            self._events.append(data)       # 追加事件
            del self._events[:-200]
            self._cv.notify_all()

    def _on_tools(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._cv:
            self._tools = data.get("tools", [])
            self._cv.notify_all()


    # 超时等待接收到数据，确认与 bridge 的连接
    def wait_ready(self, timeout: float = 10.0) -> dict[str, Any]:
        """等待直到第一帧 status 到达为止，说明 bridge 已连接"""
        deadline = time.monotonic() + timeout
        with self._cv:
            while self._status is None: # 条件:变量非空
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise BridgeError(
                        "等不到 /arachne/agent/status。请确认 agent_bridge 已启动："
                        "ros2 launch arachne_agent_bridge agent_bridge.launch.py "
                        "motion_enabled:=true confirm_agent_motion:=true"
                    )
                self._cv.wait(remaining)
            return dict(self._status)

    def tools(self, timeout: float = 5.0) -> list[dict[str, Any]]:
        """bridge 自己公布的工具清单"""
        deadline = time.monotonic() + timeout
        with self._cv:
            while self._tools is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return []      # 超时不抛异常
                self._cv.wait(remaining)
            tools = list(self._tools)
        return [t for t in tools if str(t.get("name", "")).lower() not in FORBIDDEN_TOOLS]


    # 状态读取
    def status(self, *, max_age_sec: float | None = None) -> dict[str, Any]: # max_age_sec 用于检查数据新鲜度
        with self._cv: # 加锁保护对 _status 的读(可能后台线程正在写)
            if self._status is None:
                raise BridgeError("还没收到任何 status，请先调用 wait_ready()")
            age = time.monotonic() - self._status_stamp # 算年龄 = 现在时刻 - 本地收到 status 的时刻
            snapshot = dict(self._status)
        if max_age_sec is not None and age > max_age_sec:
            raise StaleStateError(f"status 已经 {age:.2f}s 没更新（上限 {max_age_sec}s）")
        return snapshot

    def _latest(self, key: str, max_age_sec: float) -> Any:
        """读 status.latest[key]，并把 bridge 侧年龄 + 本地传输年龄一起算进来"""
        with self._cv:
            if self._status is None:
                raise BridgeError("还没收到任何 status，请先调用 wait_ready()")
            local_age = time.monotonic() - self._status_stamp    # 本地传输年龄
            item = (self._status.get("latest") or {}).get(key)
        if item is None:
            raise StaleStateError(f"bridge 从来没收到过 {key} 数据")
        total_age = float(item.get("age_sec", 0.0)) + local_age
        if total_age > max_age_sec:
            raise StaleStateError(f"{key} 数据已经 {total_age:.2f}s 没更新（上限 {max_age_sec}s）")
        return item.get("value")

    def odom(self, *, max_age_sec: float = 1.5) -> tuple[float, float, float]:
        """返回底盘位姿 (x, y, yaw)"""
        value = self._latest("odom", max_age_sec)
        return float(value["x"]), float(value["y"]), float(value["yaw"])

    def arm_joints(self, *, max_age_sec: float = 2.0) -> dict[str, float]:
        """返回 6 个关节角"""
        value = self._latest("aubo_joints", max_age_sec)
        return {k: float(v) for k, v in value.items()}
        # 返回结构:
        # {
        #     "shoulder_joint": -1.497,
        #     "upperArm_joint": 0.018,
        #     "foreArm_joint":  1.601,
        #     "wrist1_joint":   0.538,
        #     "wrist2_joint":   1.593,
        #     "wrist3_joint":   0.889,
        # }

    ###这个函数有待考证
    def gripper_position(self, *, max_age_sec: float = 3.0) -> float | None:
        """
        从 gripper_status 文本里抠出行程值。mock 格式：'mock ms42dc position=0.60'。
        解析不出来就返回 None
        """
        try:
            text = str(self._latest("gripper_status", max_age_sec))
        except StaleStateError:
            return None
        marker = "position="
        index = text.find(marker)
        if index < 0:
            return None
        try:
            return float(text[index + len(marker):].split()[0])
        except (ValueError, IndexError):
            return None

    def motion_allowed(self) -> tuple[bool, str]:
        """检查两把安全锁"""
        snapshot = self.status()
         # bridge 把两个启动参数直接放在 status 的顶层，直接 get() 即可
        enabled = bool(snapshot.get("motion_enabled")) 
        confirmed = bool(snapshot.get("confirm_agent_motion"))
        if enabled and confirmed:
            return True, "运动已解锁"
        missing = []
        if not enabled:
            missing.append("motion_enabled")
        if not confirmed:
            missing.append("confirm_agent_motion")
        return False, (
            f"bridge 的 {' 和 '.join(missing)} 还是 false，机器人不会动。"
            "重启 bridge 时加上 motion_enabled:=true confirm_agent_motion:=true"
        )

    # 发指令
    def call(self, tool: str, *, timeout: float = 3.0, **params: Any,) -> dict[str, Any]: # **params 收集所有额外的关键字参数转为一个字典
        """
        发一条工具指令，并等回它对应的 command_result 事件，返回 bridge 的执行结果 {"ok", "message", "data"}

        ★ 注意：ok=True 只代表"bridge 受理了这条指令"，不代表机器人真的动了。
          实测确认是技能层的责任，见 skills/base.py。
        """
        # 规范化名字，安全拦截
        name = str(tool).strip().lower() 
        if name in FORBIDDEN_TOOLS:
            raise ToolForbidden(f"工具 {name} 被本项目禁用（安全红线，见 AGENTS.md）")
        
        # 组装 JSON 载荷
        payload: dict[str, Any] = {"tool": name}
        payload.update({k: v for k, v in params.items() if v is not None})

        with self._cv:
            seq_before = self._event_seq

        # 发消息
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._command_pub.publish(msg)

        # 等回执
        event = self._wait_command_result(seq_before, timeout)
        if event is None:
            raise BridgeError(
                f"发了 {name} 但 {timeout}s 内没等到 command_result 事件。"
                "bridge 可能已经挂了，或者话题名不对。"
            )
        result = event.get("result") or {}
        return {
            "ok": bool(result.get("ok")),
            "message": str(result.get("message", "")),
            "data": result.get("data") or {},
            "tool": name,
        }

    def _wait_command_result(self, seq_after: int, timeout: float) -> dict[str, Any] | None:
        """ 底层的"等特定事件" """
        deadline = time.monotonic() + timeout
        with self._cv:
            while True:
                for event in self._events: # 条件:遍历事件流找符合的
                    if event.get("_seq", 0) > seq_after and event.get("kind") == "command_result":
                        # 两个条件：只看发指令之后产生的事件,老事件全部跳过；筛选需要的事件流
                        return event
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._cv.wait(remaining)

    # 安全
    def safe_stop(self) -> None:
        """无条件安全停止。任何异常、超时、不确定的情况都应该调它"""
        try:
            self.call("safe_stop", timeout=2.0) # 主方案:走正常 call 流程
        except Exception:
            # 兜底：连事件都等不回来时，至少把指令发出去
            msg = String()
            msg.data = json.dumps({"tool": "safe_stop"})
            self._command_pub.publish(msg)   


@contextmanager
def connect(**kwargs: Any) -> Iterator[BridgeClient]:
    """
    开一个 BridgeClient，自动起 spin 线程，退出时保证 safe_stop + 关闭。
    """
    rclpy.init()
    client = BridgeClient(**kwargs)
    executor = SingleThreadedExecutor()
    executor.add_node(client)
    thread = threading.Thread(target=executor.spin, daemon=True, name="mt001_agent_spin")
    thread.start()
    try:
        yield client
    finally:
        try:
            client.safe_stop()
            time.sleep(0.2)   # 给 safe_stop 一点发出去的时间
        except Exception:
            pass
        executor.shutdown()
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)
