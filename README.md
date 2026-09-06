# mt001_agent

基于大语言模型的具身智能 Agent，通过 ROS2 控制 Arachne 移动操控机器人。

## 项目简介

本项目实现一个 Agent 决策层：接收自然语言任务描述，利用 LLM 进行任务理解与分解，生成结构化指令，经由 Arachne 的 `arachne_agent_bridge` 驱动机器人执行。

## 目标平台

- 机器人：Arachne（Scout 2.0 底盘 + Aubo i5 机械臂 + MS42DC 夹爪 + Gemini335 相机 + C16 激光雷达）
- 中间件：ROS2 Humble / Ubuntu 22.04
- 接入点：`/arachne/agent/command`（JSON 指令）

## 目录结构

**v0.1 骨架已落地**

```
agent/
├── primitives.py      原语层：收发 /arachne/agent/command 与 /status，含新鲜度检查与静默失败防护
├── skills/
│   ├── base.py        Skill 基类：参数校验 → 前置条件 → 执行 → 实测确认
│   ├── navigate.py    navigate_to(x,y) / turn_to(yaw_deg)，base_velocity + odom 闭环
│   └── gripper.py     gripper(open/close) / safe_stop / get_state
├── brain.py           单步 LLM 决策（think + skill + args 结构化输出），含 ScriptedBrain 离线替身
├── perception.py      上帝视角桩：解析 arachne_showroom.sdf，带中文别名
├── kinematics.py      包一层 AuboI5Kinematics，补 base_link ↔ aubo_base_link 换算（v0.5 用）
├── config.py          配置加载（默认值 → settings.yaml → .env）
├── errors.py          共用异常（不依赖 rclpy，Mac 上也能 import）
└── main.py            主循环
config/
├── settings.yaml      运行参数
└── prompts/single_step.txt   单步决策提示词
scripts/check_link.py  ★不接 LLM 的全链路自检
tests/test_offline.py  离线单元测试（无需 ROS2/网络）
docs/                  架构文档、文献表、归类图
```

## 快速开始

### 0. 环境准备

```bash
source /root/autodl-tmp/Arachne/scripts/env/arachne_env.sh
source /root/autodl-tmp/Arachne/install/setup.bash
```

### 1. 离线自测

```bash
python3 tests/test_offline.py
```

### 2. 起 mock 与 bridge

```bash
# 终端 1
ros2 launch arachne_hardware mock_bringup.launch.py

# 终端 2 —— 两把安全锁必须显式打开，否则机器人不会动
ros2 launch arachne_agent_bridge agent_bridge.launch.py \
    motion_enabled:=true confirm_agent_motion:=true
```

Gazebo 环境下第二条要换成 `ros2 run`，因为 launch 文件没暴露 `odom_topic`：

```bash
ros2 run arachne_agent_bridge agent_bridge --ros-args \
    -p motion_enabled:=true -p confirm_agent_motion:=true \
    -p odom_topic:=/gz/odom
```

### 3. 链路自检（终端 3，不接 LLM）

```bash
python3 scripts/check_link.py
```

八项全绿就说明 `本进程 → bridge → /cmd_vel → 执行层 → /odom → 读回来确认` 这条闭环通了。

### 4. 接 LLM

```bash
cp .env.example .env      # 填进你的 API Key，.env 已在 .gitignore
python3 -m agent.main -i "向前走一米"
python3 -m agent.main                      # 交互模式
python3 -m agent.main -i "..." --dry-run   # 只看决策不执行
```

默认每一步执行前都会让你确认，`--yes` 跳过。


## 核心骨架（三层架构）

- 大脑 Brain (LLM) 
- 技能库 Skill Library（用原语组合中层技能）
- 原语 Primitive（bridge 10工具 + 机械臂通路 + AuboI5Kinematics）