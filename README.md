# mt001_agent

基于大语言模型的具身智能 Agent，通过 ROS2 控制 Arachne 移动操控机器人。

## 项目简介

本项目实现一个 Agent "大脑"层：接收自然语言任务描述，
利用 LLM 进行任务理解与分解，生成结构化指令，
经由 Arachne 的 `arachne_agent_bridge` 驱动机器人执行。

## 目标平台

- 机器人：Arachne（Scout 2.0 底盘 + Aubo i5 机械臂 + MS42DC 夹爪 + Gemini335 相机 + C16 激光雷达）
- 中间件：ROS2 Humble / Ubuntu 22.04
- 接入点：`/arachne/agent/command`（JSON 指令）

## 目录结构

agent/ Agent 核心逻辑
tools/ 工具封装（对应 agent_bridge 的 10 个工具）
config/ 配置文件
docs/ 设计文档与调研笔记
tests/ 测试脚本
scripts/ 启动与调试脚本

## 当前状态

**v0.1 骨架已落地**，可在 mock 上端到端运行。目录：

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

### 0. 环境准备（每个新终端都要）

```bash
source /root/autodl-tmp/Arachne/scripts/env/arachne_env.sh
source /root/autodl-tmp/Arachne/install/setup.bash
```

### 1. 离线自测（不需要 ROS2，Mac 上也能跑）

```bash
python3 tests/test_offline.py
```

### 2. 起 mock 与 bridge（两个终端）

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

## 设计要点

**为什么技能层不可省。** bridge 那 10 个工具是**执行器**，不是 BUMBLE 意义上的技能：
`base_velocity` 只是"以 0.1 m/s 走 1 秒"，它不知道"走到桌子那边"是什么意思。
中间这层参数化技能库是缺的、需要造的，也是本项目的主要工程量。

**为什么每个技能都要实测确认。** agent_bridge 对多个工具只做静默转发，不检查有没有人接：
`base_relative` / `base_turn` 在仿真下没有订阅者，`arm_jog` 在 mock/Gazebo 下没人订阅其输出话题
——三者都会返回 `ok:true` 但机器人纹丝不动。所以本项目约定：

> **`ok:true` 只代表 bridge 受理了指令，不代表机器人动了。**

`navigate.py` 里每一步都比对指令前后的里程计，连续 3 步没有预期位移就判定静默失败并报告
最可能的原因。这个"发指令 → 观察 → 确认"闭环，正是 `gazebo_autopick_planner.py`
（开环、无反馈、状态机单向推进）缺的东西，也是本项目相对它的核心改进。

**安全。** `aubo_teach` 在原语层就被拉黑，LLM 连见都见不到；任何异常、超时、Ctrl-C
都会走 `safe_stop`；bridge 的限速是静默夹紧的，所以技能从回执的 `data` 字段读**实际生效值**
来推算预期位移，而不是假设发什么就是什么。

## 核心骨架（三层架构）

- 大脑 Brain (LLM) 
- 技能库 Skill Library（用原语组合中层技能）
- 原语 Primitive（bridge 10工具 + 机械臂通路 + AuboI5Kinematics）

## 初级版
目标: 人话 → LLM 选一个技能 → 执行 → 读 odom 确认。单步、无记忆、无分解
任务：”向前走到桌子那边“  ”把夹爪张开“ “抓坐标已知的瓶子“

模块: 原语封装 primitives.py
做什么: 用 rclpy 收发 /arachne/agent/command 和/status,封装底盘/夹爪
参考论文的哪部分: Code as Policies §III「把控制基元封成API 供调用」
────────────────────────────────────────
模块: 极简技能库 skills/
做什么: 只 2 个技能:navigate_to(x,y)(base_velocity+odom闭环)、gripp(open/close)
参考论文的哪部分: VoxPoser §3.2「组合原语」;BUMBLE「参数化技能」骨架
────────────────────────────────────────
模块: 运动学工具 kinematics.py
做什么: 直接 import AuboI5Kinematics,包一层
参考论文的哪部分: (复用,不算模块)
────────────────────────────────────────
模块: 单步大脑 brain.py
做什么: 人话 + 工具清单 + 状态 → LLM 输出一条JSON。强制"先想后答"格式
参考论文的哪部分: EmbodiedBench §4.3「五步规划器」的简化版(描述→JSON);ManipLVM-R1「think+answer结构化输出」
────────────────────────────────────────
模块: 闭环确认 main.py
做什么: 发指令后读 status.latest.odom 实测确认动没动
参考论文的哪部分: SayCan「能不能做(affordance)」思想;IALP「闭环」
────────────────────────────────────────
模块: 感知(桩) perception.py
做什么: 上帝视角:从 showroom.sdf 读物体坐标,返回列表
参考论文的哪部分: (阶段占位,诚实标注简化)
────────────────────────────────────────
模块: 安全
做什么: 尊重 motion_enabled;异常必调 safe_stop;不给 LLM
aubo_teach
参考论文的哪部分: Arachne AGENTS.md 红线

可演示任务: "向前走到桌子那边" / "把夹爪张开"。
里程碑: LLM 自动发出你手敲过的那条 JSON,并确认机器人动了。

## 升级版

目标: 长指令自动拆步、每步查可行性、失败会恢复、有短期记忆。能做"把桌上的泡沫块放进筐"这种多步任务。

在 v0.1 上新增(粗体=新):

新增模块: 任务分解
做什么: 长指令 →
子任务序列(去桌边→对准→伸手→闭夹爪→抬起→放筐)
参考论文的哪部分: SayCan §3「LLM
把指令拆成技能序列」;VoxPoser §3.1「planner 拆
ℓ₁…ℓₙ」;Code as Policies「分层递归」
────────────────────────────────────────
新增模块: 两步决策
做什么: 每个子任务:先选技能,再单独定参数(比一步到位稳)
参考论文的哪部分: BUMBLE
§III-B「子任务预测+技能选择」「参数估计」两步
────────────────────────────────────────
新增模块: 可行性/前置条件检查
做什么: 发指令前校验前置条件(目标在视野?够得到?路径通?),不满足先调整
参考论文的哪部分: IALP「grounding mechanisms +
可行性(可抓/可达)」;ConceptAgent「precondition grounding
 + 形式化验证」;SayCan「affordance 打分」
────────────────────────────────────────
新增模块: 失败恢复闭环
做什么: 读 status
判成败,失败→反思→重试/换目标/放弃(★核心卖点)
参考论文的哪部分: ConceptAgent「失败反馈恢复」;BUMBLE「失败恢复」;RoBridge「闭环判成败(成功/错误/正常)」
────────────────────────────────────────
新增模块: 短期记忆
做什么: 存本次每步(技能/参数/成败)供恢复
参考论文的哪部分: BUMBLE §III-A「短期执行历史记忆」
────────────────────────────────────────
新增模块: 扩充技能库
做什么: approach(obj)、scan_area()、reach_to(xyz)(用
solve_pose+插值)、pick(obj)(approach→reach→grasp→lift
组合)、place(loc)、explore_patrol()
参考论文的哪部分: BUMBLE「粗到细技能库」;WildLMa「技能库设计」;VoxPoser「组合」
────────────────────────────────────────
新增模块: 机械臂通路适配
做什么: reach_to 内部:mock 发 JointTrajectory / Gazebo 发
gui_joint_states
参考论文的哪部分: (代码审查发现的必需适配)

可演示任务: "把桌上的泡沫块 A 放进检查筐"、"绕过三个桩走到起点"。
里程碑: 在 Gazebo 里看到机器人自主完成多步任务,中途抓空了会重试。

## 完全版
目标: 真视觉感知(满足项目视觉要求)、语义理解、长期学习、能拒答、可量化验证。能做"我渴了,找点喝的"这种语义+意图+长程任务。

在 v0.5 上新增:

新增模块: 语义视觉感知
做什么: 给 Gazebo 开末端相机(ee_camera.xacro 现成)→
相机图喂 VLM → 定位"红瓶子";用
Set-of-Mark:检测框标数字让 VLM 选 ID
参考论文的哪部分: BUMBLE「开放世界 RGB-D 感知 +
Set-of-Mark 提示」;VoxPoser §3.2「VLM
视觉落地」;DynaMem「VLM 特征/mLLM 查询定位」
────────────────────────────────────────
新增模块: 几何感知升级
做什么: 深度图→3D 坐标,替换上帝视角桩
参考论文的哪部分: DynaMem「深度反投影建图」
────────────────────────────────────────
新增模块: 长期记忆
做什么: 跨任务存教训,少犯重复错
参考论文的哪部分: BUMBLE §III-A「长期经验记忆」
────────────────────────────────────────
新增模块: 拒答 + 主动探索
做什么: 找不到目标→明确说"没找到"+触发探索,不硬抓
参考论文的哪部分: DynaMem「负结果拒答 + 价值图探索」
────────────────────────────────────────
新增模块: IOR 符号中间层
做什么: 让 LLM
出结构化符号(动作类型+目标+约束向量)而非直接坐标,更稳
参考论文的哪部分: RoBridge「不变可操作表示 IOR」
────────────────────────────────────────
新增模块: 多候选自评择优
做什么: LLM 生成几个方案,自己批判打分选最优
参考论文的哪部分: ConceptAgent「LLM
引导树搜索+自我批判」;IALP「候选 token 概率择优」
────────────────────────────────────────
新增模块: 能力评测
做什么: 按能力维度设计测试集,统计成功率+意图分数+进度分数
参考论文的哪部分: EmbodiedBench「六能力维度+五步流程」;VLABench「意图分数
IS+进度分数 PS」;NativeEmbodied「技能解耦诊断」

可演示任务: "我刚运动完很渴,帮我找点喝的"(需语义识别饮料+意图推断+导航+抓取+失败恢复)。
里程碑: 完整开源项目——语义 Agent + 完整场景 + 多任务 + 评测报告 + 对照 baseline(urban_trash_sorting_demo)。