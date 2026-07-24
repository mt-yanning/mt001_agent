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

## 开发阶段

- [ ] 阶段一：mock 环境下跑通指令链路
- [ ] 阶段二：实现 LLM 决策大脑
- [ ] 阶段三：接入 Gazebo 仿真验证
- [ ] 阶段四：真机部署（可选）

## 环境变量

复制 `.env.example` 为 `.env` 并填入你的 API Key：

```bash
cp .env.example .env
```
