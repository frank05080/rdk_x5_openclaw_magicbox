# RDK X5 OpenClaw MagicBox

基于 **RDK X5** 的 MagicBox 机器人项目，介绍如何与当下炙手可热的 **OpenClaw** 进行结合，实现智能机器人的 AI 赋能。

## 项目简介

本仓库面向使用地瓜机器人（D-Robotics）**RDK X5** 平台与 **MagicBox** 机器人产品的开发者，提供与 **OpenClaw** 结合的实践指南与技能示例。通过将 OpenClaw 的智能代理能力与 MagicBox 的硬件能力打通，可以构建更智能、更具交互性的机器人应用。

## 关于 RDK X5 与 MagicBox

### RDK X5

RDK X5 是地瓜机器人推出的 AI 机器人开发者套件，核心特性包括：

- **处理器**：八核 Cortex A55
- **AI 算力**：10 TOPs BPU
- **GPU 算力**：32 GFlops
- **内存**：最高 8GB LPDDR4
- **芯片**：旭日 5 智能计算芯片

RDK X5 具有高性价比、丰富接口、完整工具链等特点，适合作为机器人端侧智能计算平台。

### MagicBox

MagicBox 是基于 RDK X5 的机器人产品，支持丰富的周边技能（LED、摄像头、语音、TTS 等），可作为 OpenClaw 的物理载体与执行终端。

## 关于 OpenClaw

**OpenClaw**（原名 Clawdbot / Moltbot）是一款开源、本地优先的个人 AI 助手，具备：

- **持久化记忆**：跨会话记住用户偏好与历史
- **系统级操作**：可调用终端、脚本、文件等
- **多平台集成**：支持 Telegram、WhatsApp、Slack、Discord 等
- **语音与主动能力**：支持语音交互、主动检查与提醒
- **可扩展技能系统**：便于与机器人硬件能力对接

将 OpenClaw 部署在 RDK X5 上，或与运行在云端/本地的 OpenClaw 服务联动，可实现“AI 大脑 + 机器人身体”的架构。

## 如何与 OpenClaw 结合

本仓库提供以下方向的参考：

1. **技能系统对接**：将 MagicBox 的硬件能力（如 LED、拍摄、语音、TTS）封装为 OpenClaw 可调用的技能
2. **端侧部署**：在 RDK X5 上部署 OpenClaw 或轻量代理，实现端侧智能决策
3. **云端联动**：通过 OpenClaw 的通讯渠道（如 Telegram、Slack）控制 MagicBox 执行动作

## 技能列表 (Skills)

| 技能 | 说明 |
|------|------|
| `breathing-led` | LED 灯带呼吸灯效果 |
| `magicbox-capture` | 图像采集/拍照 |
| `magicbox-dance` | 舞蹈动作控制 |
| `magicbox-dist-light` | 距离感应灯光 |
| `magicbox-led` | LED 灯光控制 |
| `magicbox-tts` | 文字转语音 (TTS) |
| `magicbox-voice` | 语音交互 |

各技能的详细用法参见 `skills/` 目录下对应子目录中的 `SKILL.md`。

## 快速开始

1. **环境准备**：确保 MagicBox 已烧录 RDK OS 镜像并可正常启动
2. **技能部署**：将 `skills/` 下所需技能拷贝到 MagicBox 的 `.roo/skills/` 对应目录
3. **与 OpenClaw 集成**：根据 OpenClaw 的接口与协议，将上述技能封装为 OpenClaw 可调用的工具或命令

## 相关资源

- [地瓜机器人开发者社区](https://developer.d-robotics.cc/)
- [RDK X5 快速开始](https://developer.d-robotics.cc/rdk_doc/Quick_start/)
- [OpenClaw 官方文档](https://docs.clawd.bot/)

## 贡献

欢迎提交 Issue 与 Pull Request，共同完善 RDK X5 + OpenClaw + MagicBox 的生态与示例。
