# RDK X5 OpenClaw MagicBox

A MagicBox robot project based on **RDK X5**, demonstrating how to integrate with the popular **OpenClaw** to enable AI-powered intelligent robots.

## Project Introduction

This repository is designed for developers using the D-Robotics **RDK X5** platform and **MagicBox** robot products, providing practical guides and skill examples for integration with **OpenClaw**. By connecting OpenClaw's intelligent agent capabilities with MagicBox's hardware capabilities, you can build smarter and more interactive robot applications.

## About RDK X5 and MagicBox

### RDK X5

RDK X5 is an AI robot developer kit launched by D-Robotics, with core features including:

- **Processor**: Octa-core Cortex A55
- **AI Computing Power**: 10 TOPs BPU
- **GPU Computing Power**: 32 GFlops
- **Memory**: Up to 8GB LPDDR4
- **Chip**: Horizon 5 Intelligent Computing Chip

RDK X5 features high cost-effectiveness, rich interfaces, and a complete toolchain, making it suitable as an edge-side intelligent computing platform for robots.

### MagicBox

MagicBox is a robot product based on RDK X5, supporting rich peripheral skills (LED, camera, voice, TTS, etc.), and can serve as a physical carrier and execution terminal for OpenClaw.

## About OpenClaw

**OpenClaw** (formerly Clawdbot / Moltbot) is an open-source, local-first personal AI assistant with the following features:

- **Persistent Memory**: Remembers user preferences and history across sessions
- **System-Level Operations**: Can invoke terminals, scripts, files, etc.
- **Multi-Platform Integration**: Supports Telegram, WhatsApp, Slack, Discord, etc.
- **Voice and Proactive Capabilities**: Supports voice interaction, proactive checking and reminders
- **Extensible Skill System**: Easy to integrate with robot hardware capabilities

By deploying OpenClaw on RDK X5, or connecting with OpenClaw services running in the cloud or locally, you can achieve an "AI Brain + Robot Body" architecture.

## How to Integrate with OpenClaw

This repository provides references in the following areas:

1. **Skill System Integration**: Encapsulate MagicBox's hardware capabilities (such as LED, photography, voice, TTS) as skills callable by OpenClaw
2. **Edge Deployment**: Deploy OpenClaw or a lightweight agent on RDK X5 to achieve edge-side intelligent decision-making
3. **Cloud Integration**: Control MagicBox to perform actions through OpenClaw's communication channels (such as Telegram, Slack)

## Skills List

| Skill | Description |
|-------|-------------|
| `breathing-led` | LED strip breathing light effect |
| `magicbox-capture` | Image capture/photography |
| `magicbox-dance` | Dance movement control |
| `magicbox-dist-light` | Distance sensing lighting |
| `magicbox-led` | LED lighting control |
| `magicbox-tts` | Text-to-Speech (TTS) |
| `magicbox-voice` | Voice interaction |

For detailed usage of each skill, refer to the `SKILL.md` file in the corresponding subdirectory under the `skills/` directory.

## Quick Start

1. **Environment Setup**: Ensure MagicBox has been flashed with RDK OS image and can boot normally
2. **Skill Deployment**: Copy the required skills from `skills/` to the corresponding directories in MagicBox's `.roo/skills/` directory
3. **OpenClaw Integration**: Based on OpenClaw's interfaces and protocols, encapsulate the above skills as tools or commands callable by OpenClaw

## Related Resources

- [D-Robotics Developer Community](https://developer.d-robotics.cc/)
- [RDK X5 Quick Start](https://developer.d-robotics.cc/rdk_doc/Quick_start/)
- [OpenClaw Official Documentation](https://docs.clawd.bot/)

## Contributing

We welcome Issue submissions and Pull Requests to collectively improve the ecosystem and examples for RDK X5 + OpenClaw + MagicBox.
