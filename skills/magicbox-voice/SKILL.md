---
name: magicbox-voice
description: Plays audio files
---

# MagicBox Voice Playback Skill
Plays audio files

## Commands
- `/play <file> <volume> [sync]`
- `python3 /userdata/MagicBox/.roo/skills/magicbox-voice/peripheral_skill.py /play <file> <volume> [sync]`: Play audio file

### Parameters
- `file`: string of audio file path
- `volume`: 0.0-1.0
- `sync`: string "sync". Wait for playback to finish or not (optional)

### Example
`python3 /userdata/MagicBox/.roo/skills/magicbox-voice/peripheral_skill.py /play /userdata/MagicBox/voice/welcome.mp3 0.8 sync`