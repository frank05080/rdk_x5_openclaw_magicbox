---
name: magicbox-led
description: Controls WS2812B LED strip
---

# MagicBox LED Control Skill
Controls WS2812B LED strip

## Commands
- `python3 /userdata/MagicBox/.roo/skills/magicbox-led/peripheral_skill.py /light static <led_id> <r> <g> <b>`: Set static color
- `python3 /userdata/MagicBox/.roo/skills/magicbox-led/peripheral_skill.py /light flash <led_id> <r> <g> <b>`: Flash once

### Parameters
- `led_id`: 0-3
- `r`,`g`,`b`: 0-255

### Example
`python3 /userdata/MagicBox/.roo/skills/magicbox-led/peripheral_skill.py /light flash 1 255 0 0` - Flash LED1 red