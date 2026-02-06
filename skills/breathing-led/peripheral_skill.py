import sys
import time
sys.path.append('/userdata/MagicBox/Clawd')
from peripheral import Peripheral_interface

class PeripheralWrapper:
    def __init__(self):
        self.peripheral = Peripheral_interface()

    def set_light_static(self, led_id, color):
        self.peripheral.set_light_static({led_id: color})

    def set_light_flash(self, led_id, color):
        self.peripheral.set_light_flash({led_id: color})

def breathing_sequence(inst):
    """
    Cycle through LEDs with different colors for 1 cycle.
    Single-threaded.
    """
    if not inst:
        print("Peripheral interface not available")
        return

    sequence = [
        (0, [255, 0, 0]),   # LED0: Red
        (1, [0, 0, 255]),   # LED1: Blue
        (2, [0, 255, 0]),   # LED2: Green
        (3, [255, 255, 0])  # LED3: Yellow
    ]

    for led_id, color in sequence:
        # Fade in
        for brightness in range(0, 256, 5):
            r = color[0] * brightness // 255
            g = color[1] * brightness // 255
            b = color[2] * brightness // 255
            inst.set_light_static(led_id, [r, g, b])
            time.sleep(0.01)

        # Fade out
        for brightness in range(255, -1, -5):
            r = color[0] * brightness // 255
            g = color[1] * brightness // 255
            b = color[2] * brightness // 255
            inst.set_light_static(led_id, [r, g, b])
            time.sleep(0.01)

    # Turn off all LEDs at the end
    for i in range(4):
        inst.set_light_static(i, [0, 0, 0])

def main(message):
    """
    Handle /breathing command.
    Format: /breathing start
    """
    inst = PeripheralWrapper()
    parts = message.split()
    if not parts or parts[0] != "/breathing":
        return "Breathing command format: /breathing start"

    if len(parts) < 2 or parts[1].lower() != "start":
        return "Only /breathing start is supported"

    breathing_sequence(inst)
    return "Breathing sequence completed"

if __name__ == "__main__":
    if len(sys.argv) > 1:
        # 将所有参数合并为一个字符串，就像你的测试消息一样
        message = " ".join(sys.argv[1:])
        main(message)
        print("executed command:", message)
    else:
        print("Please provide a test message as command line arguments.")
