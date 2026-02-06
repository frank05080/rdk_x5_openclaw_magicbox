import sys
import time
sys.path.append('/userdata/MagicBox/Clawd')
from peripheral import Peripheral_interface

class PeripheralWrapper:
    def __init__(self):
        self.peripheral = Peripheral_interface()

    # ---- LED ----
    def set_light_static(self, led_id, color):
        self.peripheral.set_light_static({led_id: color})

    def set_light_flash(self, led_id, color):
        self.peripheral.set_light_flash({led_id: color})

    # ---- Servo ----
    def set_servo_angle(self, servo_id, angle):
        self.peripheral.set_servo_angle(servo_id, angle)

    def set_servos_angle(self, control_dict):
        self.peripheral.set_servos_angle(control_dict)


def wave_hand(inst, step):
    """
    Mirror waving motion (more natural)
    """
    wave_pattern = [
        {0: 20, 1: 160},
        {0: 80, 1: 80},
        {0: 160, 1: 20},
        {0: 80, 1: 80},
        {0: 20, 1: 160},
    ]

    pose = wave_pattern[step % len(wave_pattern)]
    inst.set_servos_angle(pose)
    

def dancing_sequence(inst):
    """
    LED breathing + servo waving (synchronized)
    Single-threaded
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

    wave_step = 0

    for led_id, color in sequence:

        # -------- Fade in --------
        for brightness in range(0, 256, 8):
            r = color[0] * brightness // 255
            g = color[1] * brightness // 255
            b = color[2] * brightness // 255

            inst.set_light_static(led_id, [r, g, b])

            # wave hand
            wave_hand(inst, wave_step)
            wave_step += 1

            time.sleep(0.02)

        # -------- Fade out --------
        for brightness in range(255, -1, -8):
            r = color[0] * brightness // 255
            g = color[1] * brightness // 255
            b = color[2] * brightness // 255

            inst.set_light_static(led_id, [r, g, b])

            # wave hand
            wave_hand(inst, wave_step)
            wave_step += 1

            time.sleep(0.02)

    # ---- Reset ----
    for i in range(4):
        inst.set_light_static(i, [0, 0, 0])

    # reset servos to neutral
    inst.set_servos_angle({0: 0, 1: 0})


def main(message):
    """
    Handle /dancing command.
    Format: /dancing start
    """
    inst = PeripheralWrapper()
    parts = message.split()

    if not parts or parts[0] != "/dancing":
        return "Dancing command format: /dancing start"

    if len(parts) < 2 or parts[1].lower() != "start":
        return "Only /dancing start is supported"

    dancing_sequence(inst)
    return "dancing + waving sequence completed"


if __name__ == "__main__":
    if len(sys.argv) > 1:
        message = " ".join(sys.argv[1:])
        main(message)
        print("executed command:", message)
    else:
        print("Please provide a test message as command line arguments.")
