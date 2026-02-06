import sys
sys.path.append("/userdata/MagicBox/Clawd")
from peripheral import Peripheral_interface

# Initialize peripheral interface

class PeripheralWrapper:
    def __init__(self):
        self.peripheral = Peripheral_interface()

    def set_light_static(self, led_id, color):
        self.peripheral.set_light_static({led_id: color})

    def set_light_flash(self, led_id, color):
        self.peripheral.set_light_flash({led_id: color})


def handle_command(cmd, args, inst):
    if cmd == "static":
        led_id = int(args[0])
        r, g, b = map(int, args[1:4])
        inst.set_light_static(led_id, [r, g, b])
        return f"Set LED{led_id} to static color RGB({r},{g},{b})"
        
    elif cmd == "flash":
        led_id = int(args[0])
        r, g, b = map(int, args[1:4])
        inst.set_light_flash(led_id, [r, g, b])
        return f"Flashed LED{led_id} with RGB({r},{g},{b})"

    return "Invalid LED command"

def main(message):
    inst = PeripheralWrapper()
    parts = message.split()
    if not parts or parts[0] != "/light":
        return "LED command format: /light <static|flash> <led_id> <r> <g> <b>"
    
    return handle_command(parts[1], parts[2:], inst)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        # 将所有参数合并为一个字符串，就像你的测试消息一样
        message = " ".join(sys.argv[1:])
        main(message)
        print("executed command:", message)
    else:
        print("Please provide a test message as command line arguments.")