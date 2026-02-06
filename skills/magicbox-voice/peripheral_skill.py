import sys
sys.path.append('/userdata/MagicBox/Clawd')
from peripheral import Peripheral_interface


class PeripheralWrapper:
    def __init__(self):
        self.peripheral = Peripheral_interface()

    def play_voice(self, voice_file, volume, asyn):
        print("[DEBUG] play_voice called")
        print("[DEBUG] file:", voice_file)
        print("[DEBUG] volume:", volume)
        print("[DEBUG] async:", asyn)
        self.peripheral.play_voice(voice_file, volume, asyn)


def handle_command(args, inst):
    if len(args) < 2:
        return "Missing parameters. Use: /play <file> <volume> [sync]"
    
    file_path = args[0]
    volume = float(args[1])

    # accept syn OR sync
    mode_args = args[2:]
    asyn = not any(x in ["sync", "syn"] for x in mode_args)

    inst.play_voice(file_path, volume, asyn)
    
    mode = "asynchronously" if asyn else "synchronously"
    return f"Playing {file_path} at {volume*100}% volume {mode}"


def main(message):
    inst = PeripheralWrapper()
    parts = message.split()

    print("[DEBUG] message:", message)
    print("[DEBUG] parts:", parts)

    if not parts or parts[0] != "/play":
        return "Voice command format: /play <file> <volume> [sync]"
    
    return handle_command(parts[1:], inst)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        message = " ".join(sys.argv[1:])
        result = main(message)
        print("[RESULT]", result)
        print("executed command:", message)
    else:
        print("Please provide a test message as command line arguments.")
