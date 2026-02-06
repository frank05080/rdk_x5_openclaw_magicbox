import sys
sys.path.append('/userdata/MagicBox/Clawd')
from peripheral import Peripheral_interface

peripheral = Peripheral_interface()
peripheral.play_voice("/userdata/MagicBox/voice/robot.mp3", 1, True)
