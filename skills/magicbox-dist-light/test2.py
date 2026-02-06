import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.append('/userdata/MagicBox/Clawd')
from peripheral import Peripheral_interface

# ⚠️ Replace with the real message type
from ai_msgs.msg import PerceptionTargets

TOPIC = "/hobot_face_depth_fusion"
RUN_TIME = 120.0
LED_IDS = [0, 1, 2, 3]  # all LEDs on the WS2812B strip


# ---------------- Wrapper ----------------
class DepthLedSkill:
    def __init__(self):
        self.node = None
        self.peripheral = Peripheral_interface()
        self.last_z = None

    def init_ros(self):
        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node("depth_led_skill_node")

        # QoS compatible with hobot topics
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.node.create_subscription(
            PerceptionTargets,
            TOPIC,
            self._callback,
            qos
        )

        print(f"[DEBUG] Subscribed to {TOPIC}")

    def _callback(self, msg):
        # get Z(cm) from first target
        for target in msg.targets:
            for attr in target.attributes:
                if attr.type == "Z(cm)":
                    self.last_z = float(attr.value)
                    print(f"[DEBUG] Z distance: {self.last_z} cm")

    # -------- LED control functions --------
    def red_flash(self):
        cmd = {i: [255, 0, 0] for i in LED_IDS}
        self.peripheral.set_light_flash(cmd)
        self.wave_motor()  # Trigger servo wave when red flashes

    def yellow_static(self):
        cmd = {i: [255, 255, 0] for i in LED_IDS}
        self.peripheral.set_light_static(cmd)

    def blue_static(self):
        cmd = {i: [0, 0, 255] for i in LED_IDS}
        self.peripheral.set_light_static(cmd)

    # -------- Servo motor control functions --------
    def wave_motor(self):
        # Assuming you have the servo control in the peripheral, this will wave the leg
        # You can control the servo angle by moving it to different positions
        print("[DEBUG] Waving motor...")
        self.peripheral.set_servo_angle(0, 20)  # Move left leg to 45 degrees
        time.sleep(0.05)
        self.peripheral.set_servo_angle(0, 40)   # Move left leg back to 0 degrees
        time.sleep(0.05)
        self.peripheral.set_servo_angle(0, 60) # Move left leg to -45 degrees
        time.sleep(0.05)
        self.peripheral.set_servo_angle(0, 80)   # Return left leg to 0 degrees


# ---------------- Core Logic ----------------
def run_depth_led_skill(inst: DepthLedSkill):
    inst.init_ros()
    start = time.time()

    while time.time() - start < RUN_TIME:
        rclpy.spin_once(inst.node, timeout_sec=0.1)

        if inst.last_z is None:
            continue

        z = inst.last_z

        # ---- LED logic ----
        if z < 30:
            inst.red_flash()  # This will also trigger motor waving
        elif z < 50:
            inst.yellow_static()
        else:
            inst.blue_static()

        time.sleep(0.2)  # prevent spamming the LEDs

    print("[INFO] Depth LED skill finished (20s)")
    return "Depth LED control finished"


# ---------------- Skill Router ----------------
def main(message=None):
    inst = DepthLedSkill()
    return run_depth_led_skill(inst)


# ---------------- CLI Entry ----------------
if __name__ == "__main__":
    result = main()
    print("[RESULT]", result)
