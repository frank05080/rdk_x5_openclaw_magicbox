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
    def breathe_light_effect(self):
        # 呼吸灯效果，逐渐变化亮度
        colors = [
            (0, 255, 0),  # 绿色
        ]
        
        max_brightness = 255  # 最大亮度（255为全亮）
        min_brightness = 0    # 最小亮度（0为全暗）
        step = 5  # 亮度步进（每次增加或减少的亮度值）

        # 逐渐变亮并逐渐变暗，模拟呼吸灯效果
        for color in colors:
            r, g, b = color
            # 亮度从低到高
            for brightness in range(min_brightness, max_brightness, step):
                cmd_list = {i: (int(r * brightness / max_brightness),
                                int(g * brightness / max_brightness),
                                int(b * brightness / max_brightness))
                            for i in range(4)}
                self.peripheral.set_light_static(cmd_list)
                time.sleep(0.02)  # 每次调整后等待时间，可以调节呼吸的快慢

            # 亮度从高到低
            for brightness in range(max_brightness, min_brightness, -step):
                cmd_list = {i: (int(r * brightness / max_brightness),
                                int(g * brightness / max_brightness),
                                int(b * brightness / max_brightness))
                            for i in range(4)}
                self.peripheral.set_light_static(cmd_list)
                time.sleep(0.02)  # 每次调整后等待时间，可以调节呼吸的快慢

    def yellow_flash(self):
        cmd = {i: [255, 255, 0] for i in LED_IDS}
        self.peripheral.set_light_flash(cmd)
        self.wave_motor_slow()  # 单臂慢速扭动

    def red_flash(self):
        cmd = {i: [255, 0, 0] for i in LED_IDS}
        self.peripheral.set_light_flash(cmd)
        self.wave_motor_fast()  # 快速挥舞双臂

    # -------- Servo motor control functions --------
    def wave_motor_slow(self):
        # 单臂慢速扭动
        print("[DEBUG] Waving motor slowly...")
        self.peripheral.set_servo_angle(0, 20)  # Move left leg to 20 degrees
        time.sleep(0.2)
        self.peripheral.set_servo_angle(0, 60)
        time.sleep(0.2)
        self.peripheral.set_servo_angle(0, 100)  # Move left leg to 60 degrees
        time.sleep(0.2)
        self.peripheral.set_servo_angle(0, 120)  # Return left leg to 0 degrees

    def wave_motor_normal(self):
        # 双臂正常速度交替扭动
        print("[DEBUG] Waving motor normally...")
        self.peripheral.set_servo_angle(0, 20)
        self.peripheral.set_servo_angle(1, 20)
        time.sleep(0.2)
        self.peripheral.set_servo_angle(0, 40)
        self.peripheral.set_servo_angle(1, 40)
        time.sleep(0.2)

    # def wave_motor_fast(self):
    #     # 双臂快速挥舞
    #     print("[DEBUG] Waving motor quickly...")
    #     self.peripheral.set_servo_angle(0, 60)
    #     self.peripheral.set_servo_angle(1, 60)
    #     time.sleep(0.05)  # 确保快速挥动
    #     self.peripheral.set_servo_angle(0, 120)
    #     self.peripheral.set_servo_angle(1, 120)
    #     time.sleep(0.05)
    
    def wave_motor_fast(self):
        # 双臂快速挥舞
        print("[DEBUG] Waving motor quickly...")
        self.peripheral.set_servo_angle(0, 60)
        self.peripheral.set_servo_angle(1, 60)
        time.sleep(0.02)  # Reduce sleep time to make the wave faster
        self.peripheral.set_servo_angle(0, 120)
        self.peripheral.set_servo_angle(1, 120)
        time.sleep(0.02)  # Further reduce sleep time for quicker motion
        self.peripheral.set_servo_angle(0, 60)
        self.peripheral.set_servo_angle(1, 60)
        time.sleep(0.02)  # Another short pause to keep it fast
        self.peripheral.set_servo_angle(0, 20)
        self.peripheral.set_servo_angle(1, 20)
        time.sleep(0.02)  # Short pause before starting the next loop


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
        if z < 40:
            inst.red_flash()  # 红色急促频闪 + 双臂快速挥舞
        elif z < 60:
            inst.yellow_flash()  # 黄色闪烁 + 单臂慢速扭动
        else:
            inst.breathe_light_effect()  # 绿色呼吸灯 + 单臂慢速扭动

        time.sleep(0.2)  # prevent spamming the LEDs

    print("[INFO] Depth LED skill finished (120s)")
    return "Depth LED control finished"


# ---------------- Skill Router ----------------
def main(message=None):
    inst = DepthLedSkill()
    return run_depth_led_skill(inst)


# ---------------- CLI Entry ----------------
if __name__ == "__main__":
    result = main()
    print("[RESULT]", result)
