import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import cv2

from hbm_img_msgs.msg import HbmMsg1080P

TOPIC_NAME = "/hbmem_img"


# ---------------- Wrapper ----------------
class HbmemImageWrapper:
    def __init__(self):
        self.node = None
        self.image = None
        self.received = False

    def init_ros(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("hbmem_image_skill_node")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,   # 🔥 FIX
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.node.create_subscription(
            HbmMsg1080P,
            TOPIC_NAME,
            self._callback,
            qos
        )

        print(f"[DEBUG] Subscribed to {TOPIC_NAME} with BEST_EFFORT QoS")

    def _callback(self, msg):
        try:
            height = msg.height
            width = msg.width
            encoding = bytes(msg.encoding).decode(errors="ignore").lower()
            raw = msg.data

            print(f"[DEBUG] Frame received {width}x{height} encoding={encoding}")

            if "nv12" in encoding:
                frame = self.decode_nv12(raw, width, height)
            else:
                print("[ERROR] Unsupported encoding:", encoding)
                return

            self.image = frame
            self.received = True

        except Exception as e:
            print("[ERROR] Image decode failed:", e)

    def decode_nv12(self, raw, width, height):
        y_size = width * height
        uv_size = y_size // 2

        raw_np = np.frombuffer(raw, dtype=np.uint8)

        y = raw_np[0:y_size].reshape((height, width))
        uv = raw_np[y_size:y_size + uv_size].reshape((height // 2, width))

        nv12 = np.vstack((y, uv))
        bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
        return bgr

    def capture_once(self, save_path):
        print("[DEBUG] capture_once called")

        self.init_ros()
        self.received = False

        start = time.time()
        timeout = 10.0  # seconds

        while not self.received:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start > timeout:
                print("[ERROR] Timeout waiting for image")
                return False

        cv2.imwrite(save_path, self.image)
        print(f"[DEBUG] Image saved to {save_path}")
        return True


# ---------------- Handlers ----------------
def handle_capture(args, inst):
    if len(args) < 1:
        return "Missing parameters. Use: /capture <save_path>"

    save_path = args[0]

    ok = inst.capture_once(save_path)
    if ok:
        return f"Image captured and saved to {save_path} 📸"
    else:
        return "Failed to capture image ❌"


# ---------------- Main Router ----------------
def main(message):
    inst = HbmemImageWrapper()
    parts = message.split()

    print("[DEBUG] message:", message)
    print("[DEBUG] parts:", parts)

    if not parts:
        return "Commands: /capture"

    cmd = parts[0].lower()
    args = parts[1:]

    if cmd == "/capture":
        return handle_capture(args, inst)
    else:
        return "Unknown command. Use: /capture <save_path>"


# ---------------- CLI Entry ----------------
if __name__ == "__main__":
    if len(sys.argv) > 1:
        message = " ".join(sys.argv[1:])
        result = main(message)
        print("[RESULT]", result)
        print("executed command:", message)
    else:
        print("Please provide a test message as command line arguments.")
