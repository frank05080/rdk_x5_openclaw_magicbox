import rclpy
from rclpy.node import Node

# ⚠️ Replace with the real message type of /hobot_face_depth_fusion
from ai_msgs.msg import PerceptionTargets   # example name

TOPIC = "/hobot_face_depth_fusion"

class ZReader(Node):
    def __init__(self):
        super().__init__("z_value_reader")

        self.sub = self.create_subscription(
            PerceptionTargets,
            TOPIC,
            self.callback,
            10
        )

        self.get_logger().info("Subscribed to /hobot_face_depth_fusion")

    def callback(self, msg):
        # msg.targets is a list
        for target in msg.targets:
            for attr in target.attributes:
                if attr.type == "Z(cm)":
                    z_value = attr.value
                    print(f"[Z-DEPTH] Face distance: {z_value} cm")


def main():
    rclpy.init()
    node = ZReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
