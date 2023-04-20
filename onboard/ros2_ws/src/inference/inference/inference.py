import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image


class Inference(Node):
    def __init__(self):
        super().__init__("inference")
        self.subscription = self.create_subscription(Image, "camera", self.run_inference, 10)

    def run_inference(self, msg):
        ...
        self.get_logger().info("Running inference on image")


def main(args=None):
    rclpy.init(args=args)

    inference = Inference()

    rclpy.spin(inference)

    inference.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

