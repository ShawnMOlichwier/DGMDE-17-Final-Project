import rclpy
import onnxruntime as ort
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2


class Inference(Node):
    def __init__(self):
        super().__init__("inference")
        
        self.publisher_ = self.create_publisher(Image, "inference", 10)
        self.subscription = self.create_subscription(Image, "camera", self.callback, 10)

        self.model = ort.InferenceSession("/home/charleslariviere/best_model.onnx")
        self.get_logger().info("model loaded")
        
        self.bridge = CvBridge()

        self.colors = np.array([
            [255, 0, 0],
            [0, 255, 0],
            [0, 0, 255],
            [255, 255, 0]
        ])

    def callback(self, msg):
        img = self.prepare_image(msg)
        prediction = self.run_inference(img)
        color_mask = self.get_color_mask(prediction)
        self.publish(color_mask)
        self.get_logger().info("Publishing inference")
    
    def publish(self, prediction):
        """Publish the multi-class semantic mask to the ROS topic."""
        msg = self.bridge.cv2_to_imgmsg(prediction, "bgr8")
        self.publisher_.publish(msg)

    def get_color_mask(self, prediction):
        """Convert the model prediction to a color-coded mask."""
        predicted_class = prediction.argmax(axis=0)
        colored_mask = self.colors[predicted_class.flatten()].reshape((224, 224, -1))
        return colored_mask.astype("uint8")

    def run_inference(self, img):
        """Get the model prediction for an image."""
        prediction = self.model.run(None, {"x.1": img})
        return np.array(prediction)[0, 0]

    def prepare_image(self, msg):
        """Convert the ROS Image message to a batched Numpy array."""
        img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.resize(img, (224, 224))
        img = img.astype(np.float32)
        img = img[np.newaxis, np.newaxis, ...]
        return img


def main(args=None):
    rclpy.init(args=args)

    inference = Inference()

    rclpy.spin(inference)

    inference.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

