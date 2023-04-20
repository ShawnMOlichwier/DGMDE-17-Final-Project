import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
    
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        topic_name= 'camera'

        self.publisher_ = self.create_publisher(Image, topic_name , 10)
        self.timer = self.create_timer(0.1, self.publish_image)

        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

    def publish_image(self):
        ret, frame = self.cap.read()     
        if ret == True:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
        self.get_logger().info('Publishing camera image')


def main(args=None):
    rclpy.init(args=args)
    
    camera_publisher = CameraPublisher()
    
    rclpy.spin(camera_publisher)
    
    camera_publisher.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()
