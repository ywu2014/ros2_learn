import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge

class ObjectDetectClient(Node):
    def __init__(self):
        super().__init__('object_detect_client')
        self.publisher_ = self.create_publisher(Image, '/image_topic', 10)
        self.bridge = CvBridge()
        self.test_image_path = get_package_share_directory('yolo_object_detect') + '/resource/IMG_5153.JPG'
        self.get_logger().info(f'test image path: {self.test_image_path}')
        self.image = cv2.imread(self.test_image_path)

    def publish_image(self):
        image_msg = self.bridge.cv2_to_imgmsg(self.image)
        
        # 发布图片消息
        self.publisher_.publish(image_msg)
        self.get_logger().info('Publishing image')

def main(args=None):
    rclpy.init(args=args)
    object_detect_client = ObjectDetectClient()
    object_detect_client.publish_image()
    # face_detector_client.send_request()
    rclpy.spin(object_detect_client)
    rclpy.shutdown()