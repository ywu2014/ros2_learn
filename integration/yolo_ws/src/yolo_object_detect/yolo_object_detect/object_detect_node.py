import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2
import time
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

class ObjectDetectNode(Node):
    def __init__(self):
        super().__init__('object_detect_node')

        self.default_model_path = get_package_share_directory('yolo_object_detect') + '/resource/best.pt'
        self.declare_parameter('model_path', self.default_model_path)

        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            '/image_topic',
            self.on_image_callback,
            10
        )

        # 创建发布者
        self.publisher_ = self.create_publisher(Image, '/object_detect_result_topic', 10)

        self.model_path = self.get_parameter('model_path').value
        # 创建检测模型
        self.model = YOLO(self.model_path)

        self.bridge = CvBridge()

    def on_image_callback(self, data):
        # 将 ROS 2 的 Image 消息类型转换为 OpenCV 格式
        # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = self.bridge.imgmsg_to_cv2(data)

        results = self.model(cv_image)
        for detection in results:
            boxes = detection.boxes

            for i in range(len(boxes)):
                [x1, y1, x2, y2] = boxes[i].xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # 转换为整数类型
                name = detection.names[i]

                self.get_logger().info(f'分类:{name}, box: {(x1, y1)}, {(x2, y2)}')
                self.get_logger().info('-'*30)

def main():
    rclpy.init()
    node = ObjectDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()