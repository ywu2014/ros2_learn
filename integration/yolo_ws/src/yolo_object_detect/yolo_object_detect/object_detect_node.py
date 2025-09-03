import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from example_interfaces.msg import String
from cv_bridge import CvBridge
import cv2
import time
import json
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
        self.publisher_ = self.create_publisher(String, '/object_detect_result_topic', 10)

        self.model_path = self.get_parameter('model_path').value
        # 创建检测模型
        self.model = YOLO(self.model_path)

        self.get_logger().info('load model from: ' + self.model_path)

        self.bridge = CvBridge()

    def on_image_callback(self, data):
        # 将 ROS 2 的 Image 消息类型转换为 OpenCV 格式
        # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = self.bridge.imgmsg_to_cv2(data)

        results = self.model(cv_image)

        # 解析结果
        detect_results = []
        for detection in results:
            # 图片中的所有框
            box_results = []

            boxes = detection.boxes

            for i in range(len(boxes)):
                # 图片中的某个框
                box_result = {}

                [x1, y1, x2, y2] = boxes[i].xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # 转换为整数类型
                category = detection.names[int(boxes[i].cls[0].item())]

                box_result['box'] = [x1, y1, x2, y2]
                box_result['category'] = category

                box_results.append(box_result)

                self.get_logger().info(f'分类:{category}, box: {(x1, y1)}, {(x2, y2)}')
                self.get_logger().info('-'*30)
            
            detect_results.append(box_results)

        msg = String()
        msg.data = json.dumps(detect_results)

        self.publisher_.publish(msg)
        self.get_logger().info(f'publish message {msg.data}')

def main():
    rclpy.init()
    node = ObjectDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()