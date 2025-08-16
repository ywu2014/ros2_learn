import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge

class FaceDetectorClient(Node):
    def __init__(self):
        super().__init__('face_detect_client')
        self.client = self.create_client(FaceDetector, '/face_detect')
        self.bridge = CvBridge()
        self.test_image_path = get_package_share_directory('demo_python_service') + '/resource/test.jpg'
        self.image = cv2.imread(self.test_image_path)

    def send_request(self):
        # 判断服务是否上线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('service not available, waiting again...')
        # 构造请求
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        # 发送并spin等待服务处理完成
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        # 处理响应
        response = future.result()
        self.get_logger().info(f'接收到响应:图像中共有{response.number}个人脸，耗时{response.use_time}ms')
        self.show_face_location(response)

    def show_face_location(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (255, 0, 0), 2)

        cv2.imshow('Face Detect', self.image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
def main():
    rclpy.init()
    face_detector_client = FaceDetectorClient()
    face_detector_client.send_request()
    # rclpy.spin(face_detector_client)
    rclpy.shutdown()