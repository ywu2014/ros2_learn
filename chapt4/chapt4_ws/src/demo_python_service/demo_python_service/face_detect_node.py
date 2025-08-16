import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import face_recognition
import time

class FaceDetectorionNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.bridge = CvBridge() # open和ROS2的Image格式不兼容, 用于将opencv的Image转换为ROS2的Image
        self.services = self.create_service(FaceDetector, '/face_detect', self.face_detector_callback)
        self.default_img_path = get_package_share_directory('demo_python_service') + '/resource/default.jpg'
        self.upsample_times = 1
        self.model = 'hog'

    def face_detector_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_img_path)
        
        start_time = time.time()
        self.get_logger().info('加载完图像, 开始检测')

        face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample=self.upsample_times, model=self.model)
        end_time = time.time()
        self.get_logger().info('检测完成, 耗时: %f' % (end_time - start_time))
        response.number = len(face_locations)
        response.use_time = end_time - start_time
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response
    
def main():
    rclpy.init()
    node = FaceDetectorionNode()
    rclpy.spin(node)
    rclpy.shutdown()