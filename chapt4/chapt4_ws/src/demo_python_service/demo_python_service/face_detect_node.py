import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2
import face_recognition
import time

class FaceDetectorionNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')

        self.declare_parameter('face_locations_upsample_times', 1)
        self.declare_parameter('face_locations_model', 'hog')
        self.upsample_times = self.get_parameter('face_locations_upsample_times').value
        self.model = self.get_parameter('face_locations_model').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.bridge = CvBridge() # open和ROS2的Image格式不兼容, 用于将opencv的Image转换为ROS2的Image
        self.service = self.create_service(FaceDetector, '/face_detect', self.face_detector_callback)
        self.default_img_path = get_package_share_directory('demo_python_service') + '/resource/default.jpg'

        # 通过代码改变自身的参数
        # self.set_parameters([rclpy.Parameter(name='face_locations_upsample_times', type=rclpy.Parameter.Type.INTEGER, value=2)])

    def parameter_callback(self, parameters):
        for parameter in parameters:
            self.get_logger().info('参数名: %s, 参数值: %s' % (parameter.name, parameter.value))
            if parameter.name == 'face_locations_upsample_times':
                self.upsample_times = parameter.value
            if parameter.name == 'face_locations_model':
                self.model = parameter.value
        return SetParametersResult(successful=True)

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