import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
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
        # 防止显示堵塞无法多次请求
        # self.show_face_location(response)

    def call_set_parameters(self, parameters):
        client = self.create_client(SetParameters, '/face_detect_node/set_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SetParameters.Request()
        request.parameters = parameters
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'设置参数结果: {response}')
        return response
    
    def update_detect_model(self, model):
        parameter = Parameter()
        parameter.name = 'face_locations_model'
        new_model_value = ParameterValue()
        new_model_value.type = ParameterType.PARAMETER_STRING
        new_model_value.string_value = model
        parameter.value = new_model_value
        response = self.call_set_parameters([parameter])
        for result in response.results:
            if result.successful:
                self.get_logger().info(f'参数{parameter.name}设置成功')
            else:
                self.get_logger().info(f'参数{parameter.name}设置失败, 失败原因: {result.reason}')

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
def main(args=None):
    rclpy.init(args=args)
    face_detector_client = FaceDetectorClient()
    face_detector_client.update_detect_model('hog')
    face_detector_client.send_request()

    face_detector_client.update_detect_model('cnn')
    face_detector_client.send_request()
    # face_detector_client.send_request()
    rclpy.spin(face_detector_client)
    rclpy.shutdown()