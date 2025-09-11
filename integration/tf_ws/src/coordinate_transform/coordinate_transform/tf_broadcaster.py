import math
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import json

class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        # tf_config_json_str格式
        # [
        #     {
        #         "type": "static",
        #         "from_frame": "xx",
        #         "to_frame": "xxx",
        #         "translation": [0, 0, 0],
        #         "rotation_euler": [0, 0, 0]
        #     },
        #     {
        #         "type": "dynamic",
        #         "frequency": 0.01,
        #         "from_frame": "xx",
        #         "to_frame": "xxx",
        #         "translation": [0, 0, 0],
        #         "rotation_euler": [0, 0, 0]
        #     }
        # ]
        self.declare_parameter('tf_config_json_str', 
                               descriptor=ParameterDescriptor(
                                   description='坐标系转换配置, json格式', type=ParameterType.PARAMETER_STRING)
        )

        self.tf_config_json_str = self.get_parameter('tf_config_json_str').value

        # self.tf_broadcaster = StaticTransformBroadcaster(self)
        # self.dynamic_tf_broadcaster = TransformBroadcaster(self)
        # self.timer_ = self.create_timer(0.01, lambda: self.publish_dynamic_tf('camera_link', 'bottle_link', [0.2, 0.0, 0.5], [0, 0, 0]))

    def publish_tf(self):
        tf_config = json.loads(self.tf_config_json_str)
        for tf_map in tf_config:
            tf_type = tf_map['type']
            from_frame = tf_map['from_frame']
            to_frame = tf_map['to_frame']
            translation = tf_map['translation']
            rotation_euler = tf_map['rotation_euler']
            if tf_type == 'static':
                tf_broadcaster = StaticTransformBroadcaster(self)
                self.do_publish_tf(tf_broadcaster, from_frame, to_frame, translation, rotation_euler)
            elif tf_type == 'dynamic':
                frequency = tf_map['frequency']
                tf_broadcaster = TransformBroadcaster(self)
                self.create_timer(frequency, lambda: self.do_publish_tf(tf_broadcaster, from_frame, to_frame, translation, rotation_euler))
            else:
                self.get_logger().error(f'未知的tf类型: {tf_type}')
    
    def do_publish_tf(self, tf_broadcaster, from_frame, to_frame, translation=[0, 0, 0], rotation_euler=[0, 0, 0]):
        """发布坐标转换"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = from_frame
        transform.child_frame_id = to_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        # 欧拉角转四元数
        q = quaternion_from_euler(
            math.radians(rotation_euler[0]), 
            math.radians(rotation_euler[1]), 
            math.radians(rotation_euler[2]))
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        tf_broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    tf_broadcaster = TfBroadcaster()
    # tf_broadcaster.publish_static_tf('base_link', 'camera_link', [0.5, 0.3, 0.6], [180, 0, 0])
    # tf_broadcaster.publish_dynamic_tf('camera_link', 'bottle_link', [0.2, 0.0, 0.5], [0, 0, 0])
    tf_broadcaster.publish_tf()
    rclpy.spin(tf_broadcaster)
    tf_broadcaster.destroy_node()
    rclpy.shutdown()