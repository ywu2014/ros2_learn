import math
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.tf_broadcaster = StaticTransformBroadcaster(self)
    
    def publish_static_tf(self, from_frame, to_frame, translation=[0, 0, 0], rotation_euler=[0, 0, 0]):
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
        self.tf_broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    tf_broadcaster = TfBroadcaster()
    tf_broadcaster.publish_static_tf('map', 'base_link', [0.5, 0.3, 0.6], [180, 0, 0])
    rclpy.spin(tf_broadcaster)
    tf_broadcaster.destroy_node()
    rclpy.shutdown()