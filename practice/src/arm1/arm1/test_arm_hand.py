import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class GripperTestNode(Node):
    def __init__(self):
        super().__init__('gripper_test_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/hand_controller/commands', 10)

        self.get_logger().info('Gripper test node started')
        self.timer = self.create_timer(1.0, self.move)
        self.value = 0.05

    def move(self):
        msg = Float64MultiArray()
        if self.value == 0:
            self.value = 0.05
        else:
            self.value = 0

        msg.data = [self.value]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()