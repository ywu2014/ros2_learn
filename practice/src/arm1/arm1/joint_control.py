import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('multiarray_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'position_controller/commands', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [self.counter/10.0]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MultiArrayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()