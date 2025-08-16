import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue
import time

class NovelSubNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.novel_subscriber = self.create_subscription(String, 'novel', self.novel_callback, 10)
        self.novel_queue = Queue()
        self.timer = self.create_timer(5.0, self.timer_callback)

    def novel_callback(self, msg: String) -> None:
        self.novel_queue.put(msg.data)
        self.get_logger().info(f'Received novel: {msg.data}')

    def timer_callback(self) -> None:
        if not self.novel_queue.empty():
            novel = self.novel_queue.get()
            self.get_logger().info(f'Consuming novel: {novel}')

def main():
    rclpy.init()
    node = NovelSubNode('novel_sub_node')
    rclpy.spin(node)
    rclpy.shutdown()