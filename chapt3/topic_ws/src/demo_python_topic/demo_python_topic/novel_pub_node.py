import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue

class NovelPubNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.novel_publisher = self.create_publisher(String, 'novel', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.novel_queue = Queue()

    def download_novel(self, url: str) -> None:
        response = requests.get(url)
        response.encoding = 'utf-8'
        self.get_logger().info(f'Downloading novel from {url}')
        for line in response.text.splitlines():
            self.novel_queue.put(line)

    def timer_callback(self) -> None:
        if not self.novel_queue.empty():
            novel = self.novel_queue.get()
            msg = String()
            msg.data = novel
            self.novel_publisher.publish(msg)
            self.get_logger().info(f'Publishing novel: {novel}')

def main():
    rclpy.init()
    node = NovelPubNode('novel_pub_node')
    node.download_novel('https://www.gutenberg.org/cache/epub/1342/pg1342.txt')
    rclpy.spin(node)
    rclpy.shutdown()