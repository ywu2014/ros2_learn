import rclpy
from rclpy.node import Node

class PersonNode(Node):
    def __init__(self, node_name: str, name: str, age: int) -> None:
        super().__init__(node_name)
        self.name = name
        self.age = age

    def eat(self, food_name: str) -> None:
        self.get_logger().info(f'{self.name} is eating {food_name}')

def main():
    rclpy.init()
    node = PersonNode('person_node', 'ywu', 18)
    node.eat('rice')
    rclpy.spin(node)
    rclpy.shutdown()