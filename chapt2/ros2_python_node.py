import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node('python_node')
    # 日志格式修改: export RCUTILS_CONSOLE_OUTPUT_FORMAT=[{function_name}:{line_number}]:{message}
    node.get_logger().info('Hello ROS2')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()