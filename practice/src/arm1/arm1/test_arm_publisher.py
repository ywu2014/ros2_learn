import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.timer = self.create_timer(6.0, self.publish_trajectory)

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names

        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 1.0, 0.0, 1.0, 0.0]    # 六个关节的位置
        # point1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 六个关节的速度
        point1.time_from_start = Duration(sec=3)    # 轨迹开始时间
        points.append(point1)

        point2 = JointTrajectoryPoint()
        point2.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]    # 六个关节的位置
        point2.time_from_start = Duration(sec=6)    # 轨迹开始时间
        points.append(point2)

        msg.points = points
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing trajectory')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()