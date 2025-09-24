from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('arm1')
urdf_file_path = package_share_directory + '/urdf/robot.urdf'
robot_desc = open(urdf_file_path, 'r').read()

def generate_launch_description():
    # 发布机器人描述(urdf内容)
    # 对应的topic名称为`/robot_description`
    robot_desc_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}]
    )

    # 发布机器人关节状态(GUI形式控制关节转动)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='both',
    )

    # 可视化机器人模型
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', package_share_directory + '/urdf/robot.rviz']
    )

    return LaunchDescription(
        [
            robot_desc_node,
            joint_state_publisher_node,
            rviz_node
        ]
    )
