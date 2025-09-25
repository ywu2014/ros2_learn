from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('arm1')
urdf_file_path = package_share_directory + '/urdf/robot_ros2_control.urdf'
robot_desc = open(urdf_file_path, 'r').read()

# print(f'robot_desc: {robot_desc}')

def generate_launch_description():
    # 发布机器人描述(urdf内容)
    # 对应的topic名称为`/robot_description`
    robot_desc_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}, {'use_sim_time': True}]
    )

    # URDF中对于joint的type为非fixed的, 这个节点不能少, 少了之后会提示类似`No transform from [XX_Link] to [base_link]`
    joint_state_publisher_node = Node(
        package="joint_state_publisher", 
        executable="joint_state_publisher",
        name='joint_state_publisher',
        output='both',
    )
    
    cm_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[package_share_directory + '/config/robot_ros2_controllers.yaml'],
        output="both",
    )

    cn_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['position_controller', 'joint_state_broadcaster'],   # yaml文件中定义的控制器名字, 要和yaml文件中的对应起来
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', package_share_directory + '/urdf/robot.rviz']
    )

    jc_node = Node(
        package='arm1',
        executable='joint_control'
    )

    return LaunchDescription(
        [
            robot_desc_node,
            joint_state_publisher_node,
            cm_node,
            cn_node,
            rviz_node,
            jc_node
        ]
    )


