from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    packagepath = get_package_share_directory("robot_arm_config")
    print(f'packagepath: {packagepath}')

    # 加载MoveIt配置
    moveit_config = (
        MoveItConfigsBuilder("arm", package_name="robot_arm_config")
        .robot_description("config/arm.gazebo.urdf.xacro")
        .robot_description_semantic("config/arm.srdf")
        .to_moveit_configs()
    )

    # print('ros_gz_sim path:')
    # print(get_package_share_directory("ros_gz_sim") + "/launch/gz_sim.launch.py")

    # 启动Gazebo
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("ros_gz_sim") + "/launch/gz_sim.launch.py"]),
            # launch_arguments=[("gz_args", "empty.sdf -r --physics-engine gz-physics-bullet-featherstone-plugin")]
            launch_arguments=[("gz_args", "empty.sdf -r")]
    )

    # 将机械臂添加到Gazebo
    robot_to_gazebo_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=['-string', moveit_config.robot_description['robot_description'], '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0'],
    )

    # Clock Bridge
    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # 发布机械臂状态
    robot_desc_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, 
                    {"use_sim_time": True},        # 确保使用仿真时间
                    {"publish_frequency": 30.0}],
    )

    # launch rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d",packagepath + "config/moveit.rviz"],
    )

    # ros2_controller manager节点
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         # moveit_config.robot_description,
    #         packagepath + "/config/ros2_controllers.yaml",
    #         {"use_sim_time": True}
    #     ],
    #     output="both",
    # )

    # 启动关节状态发布器, arm组控制器, 夹爪控制器
    controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "arm_controller", "hand_controller"],
        output="both",
    )

    # 启动MoveGroup节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True}],
    )

    return LaunchDescription([
        gazebo_node,
        robot_to_gazebo_node,
        clock_bridge_node,
        robot_desc_node,
        rviz_node,
        # ros2_control_node,
        controller_spawner_node,
        move_group_node,
    ])