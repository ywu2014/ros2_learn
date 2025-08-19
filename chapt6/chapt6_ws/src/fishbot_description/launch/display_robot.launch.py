import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_tutorial_package = get_package_share_directory('fishbot_description')
    default_model_path = urdf_tutorial_package + '/urdf/first_robot.urdf'
    default_rviz_config_path = urdf_tutorial_package + '/config/rviz/display_model.rviz'

    # 声明一个参数，用于指定URDF文件的路径
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='URDF的绝对路径'
    )
    
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )

    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # Rviz 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )

    # 启动节点
    launch_description = launch.LaunchDescription([
        action_declare_arg_model_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])

    return launch_description