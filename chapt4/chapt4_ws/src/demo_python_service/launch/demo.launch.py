import launch
import launch_ros

def generate_launch_description():
    action_node_face_detect = launch_ros.actions.Node(
        package='demo_python_service',
        executable='face_detect_node',
        output='screen',
    )
    action_node_face_detect_client = launch_ros.actions.Node(
        package='demo_python_service',
        executable='face_detect_client_node',
        output='both',
    )

    launch_description = launch.LaunchDescription([
        action_node_face_detect,
        action_node_face_detect_client,
    ])

    return launch_description