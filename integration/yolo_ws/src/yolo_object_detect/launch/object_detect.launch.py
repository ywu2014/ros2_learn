import launch
import launch_ros

def generate_launch_description():
    action_node_object_detect = launch_ros.actions.Node(
        package='yolo_object_detect',
        executable='object_detect',
        output='log',
    )
    action_node_object_detect_client = launch_ros.actions.Node(
        package='yolo_object_detect',
        executable='object_detect_client',
        output='both',
    )

    launch_description = launch.LaunchDescription([
        action_node_object_detect,
        action_node_object_detect_client,
    ])

    return launch_description