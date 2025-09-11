import launch
import launch_ros

def generate_launch_description():
    action_node_tf_broadcaster = launch_ros.actions.Node(
        package='coordinate_transform',
        executable='tf_broadcaster',
        parameters=[
            {'tf_config_json_str': '[{"type": "static", "from_frame": "base_link", "to_frame": "camera_link", "translation": [0.5, 0.3, 0.6], "rotation_euler": [180, 0, 0]}, {"type": "dynamic", "frequency": 0.01, "from_frame": "camera_link", "to_frame": "bottle_link", "translation": [0.2, 0.0, 0.5], "rotation_euler": [0, 0, 0]}]'}
        ],
        output='log',
    )

    launch_description = launch.LaunchDescription([
        action_node_tf_broadcaster,
    ])

    return launch_description