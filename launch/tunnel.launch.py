import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tunnel', node_executable='tunnel_node', output='screen',
            node_name='tunnel_node'),
    ])
