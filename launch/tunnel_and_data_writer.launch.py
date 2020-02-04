import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        launch_ros.actions.Node(
            package='tunnel', node_executable='tunnel_node', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'tunnel_node']),
        launch_ros.actions.Node(
            package='tunnel', node_executable='tunnel_data_writer_node', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'tunnel_data_writer_node']),
    ])
