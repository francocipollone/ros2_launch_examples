import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    return launch.LaunchDescription([
        # Nodes Definition
        launch_ros.actions.Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        launch_ros.actions.Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        launch_ros.actions.Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        ),
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        
        # Components Definition
        launch_ros.actions.ComposableNodeContainer(
        name='talker_listener_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='demo_nodes_cpp',
                plugin='demo_nodes_cpp::Talker',
                name=[launch.substitutions.LaunchConfiguration('node_prefix'),'talker']),
            launch_ros.descriptions.ComposableNode(
                package='demo_nodes_cpp',
                plugin='demo_nodes_cpp::Listener',
                name=[launch.substitutions.LaunchConfiguration('node_prefix'),'listener']),
        ],
        output='screen'),

        # Process Definition
        launch.actions.ExecuteProcess(
        cmd=[[
            'ros2 bag record ',
            '-a',
        ]],
        shell=True
    )
    ])  