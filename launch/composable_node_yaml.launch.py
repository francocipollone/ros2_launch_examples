import os
import launch
import launch.actions

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # This variable obtains the path of the yaml file that have the parameter configuration.
    # In order for this to work, the yaml file must be installed into the share folder of 
    # the package, check the setup.py file lines 18-19.
    config = os.path.join(
        get_package_share_directory('ros2_launch_examples'),
        'config',
        'image_tools_parameters.yaml'
        )

    # On this variable we create a Component Container where the components will be loaded.
    container = ComposableNodeContainer(
        name='image_tools_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            #The composable nodes we load to the container have parameters that can be configured
            # in 2 possible ways: 
            #   - Create a dictionary with the parameters names and values. Ex: parameters=[burger_mode: True, height: 200, width: 300]
            #   - Give the path to the yaml file configuration (on this case, the variable we defined before as `config`). Ex:
            ComposableNode(
                package='image_tools',
                plugin='image_tools::Cam2Image',
                name='cam_2_image',
                parameters=[config]),
            ComposableNode(
                package='image_tools',
                plugin='image_tools::ShowImage',
                name='show_image',
                parameters=[config]),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])