from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    #Creates a LaunchDescription with the elements to be launched as a list.
    launch_description = LaunchDescription([
        # Using the following classes, the parent launch file will look for the 
        # 'substitution_argument_parameter.launch.py' file and launch it with it's
        # corresponding arguments.
        IncludeLaunchDescription(
            # Using the following classes, will look inside the installed shared folder from
            # 'ros2_launch_examples' package for the 'substitution_argument_parameter.launch.py'
            # file and launch it.
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros2_launch_examples'),
                    'launch',
                    'substitution_argument_parameter.launch.py'
                ])
            ]),
            #`launch_arguments` is a dictionary of launch arguments to be passed on to the 
            # included launch file. The dictionary is passed as a view object.
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        ),
        IncludeLaunchDescription(
            # Using the following classes, will look inside the 'ros2_launch_examples' package 
            # for the 'substitution_argument_parameter.launch.py' file and launch it.
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros2_launch_examples'),
                    'launch',
                    'node_component_process.launch.py'
                ])
            ]))
    ])

    # Return a LaunchDescription. This class takes as arguments a list of elements,
    # these elements are launch items such as arguments, actions, nodes, components, etc.
    return launch_description
        
