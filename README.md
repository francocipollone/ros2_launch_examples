# ros2_launch_examples

## Introduction

This repository aims to gather information and provide examples about ROS2 launch files in python.

## Official Documentation

- [Architecture of launch](https://github.com/ros2/launch/blob/foxy/launch/doc/source/architecture.rst)


## Official Tutorials

 - [ROS2 Launch Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
 - [Launching composable nodes](https://docs.ros.org/en/humble/How-To-Guides/Launching-composable-nodes.html)


## Some theory
### Using substitutions

A substitution is something that cannot, or should not, be evaluated until it's time to execute the launch description that they are used in. There are many possible variations of a substitution, but here are some of the core ones implemented by launch (all of which inherit from class [`launch.Substitution`](https://github.com/ros2/launch/blob/master/launch/launch/substitution.py)):

The already provided substitutions can be found at [ros2/launch/substitutions](https://github.com/ros2/launch/tree/master/launch/launch/substitutions)

#### Including Launch description

```py

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'launch',
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```


- `launch_ros.substitutions.FindPackageShare`: `FindPackageShare` substitution is used to find the path to a package.

- `launch.substitutions.PathJoinSubstitution`: `PathJoinSubstitution` substitution is then used to join the path of a package path with the launch file name.

- `launch.substitutions.TextSubstitution`: `TextSubstitution` substitution is used to define the new_background_r argument with the value of the background_r key in the colors dictionary.


##### Child launch file

```py
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```

- `launch.substitutions.LaunchConfiguration` : `LaunchConfiguration` substitutions allow us to acquire the value of the launch argument in any part of the launch description.

- `launch.actions.DeclareLaunchArgument`: `DeclareLaunchArgument` is used to define the launch argument that can be passed from the above launch file or from the console.

- `launch.substitutions.PythonExpression`: This substitution will evaluate a python expression and get the result as a string.

- `launch.actions.ExecuteProcess`: Executes a process with the corresponding `cmd` argument.


## Examples

- Launch file for executing rosbag process: See [ros2_launch_examples](launch/README.md) package.