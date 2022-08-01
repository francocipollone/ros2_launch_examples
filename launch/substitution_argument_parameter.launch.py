from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

#This launch example 
def generate_launch_description():

    # This variables are defined as launch configuration elements.
    # This only defines that exist a configuration element with the name:
    # 'turtlesim_ns', 'use_provided_red', 'new_background_r'.
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    # This variables defines the actual value that the launch configuration 
    # elements will take. Using this is possible to pass arguments to the 
    # launch file in order to substitute the value of an specific variable.
    # On this case: 'turtlesim_ns', 'use_provided_red', 'new_background_r'.
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

    # This variable defines a Node that is going to be run on the launch file.
    # On ros2 each node is able to declare their own parameters, that way, a
    # specific parameter is defined for an specific node. On the launch file is
    # possible to run a Node or Component giving it parameter values as seen on
    # this definition.
    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim',
        parameters=[{'background_r': 150,
                     'background_g': 20,
                     'background_b': 20}]
    )

    # This variables defines some Process that is going to be run on the launch file.
    # Think of a process as a console command you would want to do in a specific moment
    # of the launch file.
    # On this case, that console command is calling a ros2 service to create a turtle on a specific
    # location.
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

    # On this case, the console command is modifying a ros2 parameter regarding the red color value
    # from the turtlesim background to '120'.
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    # On this case, the console command is modifying a ros2 parameter regarding the red color value
    # from the turtlesim background using some arguments as conditions for this to happen. Depending
    # on the argument passed to new_background_r and use_provided_red, the background may change.
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

    # Return a LaunchDescription. This class takes as arguments a list of elements.
    # On this case, all the elements were defined previously and stored in variables and then
    # those variables become the list elements, this is another way to do it instead of just plugging
    # all the definitions inside the LaunchDescription on the return statement.
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