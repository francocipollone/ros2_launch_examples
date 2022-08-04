# Ros2 Launch Examples Setup

## Requirements

ROS2 Foxy is targeted

## Build

### Install dependencies

```sh
rosdep install -y --from-path src
```

### Colcon build

Build the workspace.
```sh
colcon build --packages-up-to ros2_launch_examples
```
Source the install folder.
```sh
source install/setup.bash
```


# Examples
## rosbag_record.launch.py

Launch file example for launching a simple Talker node whit a rosbag recording process.

### Try it out
Run ros2 launch.
```sh
ros2 launch ros2_launch_examples rosbag_record.launch.py
```

You can take a look at the possible launch file arguments doing:
```sh
ros2 launch ros2_launch_examples rosbag_record.launch.py -s
```
Change namespace where the talker is being executed.

```sh
ros2 launch ros2_launch_examples rosbag_record.launch.py talker_ns:="my_random_ns" 
```

## node_component_process.launch.py
Launch file with Nodes, components and process definitions.
### Try it out

Run ros2 launch.
```sh
ros2 launch ros2_launch_examples node_component_process.launch.py
```

### Check the ros2 architecture
Run a rqt_graph to check all the nodes, components and process(on this case rosbag2)
```sh
rqt_graph
```

## substitution_argument_parameter.launch.py
Launch file that uses arguments to modify variables values using substitutions and launch a node with a specific parameter configuration.
### Try it out
Run ros2 launch.
```sh
ros2 launch ros2_launch_examples substitution_argument_parameter.launch.py
```

### Take a look at the arguments
```sh
ros2 launch ros2_launch_examples substitution_argument_parameter.launch.py --show-args
```
### Launch passing arguments on the console
```sh
ros2 launch ros2_launch_examples substitution_argument_parameter.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```

## parent_child.launch.py
Launch file that calls other launch files on a specific package
### Try it out
Run ros2 launch.
```sh
ros2 launch ros2_launch_examples parent_child.launch.py
```

## composable_node_yaml.launch.py
Launch file with composable nodes with parameters configured via a YAML file.
### Try it out
Run ros2 launch.
```sh
ros2 launch ros2_launch_examples composable_node_yaml.launch.py
```
