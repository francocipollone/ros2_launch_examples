# execute_rosbag_py

Launch file example for launching a simple Talker node whit a rosbag recording process.

## Requirements

ROS2 Foxy is targeted

## Build

### Install dependencies

```sh
rosdep install -y --from-path src
```

### Colcon build

```sh
colcon build --packages-up-to execute_rosbag_py
```

### Try it out

```sh
source install/setup.bash
```

Run ros2 launch.
```sh
ros2 launch execute_rosbag_py rosbag_record.launch.py
```

You can take a look at the possible launch file arguments doing:
```sh
ros2 launch execute_rosbag_py rosbag_record.launch.py -s
```
#### Launch Arguments
Change namespace where the talker is being executed.

```sh
ros2 launch execute_rosbag_py rosbag_record.launch.py talker_ns:="my_random_ns" 
```