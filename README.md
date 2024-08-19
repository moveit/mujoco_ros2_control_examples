# mujoco_ros2_control_examples

## How to run

### Clone and build relevant repositories

1. `mujoco_ros2_control`

Before clone the repo, please make sure you have [mujoco](https://github.com/google-deepmind/mujoco/releases) installed.
```
git clone https://github.com/sangteak601/mujoco_ros2_control.git -b humble-devel
cd mujoco_ros2_control
# update this according to your env
export MUJOCO_DIR=/PATH/TO/MUJOCO/mujoco-3.x.x
colcon build
source install/setup.bash
```

### Clone and build example repository
```
git clone https://github.com/sangteak601/mujoco_ros2_control_examples.git
cd mujoco_ros2_control_examples
colcon build
source install/setup.bash
```

### Running demos
```
ros2 launch interactive_marker interactive_marker.launch.py
ros2 launch peg_in_hole peg_in_hole.launch.py
```