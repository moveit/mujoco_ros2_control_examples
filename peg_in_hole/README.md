# peg_in_hole

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

2. `moveit_resources`

Clone, build and source moveit_resources package.
```
git clone https://github.com/sangteak601/moveit_resources.git -b mujoco_panda_position_error
cd moveit_resources
colcon build
source install/setup.bash
```

3. `mujoco_menagerie`

```
git clone https://github.com/sangteak601/mujoco_menagerie.git -b peg_in_hole
```

### Clone and build example repository
```
git clone https://github.com/sangteak601/mujoco_ros2_control_examples.git
cd mujoco_ros2_control_examples
# update this according to your env
colcon build
source install/setup.bash
```

### Running the MoveIt interactive marker demo
```
export MUJOCO_MENAGERIE_DIR=/PATH/TO/MUJOCO_MENAGERIE
ros2 launch peg_in_hole peg_in_hole.launch.py
```