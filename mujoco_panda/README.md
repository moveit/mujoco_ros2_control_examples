# mujoco_panda

## How to run

### Clone and build relevant repositories

1. mujoco_ros2_control
```
git clone https://github.com/sangteak601/mujoco_ros2_control.git -b humble-devel
cd mujoco_ros2_control
colcon build
source install/setup.bash
```

2. moveit_resources
```
git clone https://github.com/sangteak601/moveit_resources.git -b mujoco_panda
cd moveit_resources
colcon build
source install/setup.bash
```

3. mujoco_menagerie
```
git clone https://github.com/sangteak601/mujoco_menagerie.git -b mujoco_ros2_example
export MUJOCO_MENAGERIE_DIR=/PATH/TO/MUJOCO_MENAGERIE
```

### Clone and build the repository
```
git clone https://github.com/sangteak601/mujoco_ros2_control_examples.git
cd mujoco_ros2_control_examples
colcon build
source install/setup.bash
```

### Launch demo file
```
ros2 launch mujoco_panda mujoco_panda.launch.py
```