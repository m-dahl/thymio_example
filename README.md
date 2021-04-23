# Minimal example of multiplie "thymio" robots controlled by ROS2

```
colcon build
. install/setup.bash
ros2 launch thymio_example example.launch.py
```

Teleop and joint states robot 1:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/thymio1
ros2 topic echo /thymio1/joint_states
```

Teleop and koint states robot 2:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/thymio2
ros2 topic echo /thymio2/joint_states
```

Send four simultaneous goals:
```
./test.bash
```
