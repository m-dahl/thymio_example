# Minimal example of multiplie "thymio" robots controlled by ROS2

```
colcon build
. install/setup.bash
ros2 launch thymio_example example.launch.py
```

Teleop robot 1:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/thymio1
```

Teleop robot 2:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/thymio2
```
