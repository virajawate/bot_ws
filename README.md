# Bot workspace
This is a sandbox package on nav2 and localization stack

## Launching Commands
Build from the ROS workspace root (the parent directory of `src`) and source
the workspace in each terminal before launching:

```sh
colcon build --packages-select bot_bringup bot_sim bot_nav bot_explorer
source install/setup.bash
```

```sh
# For Robot Visualization
ros2 launch bot_bringup bot_bringup.launch.py

# For Simulation
ros2 launch bot_sim bot_sim.launch.py

# For Mapping
ros2 launch bot_nav bot_slam.launch.py

# For exploration while mapping (in another terminal)
ros2 run bot_explorer map_explorer --ros-args -p use_sim_time:=true

# For Navigation
ros2 launch bot_nav bot_nav.launch.py
```

The mapping launch defaults to `CylinderCube.sdf`. Use
`world_file:=/absolute/path/to/world.sdf` to select another world. A custom
world must support Gazebo sensors; the explorer command above uses simulation
time to match Nav2 and SLAM.