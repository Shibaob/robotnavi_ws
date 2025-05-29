### Build
```
colcon build --symlink-install
```

### Source
```
source install/local_setup.bash
```

### Scrip for solution
```
ros2 launch sol solution_launch.py use_rviz:=false
```
 
### Launch 2 robots, disable RViz, run for 20 min (1200 s sim-time), using map seed 36
```
ros2 launch sol solution_launch.py \
  num_robots:=2 \
  use_rviz:=false \
  random_seed:=36 \
  experiment_duration:=1200.0
```

Spawns two robots so you can observe multi-robot behaviour.
Skips RViz to save resources when you only need console output or log files.
Uses map seed 36 for a repeatable layout of items—handy when comparing runs.
Runs for 1,200 s of simulation time (≈ 20 min), giving each robot enough time to demonstrate full task cycles before the system shuts down automatically.
