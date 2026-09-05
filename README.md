# GZ Course Week 1

A ROS 2 (Humble) + Gazebo (Harmonic) colcon workspace featuring a TurtleBot3 Burger with a Control Barrier Function (CBF) safety filter.

## Features

- Custom indoor lab world simulation
- TurtleBot3 Burger robot
- Keyboard teleop and spiral trajectory tracking
- CBF safety filter between nominal commands and robot actuation
- Ground truth pose estimation
- Trajectory recording and visualization

## Setup

```bash
colcon build
source install/setup.bash
```

## Launch

Default (teleop mode with CBF):
```bash
ros2 launch gz_course_week_1 NCR_Indoor_Lab_world.launch.py
```

Spiral trajectory with CBF:
```bash
ros2 launch gz_course_week_1 NCR_Indoor_Lab_world.launch.py mode:=spiral cbf:=true
```

Without safety filter:
```bash
ros2 launch gz_course_week_1 NCR_Indoor_Lab_world.launch.py cbf:=false
```

## Manual Control

```bash
ros2 run gz_course_week_1 teleop_keyboard
```

Controls: `w`/`x` linear velocity, `a`/`d` angular, `s` stop, `q` quit

## Testing

```bash
colcon test --packages-select gz_course_week_1
colcon test-result --verbose
```

## Visualization

```bash
rviz2 -d $(ros2 pkg prefix gz_course_week_1)/share/gz_course_week_1/rviz/cbf_trajectory.rviz
```

## Project Structure

- `src/gz_course_week_1/` - Main package
  - `gz_course_week_1/` - Python nodes and utilities
  - `launch/` - Launch files
  - `worlds/` - Gazebo world definitions
  - `models/` - Robot and environment models
  - `rviz/` - RViz configuration
