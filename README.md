# SocialDWA

Modified Dynamic Window Approach local ROS planner. 

# Human-Aware DWA Local Planner

Enhanced Dynamic Window Approach (DWA) local planner for ROS Navigation Stack that incorporates human-aware navigation features. This fork extends the standard `dwa_local_planner` with social navigation capabilities.

## Key Points

- Penalizes trajectories heading toward humans
- Adjusts costs based on human movement speed
- Automatically reduces speed near humans
- Respects personal space and social norms
- Configurable ROS parameters

## Installation

### Dependencies
- ROS Noetic
- `people_msgs` package
- Standard ROS navigation stack: https://github.com/ros-planning/navigation/tree/noetic-devel/dwa_local_planner

### Building
```bash
cd ~/your_ws/src
git clone https://github.com/PaoAvalos/SocialDWA
cd ..
catkin build
