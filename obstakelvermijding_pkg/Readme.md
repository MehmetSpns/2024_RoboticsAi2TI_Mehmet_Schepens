# Obstacle Avoidance Node

This repository contains a Python-based ROS2 node that implements a simple obstacle avoidance behavior using sensor data from a LaserScan topic. The robot adjusts its linear and angular velocities based on the distances to obstacles in the front, left, and right directions.

## Prerequisites

### Dependencies

- **ROS2**: Ensure that ROS2 is installed on your system.
- **Python 3.10.12**
- ROS2 packages:
  - `geometry_msgs`
  - `sensor_msgs`

### Installation

1. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/MehmetSpns/2024_RoboticsAi2TI_Mehmet_Schepens.git
   ```
2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```
3. Source your workspace:
   ```bash
   source install/setup.bash
   ```

## Running the Node

1. Launch your simulation or connect to your robot that provides LaserScan data.
2. Run the obstacle avoidance node:
   ```bash
   ros2 launch obstakelvermijding_pkg obstakelvermijding_pkg_launch_file.launch.py 
   ```
