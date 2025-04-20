# factor_slam

## Project Overview
factor_slam is a ROS1 package intended for Simultaneous Localization and Mapping (SLAM) using point cloud data and inertial measurements. It integrates sensor data from LiDAR (using the PCL library) and IMU to estimate the robot’s pose and generate a map.

## Folder Structure
- **/src**:  
  Contains the C++ node implementations:
  - `slam_node.cpp`: Implements a SLAM node that fuses point clouds using ICP and GTSAM.
  - `fuse_clouds.cpp`: Merges point clouds from multiple sensors.
  - `imu_integration.cpp`: Performs IMU data integration.
- **/include**:  
  (Header files can be placed here if needed.)
- **CMakeLists.txt**:  
  Build configuration using CMake which includes dependencies on PCL, GTSAM, Eigen, and ROS packages.
- **package.xml**:  
  ROS package manifest with build and run dependencies.
- **.gitignore**:  
  Specifies files and directories to ignore in version control.

## Usage
1. **Build the Package:**
   - Navigate to your catkin workspace.
   - Run `catkin_make` (or `catkin build` if using catkin_tools).

2. **Launch the Nodes:**
   - **SLAM Node:**  
     `rosrun factor_slam slam_node`  
     This node subscribes to point cloud topics, performs ICP alignment, and builds a map.
   - **Fuse Clouds Node:**  
     `rosrun factor_slam fuse_clouds_node`  
     Merges point clouds from multiple LiDAR sensors.
   - **IMU Integration Node:**  
     `rosrun factor_slam imu_integration`  
     Integrates IMU data and publishes the results.

3. **Topics:**
   - The package subscribes to raw sensor topics (e.g. `output`, `/forward_center_fused`, `/left_fused`, `/right_fused`) and publishes processed point clouds (e.g. `map`, `source_pc_`, `target_pc_`).

## Dependencies
- ROS (roscpp, sensor_msgs, geometry_msgs, nav_msgs, tf, etc.)
- PCL (Point Cloud Library)
- GTSAM (for nonlinear optimization)
- Eigen3

## Notes
- Ensure that all required dependencies are installed.
- Modify topic names and parameters as needed in the source files or via ROS parameter server.
- For further customization, you may adjust build flags in the CMakeLists.txt.
