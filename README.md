# Cooperative transport with UR5
## Description
This project implements a collaborative co-transport system between a human operator and a UR5 robotic arm using **ROS2 Humble**.  
The robot follows the human’s movements by tracking a **red marker** placed on a flexible sheet, detected through an **Intel Realsense D435i** depth camera mounted on the end-effector.  

The main goal is to maintain a **fixed distance of 1 meter** along the y-axis and a **constant orientation** with respect to the target. To ensure safe and smooth human-robot interaction, an algorithm was implemented that, based on the maximum linear velocity (0.1 m/s), computes the maximum distancethe robot can travel within the timer interval (0.4 s). This results in reaching an **intermediate calculated position** along the planned trajectory. In addition, a control mechaninsm was introduced to limit joint angle variations, ensuring that the robot always ramains in a safe configuration.
### Key Features
- **Real-time visual tracking** of a red marker for reference point detection.  
- **Modular kinematic control** to maintain predefined distance and orientation.  
- **Safety constraints** on joint positions and velocities to prevent abrupt or unsafe motions.  
- **Logging and performance analysis**, with error visualization through plots.

## Requirements
- **Operating System:** Ubuntu 22.04 LTS  
- **Framework:** ROS2 Humble  
- **Robot Drivers:**  
  - [UR ROS2 Driver (including MoveIt)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)  
  - [Intel Realsense ROS2 Driver](https://github.com/IntelRealSense/realsense-ros)
- **Python libraries:**  
  - `numpy`  
  - `matplotlib`  
  - `rclpy`  
  - `datetime`  
  - `csv`ù
## Script Structure
The main package is `ur5_custom_control` and contains four key scripts:

1. **`red_dot_tracker`**  
   - Detects the red marker on the sheet using the Intel Realsense D435i.  
   - Computes the 3D position of the marker relative to the robot's `base_link` frame and publishes it on a ROS2 topic.

2. **`trajectory_sender`**  
   - Receives the 3D target position from `red_dot_tracker` relative to the `base_link`.  
   - Computes the robot trajectory to follow the target while maintaining a 1-meter offset along the y-axis.  
   - Calculates intermediate positions along the planned path to ensure smooth motion, taking into account the maximum linear velocity (0.1 m/s) and the timer          interval (0.4 s).  
   - Implements all safety conditions, including limiting joint angle variations and enforcing velocity constraints, ensuring the robot remains in a safe               configuration throughout the motion.

3. **`mover`**  
   - Executes the calculated trajectory on the UR5.  
   - Controls the end-effector to maintain fixed orientation and distance.  
   - Applies safety constraints on joint angles and velocities to prevent abrupt or unsafe motions.

4. **`plot_error`**  
   - Reads log files from the executed trajectory.  
   - Generates plots showing tracking errors for performance analysis.
## Usage
### 1. Hardware setup
- Connect the **UR5 robot** to your computer via **Ethernet**.  
- Connect the **Intel Realsense D435i** camera via **USB**.  
- Make sure PC and UR5 are on the **same subnet** (compatible IP addresses).
### 2. Launch robot drivers
 `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 launch_rviz:=true`
### 3. Teach Pendant 
On the Teach Pendant of the robot, run the '.urcap' file for External Control.
### 4. Launch MoveIt for the kinematics services
`ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 use_fake_hardware:=false`
### 5. Launch Intel Realsense drivers with pointcloud
```
    ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30 \
    pointcloud.enable:=true \
    unite_imu_method:=linear_interpolation \
    enable_sync:=true \
    align_depth.enable:=true \
    device_type:=d435i ```

### 6. Run the main scripts in order
Open **three separate terminals**, all positioned in the ROS2 workspace, and run each script:
   - `ros2 run ur5_custom_control red_dot_tracker`
   - `ros2 run ur5_custom_control trajectory_sender`
   - `ros2 run ur5_custom_control mover`
### 7. Analyze performance
  - Press **Ctrl+C** to stop execution.
  - Visualize tracking errors with command `ros2 run ur5_custom_control plot_error` 

