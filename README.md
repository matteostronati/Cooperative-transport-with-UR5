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

## Requisiti
- Ubuntu versione 22.04 LTS 
- ROS2 Humble
- Driver ROS2 per UR5 (compreso moveit) e Intel Realsense
- Librerie Python (numpy, matplotlib, rclpy, datetime, csv)
## Utilizzo
- Collegamento via Ethernet tra UR5 e Computer con ROS2
- Collegamento via USB tra Intel Realsense e Computer con ROS2
- Impostare indirizzi IP compatibili tra Computer e UR5, in modo che siano nella stessa maschera
- Lanciare i driver per l'UR5 con il comando `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 launch_rviz:=true`
- Eseguire dal Teach Pendant del robot il file .urcap per l'External Control
- Lanciare moveit per avere accesso ai servizi di cinematica diretta e inversa con 
`ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 use_fake_hardware:=false`
- Lanciare i driver per la telecamera Intel Realsense D435i con la pointcloud disponibile con il comando:  
    ```
    ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30 \
    pointcloud.enable:=true \
    unite_imu_method:=linear_interpolation \
    enable_sync:=true \
    align_depth.enable:=true \
    device_type:=d435i
- Lanciare i tre script, posizionandosi all'interno del workspace secondo questa sequenza:  
1. `ros2 run ur5_custom_control red_dot_tracker`
2. `ros2 run ur5_custom_control trajectory_sender`
3. `ros2 run ur5_custom_control mover`
- Premere Ctrl+C nel terminale per fermare l'esecuzione e eseguire il comando `ros2 run ur5_custom_control plot_error` per visualizzare i grafici dell'errore

