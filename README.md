# Co-trasporto collaborativo Umano-Robot: implementazione in ROS2 con UR5 e inseguimento visivo con telecamera di profondità
## Descrizione breve
Il software consiste in un sistema di controllo robotico in ROS2 che abilita il co-trasporto di un telo tra operatore umano e UR5, utilizzando l'inseguimento visivo grazie a una telecamera di profondità Intel Realsense D435i montata sul braccio robotico.
Le caratteristiche principali sono l'integrazione del riconoscimento visivo del punto rosso, controllo cinematico del robot per l'inseguimento di un target mantenedo una distanza di un metro dal punto rosso con orientamento fisso dell'end effector, implementazione di vincoli di sicurezza sui movimenti dei giunti e velocità, logging e analisi delle prestazioni mostrando gli errori in dei grafici.
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

