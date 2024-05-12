# explorer_wanderer

REF: https://github.com/DaniGarciaLopez/ros2_explorer/tree/main  

Searched for "ROS2 wander node" found explorer_wanderer package which uses LIDAR /scan topic to avoid walls/obstacles.  

Changes:  
  * wanderer.py
    - replaced explicit qos profile of "10" with qos_profile_sensor_data, which is 10 with BEST_EFFORT   
    - scan range[0] faces back on GoPiGo3 robot Dave  
    - to average /scan ranges to tolerate LIDAR returning ocaisional zero values  
    - QoS profile to depth 10 with BEST_EFFORT  
    - Lowered allowed distance from obstacles to 350mm (GoPiGo3 safe turning circle is 140mm)  
    - Lowered max_speed to 0.1m/s to prevent tip-over when stopping GoPiGo3 robot HumbleDave
    - Change algorithm to lower speed when closer than twice the minimum obstacle distance_to_wall
  * setup.cfg
    - changed script-dir to script_dir, install-dir to install_dir
