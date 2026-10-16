################### IF you want to turn on the Navigation mode########################## 
step1. ros2 launch bringup bringup_localization.launch.py rviz:=false
- Activate mid360, imu sensor, tf_link, fastlio_localization
- if you want to change the map change here 
        launch_arguments={
            'map': '/home/unicon/nav_ws/src/bringup/map/3d/new_file.pcd',

step2. ros2 launch bringup bringup_navigation.launch.py 
- using the tf2_ros to make camera-init to odom
- slope_cloud_filter is imu sensor sensing when we go incline road. And change the costmap parameter
- But we use this only /livox/imu
- And this part was made by Jung SungJun
-If you want to change map check this part
        output='screen',
        parameters=[{
            'yaml_filename': (
                '/home/unicon/nav_ws/src/bringup/map/new_file.yaml'
            ),

- If you want to change params about path, accel, costmap check here
(I put the TEB algorithm here)
                '/home/unicon/nav_ws/src/bringup/config/'
                'nav2_pcd_params_teb.yaml'


step3. rviz2 -d /home/unicon/nav_ws/src/bringup/config/full_nav.rviz
- We can see all sensing thing(map, lidar, global / local cost map, global / local path   


step4. ros2 launch bringup bringup.launch.py 
- bringup the uart communication and wheel_cmd


step5. ros2 run bringup mode_keyboard
- Changing the mode of wheelchair


etc. 
#0. imu
ros2 launch myahrs_ros2_driver   myahrs_ros2_driver.launch.py use_rviz:=false

#Change livox lidar fov
./src/bringup/tools/set_mid360_fov_x86_64 \
  src/livox_ros_driver2/config/MID360_config.json 210
  
 #checking the tf and lidar fov
 ros2 launch bringup tf_link.launch.py rviz:=true
