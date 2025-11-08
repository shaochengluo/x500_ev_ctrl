# CPSL_ROS2_PX4
A collection of ROS2 packages/nodes used to integrate with px4 UAVs via simulation or real-world experiments
## Building and sourcing
colcon build --packages-select cpsl_px4_bridge

## --------------------------------PX4 Control Pipeline--------------------------------------- ##

## Change PX4 namespace - QGC-Analyze Tools - Mavlink Console
uxrce_dds_client stop
uxrce_dds_client start -n cpsl_uav_7

ekf2 stop 
ekf2 start 

## Connect PX4 to ROS2
MicroXRCEAgent udp4 -p 8888


## Launch the PX4 control node
cd CPSL_ROS2_PX4
source install/setup.bash
ros2 launch px4_controller joy_control_launch.py joy_enable:=false control_enable:=true namespace:=cpsl_uav_7

## Lanuch the PX4 vicon bridge node (used luo version)
ros2 launch cpsl_px4_bridge vicon_to_px4_ev.launch.py namespace:=cpsl_uav_7

## Launch Vicon node
ros2 launch vicon_bridge all_segments.launch.py

## Change Mode to Burst+Gated+External_Source

## Launch Arduino Switch--NOTE the ACM number

# Plug in Arduino
conda deactivate
cd /home/cpsl/px4_ws/src/arduino
python switch_uno.py
## RESET ARUDINO!!! MAKE SURE FunGen does not wait for trigger sig

## Launch Interceptor
conda deactivate
cd /home/cpsl/px4_ws/src/live_ros_scripts
python gyro_sine_offset_monitor.py

## Turn on signal in FuncGen

## ---------For Debug------------------##
ros2 run sensor_combined2imu sensor_combined2imu --ros-args -p frame_id:=fmu_imu

conda deactivate
cd px4_ws/src/quat2euler
python quat2euler.py 


## Launch the waypoint controller
<!-- ros2 launch cpsl_px4_vicon_controller waypoint_mission_launch.py   params_file:=/home/cpsl/px4_ws/src/CPSL_ROS2_PX4/src/cpsl_px4_vicon_controller/cpsl_px4_vicon_controller/config/mission_example.yaml -->

## ---------------------------------------- End ---------------------------------------------- ##

## ---------------------------------------- Check ---------------------------------------------- ##
ros2 topic echo /vicon/x500_7/x500_7

ros2 topic echo /cpsl_uav_7/fmu/in/vehicle_visual_odometry
ros2 topic echo /cpsl_uav_7/fmu/out/vehicle_odometry


ros2 bag record /vicon/x500_7/x500_7 /cpsl_uav_7/fmu/in/vehicle_visual_odometry  /cpsl_uav_7/fmu/out/vehicle_odometry /cpsl_uav_7/fmu/out/sensor_combined /gyro_offset_cutoff /attack_enable /gyro_schedule_debug

### ros2 bag to csv ###
cd /home/cpsl/px4_ws/src/CPSL_UAV_Tracking/scripts
conda deactivate
python3 ros2bag_to_csv.py /home/cpsl/px4_ws/new_test_log38 --out ./csv_out