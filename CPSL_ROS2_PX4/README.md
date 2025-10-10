# CPSL_ROS2_PX4
A collection of ROS2 packages/nodes used to integrate with px4 UAVs via simulation or real-world experiments
## Building and sourcing
colcon build --packages-select cpsl_px4_bridge

## --------------------------------PX4 Control Pipeline--------------------------------------- ##

## Change PX4 namespace - QGC-Analyze Tools - Mavlink Console
uxrce_dds_client stop

uxrce_dds_client start -n cpsl_uav_7

## Connect PX4 to ROS2
MicroXRCEAgent udp4 -p 8888


## Launch the PX4 control node
cd CPSL_ROS2_PX4
source install/setup.bash
ros2 launch px4_controller joy_control_launch.py joy_enable:=false control_enable:=true namespace:=cpsl_uav_7

## Lanuch the PX4 vicon bridge node
ros2 launch cpsl_px4_bridge vicon_to_px4_ev.launch.py namespace:=cpsl_uav_7

## Spoofing PX4 vicon bridge
## ros2 launch cpsl_px4_bridge spoof_vicon_to_px4_ev.launch.py

## Launch Vicon node
ros2 launch vicon_bridge all_segments.launch.py

## Launch the waypoint controller
ros2 launch cpsl_px4_vicon_controller waypoint_mission_launch.py   params_file:=/home/cpsl/px4_ws/src/CPSL_ROS2_PX4/src/cpsl_px4_vicon_controller/cpsl_px4_vicon_controller/config/mission_example.yaml

## ---------------------------------------- End ---------------------------------------------- ##

## ---------------------------------------- Check ---------------------------------------------- ##
ros2 topic echo /cpsl_uav_7/fmu/in/vehicle_visual_odometry
ros2 topic echo /cpsl_uav_7/fmu/out/vehicle_odometry

ros2 topic echo /vicon/x500_7/x500_7

ros2 bag record /vicon/x500_7/x500_7 /cpsl_uav_7/fmu/in/vehicle_visual_odometry  /cpsl_uav_7/fmu/out/vehicle_odometry /cpsl_uav_7/fmu/out/sensor_combined