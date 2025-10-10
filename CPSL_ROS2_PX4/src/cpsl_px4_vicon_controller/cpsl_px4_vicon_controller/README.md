# cpsl_px4_vicon_controller — PX4 Offboard Waypoint (ROS 2 Jazzy)

Streams **OffboardControlMode** + **TrajectorySetpoint** to PX4 via uXRCE-DDS, switches to **OFFBOARD**, arms, and executes an ENU-defined waypoint mission (converted to NED internally).

## Build
```bash
cd <your_ws>
mkdir -p src && cd src
# Place this package as src/cpsl_px4_vicon_controller
cd ..
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
MicroXRCEAgent udp4 -p 8888
ros2 launch cpsl_px4_vicon_controller waypoint_mission_launch.py
```

## Parameters
- `frame`: `ENU` or `NED`
- `waypoints`: list of `[x,y,z,yaw_deg]` (yaw optional). ENU uses Up-positive z; NED uses Down-positive z.
- `xy_accept`, `z_accept`, `hold_time`: waypoint acceptance logic
- `publish_rate_hz`: Hz for streaming Offboard messages
- `auto_arm`, `auto_offboard`: auto-switch & arm

## Notes
- Keep publish rate ≥ 10 Hz for Offboard.
- Define waypoints relative to your arming point if Vicon origin ≠ PX4 local origin.
- Provide yaw on each waypoint if you want heading control; leave out to keep yaw unconstrained.


## Use a custom YAML mission
```bash
ros2 launch cpsl_px4_vicon_controller waypoint_mission_launch.py \
  params_file:=/absolute/path/to/your_mission.yaml
```

### YAML schema
```yaml
cpsl_px4_vicon_controller:
  ros__parameters:
    frame: ENU  # or NED
    waypoints:  # list of 3- or 4-tuples; yaw is degrees in the given frame
      - [x, y, z, yaw_deg]
    xy_accept: 0.10
    z_accept: 0.08
    hold_time: 0.5
    publish_rate_hz: 20.0
    auto_arm: true
    auto_offboard: true
```
