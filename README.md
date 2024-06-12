# as2_platform_ardupilot

[Aerostack2](https://aerostack2.github.io/) aerial platform for the ArduPilot autopilot.

## ArduPilot Msgs:

### Available

#### Publishers

- `/ap/battery/battery0: sensor_msgs/msg/BatteryState`
- `/ap/clock: rosgraph_msgs/msg/Clock`
- `/ap/geopose/filtered: geographic_msgs/msg/GeoPoseStamped`
- `/ap/gps_global_origin/filtered: geographic_msgs/msg/GeoPointStamped`
- `/ap/imu/experimental/data: sensor_msgs/msg/Imu`
- `/ap/navsat/navsat0: sensor_msgs/msg/NavSatFix`
- `/ap/pose/filtered: geometry_msgs/msg/PoseStamped`
- `/ap/tf_static: tf2_msgs/msg/TFMessage`
- `/ap/time: builtin_interfaces/msg/Time`
- `/ap/twist/filtered: geometry_msgs/msg/TwistStamped`

#### Subscribers

- `/ap/cmd_gps_pose: ardupilot_msgs/msg/GlobalPosition`
- `/ap/cmd_vel: geometry_msgs/msg/TwistStamped`
- `/ap/joy: sensor_msgs/msg/Joy`
- `/ap/tf: tf2_msgs/msg/TFMessage`

#### Service Servers

- `/ap/arm_motors: ardupilot_msgs/srv/ArmMotors`
- `/ap/mode_switch: ardupilot_msgs/srv/ModeSwitch`


### Used

- Battery: `/ap/battery/battery0: sensor_msgs/msg/BatteryState`
- IMU: `/ap/imu/experimental/data: sensor_msgs/msg/Imu`
- GPS: `/ap/navsat/navsat0: sensor_msgs/msg/NavSatFix`
- Odometry: `/ap/pose/filtered: geometry_msgs/msg/PoseStamped`
- Odometry: `/ap/twist/filtered: geometry_msgs/msg/TwistStamped`

- Arm: `/ap/arm_motors: ardupilot_msgs/srv/ArmMotors`
- Disarm: `/ap/arm_motors: ardupilot_msgs/srv/ArmMotors`
- Mode: `/ap/mode_switch: ardupilot_msgs/srv/ModeSwitch`
