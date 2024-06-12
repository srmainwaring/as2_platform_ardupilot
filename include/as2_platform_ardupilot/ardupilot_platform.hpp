/**
 * Copyright 2024 ArduPilot.org.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef ARDUPILOT_PLATFORM_HPP_
#define ARDUPILOT_PLATFORM_HPP_

#include <memory>
#include <string>

#include <as2_core/aerial_platform.hpp>
#include <as2_core/node.hpp>
#include <as2_core/sensor.hpp>

#include <rclcpp/publisher.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ardupilot_msgs/msg/global_position.hpp>
#include <ardupilot_msgs/srv/arm_motors.hpp>
#include <ardupilot_msgs/srv/mode_switch.hpp>
#include <as2_msgs/msg/alert_event.hpp>
#include <as2_msgs/msg/control_mode.hpp>
#include <as2_msgs/msg/platform_info.hpp>
#include <as2_msgs/msg/platform_status.hpp>
#include <as2_msgs/msg/thrust.hpp>
#include <as2_msgs/msg/trajectory_point.hpp>
#include <as2_msgs/srv/list_control_modes.hpp>
#include <as2_msgs/srv/set_control_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>


namespace ardupilot_platform
{

class ArduPilotPlatform : public as2::AerialPlatform
{
public:
  ArduPilotPlatform();
  virtual ~ArduPilotPlatform();

public:
  void configureSensors() override;
  bool ownSendCommand() override;
  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg) override;
  void ownKillSwitch() override;
  void ownStopPlatform() override;
  bool ownTakeoff() override;
  bool ownLand() override;

private:
  // frame ids
  std::string base_link_frame_id_;
  std::string odom_frame_id_;

  // parameters
  float mass_;
  float max_thrust_;
  float min_thrust_;
  bool use_external_odom_ {true};

  // as2_msgs::msg::ControlMode control_in_;
  // double yaw_rate_limit_ = M_PI_2;

  // bool enable_takeoff_           = false;
  // bool enable_land_              = false;
  // bool state_received_           = false;
  // double current_height_         = 0.0;
  // double current_vertical_speed_ = 0.0;
  // std::shared_ptr<as2::tf::TfHandler> tf_handler_;

  // void resetCommandTwistMsg();
  // void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);

  // Sensors
  std::unique_ptr<as2::sensors::Barometer> barometer0_;
  std::unique_ptr<as2::sensors::Battery> battery0_;
  std::unique_ptr<as2::sensors::Compass> compass0_;
  std::unique_ptr<as2::sensors::GPS> gps0_;
  std::unique_ptr<as2::sensors::Imu> imu0_;
  std::unique_ptr<as2::sensors::Odometry> odometry_filtered_;

  // Ardupilot publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ap_cmd_vel_pub_;
  rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr ap_cmd_gps_pose_pub_;

  // Ardupilot subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr ap_nav_sat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr ap_battery_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ap_imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ap_pose_filtered_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ap_twist_filtered_sub_;

  // Ardupilot service clients
  rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedPtr ap_arm_motors_client_;
  rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedPtr ap_mode_switch_client_;

  // ArduPilot functions
  void apArm();
  void apDisarm();
  void apPublishOffboardControlMode();
  void apPublishTrajectorySetpoint();
  void apPublishAttitudeSetpoint();
  void apPublishRatesSetpoint();
  void apPublishVehicleCommand();

  // ArduPilot callbacks
  void apNavSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void apBatteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void apImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void apPoseFilteredCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void apTwistFilteredCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
};

} // namespace ardupilot_platform

#endif // ARDUPILOT_PLATFORM_HPP_
