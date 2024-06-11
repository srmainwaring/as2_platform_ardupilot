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

#include <as2_core/aerial_platform.hpp>
#include <as2_core/node.hpp>

#include <rclcpp/publisher.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/rclcpp.hpp>

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
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>


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

  // Publishers
  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_;

  // Subscribers
  // rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_state_sub_;

private:
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
};

} // namespace ardupilot_platform

#endif // ARDUPILOT_PLATFORM_HPP_
