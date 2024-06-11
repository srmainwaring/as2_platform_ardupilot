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

#include "as2_platform_ardupilot/ardupilot_platform.hpp"

#include <memory>
#include <string>

#include <as2_core/utils/tf_utils.hpp>

namespace ardupilot_platform
{

ArduPilotPlatform::ArduPilotPlatform()
  : as2::AerialPlatform()
{
  // dev guide states it is not necessary to call configureSensors, but it
  // is called in the PX4 platform?
  // configureSensors();

  // generate frame ids
  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  odom_frame_id_ = as2::tf::generateTfName(this, "odom");

  // create static transforms

  // create subscribers
  ap_nav_sat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/ap/navsat/navsat0", rclcpp::SensorDataQoS(),
      std::bind(&ArduPilotPlatform::apNavSatFixCallback, this, std::placeholders::_1));

  ap_battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/ap/battery/battery0", rclcpp::SensorDataQoS(),
      std::bind(&ArduPilotPlatform::apBatteryCallback, this, std::placeholders::_1));

  ap_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/ap/imu/experimental/data", rclcpp::SensorDataQoS(),
      std::bind(&ArduPilotPlatform::apImuCallback, this, std::placeholders::_1));

  ap_pose_filtered_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ap/pose/filtered", rclcpp::SensorDataQoS(),
      std::bind(&ArduPilotPlatform::apPoseFilteredCallback, this, std::placeholders::_1));

  ap_twist_filtered_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/ap/twist/filtered", rclcpp::SensorDataQoS(),
      std::bind(&ArduPilotPlatform::apTwistFilteredCallback, this, std::placeholders::_1));

  // create publishers
  ap_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/ap/cmd_vel", rclcpp::SensorDataQoS());

  ap_cmd_gps_pose_pub_ = this->create_publisher<ardupilot_msgs::msg::GlobalPosition>(
    "/ap/cmd_vel", rclcpp::SensorDataQoS());

  // create service clients 
  ap_arm_motors_client_ =
    create_client<ardupilot_msgs::srv::ArmMotors>("/ap/arm_motors");

  ap_mode_switch_client_ =
    create_client<ardupilot_msgs::srv::ModeSwitch>("/ap/mode_switch");

}

ArduPilotPlatform::~ArduPilotPlatform()
{
}

void ArduPilotPlatform::configureSensors()
{
}

bool ArduPilotPlatform::ownSendCommand()
{
  return false;
}

bool ArduPilotPlatform::ownSetArmingState(bool /*state*/)
{
  return false;
}

bool ArduPilotPlatform::ownSetOffboardControl(bool /*offboard*/)
{
  return false;
}

bool ArduPilotPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &/*msg*/)
{
  return false;
}

void ArduPilotPlatform::ownKillSwitch()
{
}

void ArduPilotPlatform::ownStopPlatform()
{
}

bool ArduPilotPlatform::ownTakeoff()
{
  return false;
}

bool ArduPilotPlatform::ownLand()
{
  return false;
}


void ArduPilotPlatform::apArm()
{
  // make the service call

  // wait

  // check status

  RCLCPP_DEBUG(this->get_logger(), "Sent arm command");
}

void ArduPilotPlatform::apDisarm()
{
  RCLCPP_DEBUG(this->get_logger(), "Sent disarm command");
}

void ArduPilotPlatform::apPublishOffboardControlMode()
{
}

void ArduPilotPlatform::apPublishTrajectorySetpoint()
{
}

void ArduPilotPlatform::apPublishAttitudeSetpoint()
{
}

void ArduPilotPlatform::apPublishRatesSetpoint()
{
}

void ArduPilotPlatform::apPublishVehicleCommand()
{
}

void ArduPilotPlatform::apNavSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr /*msg*/)
{
}

void ArduPilotPlatform::apBatteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr /*msg*/)
{
}

void ArduPilotPlatform::apImuCallback(const sensor_msgs::msg::Imu::SharedPtr /*msg*/)
{
}

void ArduPilotPlatform::apPoseFilteredCallback(const geometry_msgs::msg::PoseStamped::SharedPtr /*msg*/)
{
}

void ArduPilotPlatform::apTwistFilteredCallback(const geometry_msgs::msg::TwistStamped::SharedPtr /*msg*/)
{
}

} // namespace ardupilot_platform
