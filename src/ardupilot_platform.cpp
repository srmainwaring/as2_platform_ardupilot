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

#include <chrono>
#include <memory>
#include <string>

#include <as2_core/sensor.hpp>
#include <as2_core/utils/tf_utils.hpp>

using namespace std::chrono_literals;

namespace ardupilot_platform
{

ArduPilotPlatform::ArduPilotPlatform()
  : as2::AerialPlatform()
{
  // dev guide states it is not necessary to call configureSensors, but it
  // is called in the PX4 platform?
  configureSensors();

  // generate frame ids
  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  odom_frame_id_ = as2::tf::generateTfName(this, "odom");

  // declare parameters - is this required for ArduPilot?
  this->declare_parameter<float>("mass");
  mass_ = this->get_parameter("mass").as_double();

  this->declare_parameter<float>("max_thrust");
  max_thrust_ = this->get_parameter("max_thrust").as_double();

  this->declare_parameter<float>("min_thrust");
  min_thrust_ = this->get_parameter("min_thrust").as_double();

  this->declare_parameter<bool>("use_external_odom");
  use_external_odom_ = this->get_parameter("use_external_odom").as_bool();

  RCLCPP_INFO(this->get_logger(), "Use sim time: %s",
      this->get_parameter("use_sim_time").as_bool() ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Mass: %f", mass_);
  RCLCPP_INFO(this->get_logger(), "Max thrust: %f", max_thrust_);
  RCLCPP_INFO(this->get_logger(), "Min thrust: %f", min_thrust_);
  RCLCPP_INFO(this->get_logger(), "Use external odometry: %s",
      use_external_odom_ ? "true" : "false");

  // create static transforms

  // create subscribers
  ap_nav_sat_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/ap/navsat/navsat0", rclcpp::SensorDataQoS(),
      std::bind(&ArduPilotPlatform::apNavSatFixCallback, this, std::placeholders::_1));

  ap_battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/ap/battery/battery0", rclcpp::SensorDataQoS(),
      std::bind(&ArduPilotPlatform::apBatteryCallback, this, std::placeholders::_1));

  ap_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/ap/imu/experimental/data", rclcpp::SensorDataQoS(),
      std::bind(&ArduPilotPlatform::apImuCallback, this, std::placeholders::_1));

  // message filters
  ap_pose_filtered_sub_ =
      std::make_unique<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
          this, "/ap/pose/filtered");

  ap_twist_filtered_sub_ =
      std::make_unique<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>>(
          this, "/ap/twist/filtered");

  ap_odom_sync_ = std::make_unique<message_filters::TimeSynchronizer<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped>>(
      *ap_pose_filtered_sub_, *ap_twist_filtered_sub_, 5);
  ap_odom_sync_->registerCallback(
      std::bind(&ArduPilotPlatform::apOdomCallback, this, std::placeholders::_1, std::placeholders::_2));

  // create publishers
  ap_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/ap/cmd_vel", rclcpp::SensorDataQoS());

  ap_cmd_gps_pose_pub_ = this->create_publisher<ardupilot_msgs::msg::GlobalPosition>(
    "/ap/cmd_gps_pose", rclcpp::SensorDataQoS());

  // create service clients 
  ap_arm_motors_client_ =
    this->create_client<ardupilot_msgs::srv::ArmMotors>("/ap/arm_motors");

  ap_mode_switch_client_ =
    this->create_client<ardupilot_msgs::srv::ModeSwitch>("/ap/mode_switch");
}

ArduPilotPlatform::~ArduPilotPlatform()
{
}

void ArduPilotPlatform::configureSensors()
{
  barometer0_ = std::make_unique<as2::sensors::Barometer>("barometer0", this);
  battery0_ = std::make_unique<as2::sensors::Battery>("battery0", this);
  compass0_ = std::make_unique<as2::sensors::Compass>("compass0", this);
  gps0_ = std::make_unique<as2::sensors::GPS>("gps0", this);
  imu0_ = std::make_unique<as2::sensors::Imu>("imu0", this);
  odometry_filtered_ = std::make_unique<as2::sensors::Odometry>("odom", this);
}

bool ArduPilotPlatform::ownSendCommand()
{
  // current control mode
  // as2_msgs::msg::ControlMode platform_control_mode = this->getControlMode();


  // ap_cmd_vel_;

  RCLCPP_ERROR(this->get_logger(), "Send command not supported");
  return false;
}

bool ArduPilotPlatform::ownSetArmingState(bool state)
{
  if (state) {
    this->apArm();
  } else {
    // set_disarm_ = true;
    this->apDisarm();
  }
  return true;
}

bool ArduPilotPlatform::ownSetOffboardControl(bool /*offboard*/)
{
  RCLCPP_DEBUG(this->get_logger(), "Switching to GUIDED mode");

  //! @todo(srmainwaring) the PX4 platform sets a rate setpoint 10x
  //  before switching to guided. Not replicated here.   
  apSetModeToGuided();
  return true;
}

bool ArduPilotPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg)
{
  switch (msg.control_mode) {
    case as2_msgs::msg::ControlMode::POSITION: {
      RCLCPP_INFO(this->get_logger(), "POSITION mode enabled");
    } break;
    case as2_msgs::msg::ControlMode::SPEED: {
      RCLCPP_INFO(this->get_logger(), "SPEED mode enabled");
    } break;
    case as2_msgs::msg::ControlMode::ATTITUDE: {
      RCLCPP_INFO(this->get_logger(), "ATTITUDE mode enabled");
    } break;
    default:
      RCLCPP_WARN(this->get_logger(), "Control mode %s not supported",
          as2::control_mode::controlModeToString(msg).c_str());
      is_valid_mode_ = false;
      return false;
  }

  is_valid_mode_ = true;
  return false;
}

void ArduPilotPlatform::ownKillSwitch()
{
  RCLCPP_ERROR(this->get_logger(), "Kill switch not supported");
}

void ArduPilotPlatform::ownStopPlatform()
{
  RCLCPP_ERROR(this->get_logger(), "Stop platform not supported");
}

bool ArduPilotPlatform::ownTakeoff()
{
  RCLCPP_ERROR(this->get_logger(), "Takeoff not supported");
  return false;
}

bool ArduPilotPlatform::ownLand()
{
  RCLCPP_ERROR(this->get_logger(), "Land not supported");
  return false;
}


void ArduPilotPlatform::apArm()
{
  auto request = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
  request->arm = true;

  if (!ap_arm_motors_client_->wait_for_service(1s)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Service ["
        << ap_arm_motors_client_->get_service_name()
        << "] not available.");
    return;
  }

  bool got_response = false;
  using ServiceResponseFuture =
      rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedFuture;
  auto callback = [&got_response, this](ServiceResponseFuture future) {
    got_response = true;
    auto result = future.get();
    RCLCPP_INFO_STREAM(this->get_logger(), "Arm request status: "
        << result->result);
  };
  auto future = ap_arm_motors_client_->async_send_request(request, callback);
}

void ArduPilotPlatform::apDisarm()
{
  auto request = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
  request->arm = false;

  if (!ap_arm_motors_client_->wait_for_service(1s)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Service ["
        << ap_arm_motors_client_->get_service_name()
        << "] not available.");
    return;
  }

  bool got_response = false;
  using ServiceResponseFuture =
      rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedFuture;
  auto callback = [&got_response, this](ServiceResponseFuture future) {
    got_response = true;
    auto result = future.get();
    RCLCPP_INFO_STREAM(this->get_logger(), "Disarm request status: "
        << result->result);
  };
  auto future = ap_arm_motors_client_->async_send_request(request, callback);
}

void ArduPilotPlatform::apSetModeToGuided()
{
  auto request = std::make_shared<ardupilot_msgs::srv::ModeSwitch::Request>();
  request->mode = 4; // GUIDED = 4 for Copter

  if (!ap_mode_switch_client_->wait_for_service(1s)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Service ["
        << ap_mode_switch_client_->get_service_name()
        << "] not available.");
    return;
  }

  bool got_response = false;
  using ServiceResponseFuture =
      rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedFuture;
  auto callback = [&got_response, this](ServiceResponseFuture future) {
    got_response = true;
    auto result = future.get();
    RCLCPP_INFO_STREAM(this->get_logger(), "Mode switch request status: "
        << result->status);
    RCLCPP_INFO_STREAM(this->get_logger(), "Current mode: "
        << result->curr_mode);
  };
  auto future = ap_mode_switch_client_->async_send_request(request, callback);
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

void ArduPilotPlatform::apNavSatFixCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  gps0_->updateData(*msg);
}

void ArduPilotPlatform::apBatteryCallback(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
{
  battery0_->updateData(*msg);
}

void ArduPilotPlatform::apImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  imu0_->updateData(*msg);
}

void ArduPilotPlatform::apOdomCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg,
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg)
{
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp    = pose_msg->header.stamp;
  odom_msg.header.frame_id = odom_frame_id_;
  odom_msg.child_frame_id  = base_link_frame_id_;

  odom_msg.pose.pose.position.x = pose_msg->pose.position.x;
  odom_msg.pose.pose.position.y = pose_msg->pose.position.y;
  odom_msg.pose.pose.position.z = pose_msg->pose.position.z;
  odom_msg.pose.pose.orientation.x = pose_msg->pose.orientation.x;
  odom_msg.pose.pose.orientation.y = pose_msg->pose.orientation.y;
  odom_msg.pose.pose.orientation.z = pose_msg->pose.orientation.z;
  odom_msg.pose.pose.orientation.w = pose_msg->pose.orientation.w;

  odom_msg.twist.twist.linear.x = twist_msg->twist.linear.x;
  odom_msg.twist.twist.linear.y = twist_msg->twist.linear.y;
  odom_msg.twist.twist.linear.z = twist_msg->twist.linear.z;
  odom_msg.twist.twist.angular.x = twist_msg->twist.angular.x;
  odom_msg.twist.twist.angular.y = twist_msg->twist.angular.y;
  odom_msg.twist.twist.angular.z = twist_msg->twist.angular.z;

  odometry_filtered_->updateData(odom_msg);
}

} // namespace ardupilot_platform
