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

#include <rmw/qos_profiles.h>

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
          this, "/ap/pose/filtered", rmw_qos_profile_sensor_data);

  ap_twist_filtered_sub_ =
      std::make_unique<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>>(
          this, "/ap/twist/filtered", rmw_qos_profile_sensor_data);

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
  //! @note the aerial platform expects sensor topics to have standard names
  //  and currently does not support multiple instances
  barometer_ = std::make_unique<as2::sensors::Barometer>("barometer", this);
  battery_ = std::make_unique<as2::sensors::Battery>("battery", this);
  compass_ = std::make_unique<as2::sensors::Compass>("compass", this);
  gps_ = std::make_unique<as2::sensors::GPS>("gps", this);
  imu_ = std::make_unique<as2::sensors::Imu>("imu", this);
  odometry_filtered_ = std::make_unique<as2::sensors::Odometry>("odom", this);
}

bool ArduPilotPlatform::ownSendCommand()
{
  /**
   * yaw_mode
   * NONE
   * YAW_ANGLE
   * YAW_SPEED
   *
   * control_mode
   * UNSET
   * HOVER
   * POSITION
   * SPEED
   * SPEED_IN_A_PLANE
   * ATTITUDE
   * ACRO
   * TRAJECTORY
   * ACEL
   *
   * reference_frame
   * UNDEFINED_FRAME
   * LOCAL_ENU_FRAME
   * BODY_FLU_FRAME
   * GLOBAL_LAT_LONG_ASML
   * 
   * AerialPlatform has the following protected attributes
   * 
   * // set by the parameter cmd_freq
   * // used to set the rate at which sendCommand() is called
   * float cmd_freq_
   * 
   * // set by the parameter info_freq
   * // used to set the rate at which publishPlatformInfo() is called
   * float info_freq_
   * 
   * // updated in subscription to trajectory actuator commands
   * as2_msgs::msg::TrajectoryPoint command_trajectory_msg_
   * 
   * // updated in subscription to pose actuator commands
   * geometry_msgs::msg::PoseStamped command_pose_msg_
   * 
   * // updated in subscription to twist actuator commands
   * geometry_msgs::msg::TwistStamped command_twist_msg_
   * 
   * // updated in subscription to thrust actuator commands
   * as2_msgs::msg::Thrust command_thrust_msg_
   * 
   * // updated in protected methods
   * as2_msgs::msg::PlatformInfo platform_info_msg_
   * 
   * // set true whenever a command message is received
   * // set false when the offboard control mode changes
   * bool has_new_references_ = false
   * 
   */

  // current control mode
  as2_msgs::msg::ControlMode platform_control_mode = this->getControlMode();

  switch (platform_control_mode.control_mode)
  {
    case as2_msgs::msg::ControlMode::UNSET: {
      return apSetModeToLoiter();
    } break;
    case as2_msgs::msg::ControlMode::HOVER: {
      return false;
    } break;
    case as2_msgs::msg::ControlMode::POSITION: {
      return false;
    } break;
    case as2_msgs::msg::ControlMode::SPEED: {
      apPublishRatesSetpoint();
    } break;
    default: {
      RCLCPP_WARN(this->get_logger(), "Control mode %s not supported",
          as2::control_mode::controlModeToString(platform_control_mode).c_str());
      return false;
    }
  }

  return true;
}

// set the arming state of the vehicle
bool ArduPilotPlatform::ownSetArmingState(bool state)
{
  if (state) {
    return this->apArm();
  } else {
    return this->apDisarm();
  }
}

// set the control mode the vehicle to offboard (guided)
bool ArduPilotPlatform::ownSetOffboardControl(bool offboard)
{
  //! @todo(srmainwaring) record the initial control state prior,
  // to offboard control being enabled.  
  if (offboard == false) {
      RCLCPP_INFO(this->get_logger(), "Switching to LOITER mode");
      return apSetModeToLoiter();
  }

  //! @todo(srmainwaring) the PX4 platform sets a rate setpoint 10x
  //  before switching to guided. Not replicated here.   
  RCLCPP_INFO(this->get_logger(), "Switching to GUIDED mode");
  return apSetModeToGuided();
}

// set the vehicle offboard control mode
bool ArduPilotPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg)
{

  RCLCPP_INFO_STREAM(this->get_logger(), "Control mode: ["
      << as2::control_mode::controlModeToString(msg) << "]");

  switch (msg.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET: {
      return apSetModeToLoiter();
    } break;
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
      return false;
  }

  return true;
}

void ArduPilotPlatform::ownKillSwitch()
{
  RCLCPP_ERROR(this->get_logger(), "Kill switch not supported");
}

void ArduPilotPlatform::ownStopPlatform()
{
  RCLCPP_ERROR(this->get_logger(), "Stop platform not supported");
}

// bool ArduPilotPlatform::ownTakeoff()
// {
//   RCLCPP_ERROR(this->get_logger(), "Takeoff not supported");
//   return false;
// }

// bool ArduPilotPlatform::ownLand()
// {
//   RCLCPP_ERROR(this->get_logger(), "Land not supported");
//   return false;
// }

bool ArduPilotPlatform::apArm()
{
  auto request = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
  request->arm = true;

  if (!ap_arm_motors_client_->wait_for_service(1s)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Service ["
        << ap_arm_motors_client_->get_service_name()
        << "] not available.");
    return false;
  }

  using ServiceResponseFuture =
      rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedFuture;
  auto callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    RCLCPP_INFO_STREAM(this->get_logger(), "Arm request status: "
        << result->result);
  };
  auto future = ap_arm_motors_client_->async_send_request(request/*, callback*/);
  return true;

  //! @todo(srmainwaring) blocking
  // bool did_arm = false;
  // std::future_status status = future.wait_for(1s);
  // switch (status) {
  //   case std::future_status::ready: {
  //       if (future.get()->result) {
  //         did_arm = true;
  //       }
  //   } break;
  //   default:
  //     break;
  // }

  // if (did_arm) {
  //   RCLCPP_INFO_STREAM(this->get_logger(), "Arm succeeded");
  // } else {
  //   RCLCPP_WARN_STREAM(this->get_logger(), "Arm failed");
  // }
  // return did_arm;
}

bool ArduPilotPlatform::apDisarm()
{
  auto request = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
  request->arm = false;

  if (!ap_arm_motors_client_->wait_for_service(1s)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Service ["
        << ap_arm_motors_client_->get_service_name()
        << "] not available.");
    return false;
  }

  using ServiceResponseFuture =
      rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedFuture;
  auto callback = [this](ServiceResponseFuture future) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Disarm request status: "
        << future.get()->result);
  };
  auto future = ap_arm_motors_client_->async_send_request(request, callback);
  return true;

  //! @todo(srmainwaring) blocking
  // bool did_disarm = false;
  // std::future_status status = future.wait_for(1s);
  // switch (status) {
  //   case std::future_status::ready: {
  //       if (future.get()->result) {
  //         did_disarm = true;
  //       }
  //   } break;
  //   default:
  //     break;
  // }

  // if (did_disarm) {
  //   RCLCPP_INFO_STREAM(this->get_logger(), "Disarm succeeded");
  // } else {
  //   RCLCPP_WARN_STREAM(this->get_logger(), "Disarm failed");
  // }
  // return did_disarm;
}

bool ArduPilotPlatform::apSetMode(uint8_t mode)
{
  auto request = std::make_shared<ardupilot_msgs::srv::ModeSwitch::Request>();
  request->mode = mode;

  if (!ap_mode_switch_client_->wait_for_service(1s)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Service ["
        << ap_mode_switch_client_->get_service_name()
        << "] not available.");
    return false;
  }

  using ServiceResponseFuture =
      rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedFuture;
  auto callback = [this](ServiceResponseFuture future) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Mode switch request: status: "
        << future.get()->status
        << ", current mode: "
        << static_cast<int>(future.get()->curr_mode));
  };
  auto future = ap_mode_switch_client_->async_send_request(request, callback);
  return true;

  //! @todo(srmainwaring) blocking
  // bool did_change_mode = false;
  // std::future_status status = future.wait_for(5s);
  // switch (status) {
  //   case std::future_status::ready: {
  //       if (future.get()->status && future.get()->curr_mode == mode) {
  //         RCLCPP_INFO_STREAM(this->get_logger(), "Mode switch succeeded: "
  //             << static_cast<int>(future.get()->curr_mode));
  //         did_change_mode = true;
  //       } else {
  //         RCLCPP_WARN_STREAM(this->get_logger(), "Mode switch failed: "
  //             << static_cast<int>(future.get()->curr_mode));
  //       }
  //   } break;
  //   case std::future_status::timeout: {
  //         RCLCPP_WARN_STREAM(this->get_logger(), "Mode switch timed out: "
  //             << static_cast<int>(mode));
  //   } break;
  //   default:
  //     RCLCPP_WARN_STREAM(this->get_logger(), "Mode switch failed: "
  //         << static_cast<int>(mode));
  //     break;
  // }
  // return did_change_mode;
}

bool ArduPilotPlatform::apSetModeToGuided()
{
  constexpr uint8_t mode_guided = 4;
  return apSetMode(mode_guided);
}

bool ArduPilotPlatform::apSetModeToLand()
{
  constexpr uint8_t mode_land = 9;
  return apSetMode(mode_land);
}

bool ArduPilotPlatform::apSetModeToLoiter()
{
  constexpr uint8_t mode_loiter = 5;
  return apSetMode(mode_loiter);
}

bool ArduPilotPlatform::apSetModeToRTL()
{
  constexpr uint8_t mode_rtl = 6;
  return apSetMode(mode_rtl);
}

bool ArduPilotPlatform::apSetModeToStabilize()
{
  constexpr uint8_t mode_stabilize = 0;
  return apSetMode(mode_stabilize);
}

void ArduPilotPlatform::apPublishTrajectorySetpoint()
{
}

void ArduPilotPlatform::apPublishAttitudeSetpoint()
{
}

void ArduPilotPlatform::apPublishRatesSetpoint()
{
  // ArduPilot expects frame_id: `map`
  geometry_msgs::msg::TwistStamped ap_cmd_vel = command_twist_msg_;
  ap_cmd_vel.header.frame_id = "map";
  ap_cmd_vel_pub_->publish(ap_cmd_vel);
}

void ArduPilotPlatform::apNavSatFixCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  // RCLCPP_INFO_STREAM(this->get_logger(), "Received GPS data");
  gps_->updateData(*msg);
}

void ArduPilotPlatform::apBatteryCallback(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
{
  // RCLCPP_INFO_STREAM(this->get_logger(), "Received battery data");
  battery_->updateData(*msg);
}

void ArduPilotPlatform::apImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  // RCLCPP_INFO_STREAM(this->get_logger(), "Received IMU data");
  imu_->updateData(*msg);
}

void ArduPilotPlatform::apOdomCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg,
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg)
{
  // RCLCPP_INFO_STREAM(this->get_logger(), "Received odom data");

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
