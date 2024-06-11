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


  // create ardupilot_dds subscribers


  // create ardupilot_dds publishers

  // 

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

} // namespace ardupilot_platform
