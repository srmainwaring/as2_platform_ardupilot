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

namespace ardupilot_platform
{

ArduPilotPlatform::ArduPilotPlatform()
  : as2::AerialPlatform()
{
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

bool ArduPilotPlatform::ownSetArmingState(bool state)
{
  return false;
}

bool ArduPilotPlatform::ownSetOffboardControl(bool offboard)
{
  return false;
}

bool ArduPilotPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg)
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
