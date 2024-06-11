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

#include <as2_core/core_functions.hpp>
#include <as2_platform_ardupilot/ardupilot_platform.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ardupilot_platform::ArduPilotPlatform>();
  node->preset_loop_frequency(300);
  as2::spinLoop(node);

  rclcpp::shutdown();
  return 0;
}
