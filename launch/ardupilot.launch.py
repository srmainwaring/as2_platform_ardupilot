# Copyright 2024 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""
Launch the Aerostack platform for ArduPilot.

Run with default arguments:

ros2 launch as2_platform_ardupilot ardupilot.launch.py

Show launch arguments:

ros2 launch as2_platform_ardupilot ardupilot.launch.py --show-args
"""
from launch import LaunchDescription
from as2_platform_ardupilot.launch import PlatformLaunch


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description for the ArduPilot platform."""
    return PlatformLaunch.generate_launch_description()
