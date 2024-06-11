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

"""Launch actions for ArduPilot."""
from typing import List
from typing import Dict
from typing import Text
from typing import Tuple

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ardupilot_sitl.actions import ExecuteFunction


class PlatformLaunch:
    """Launch functions for ArduPilot SITL."""

    @staticmethod
    def generate_action(context: LaunchContext, *args, **kwargs) -> ExecuteProcess:
        """Return an Aerostack ArduPilot platform process."""
        # Retrieve launch arguments.
        namespace = LaunchConfiguration("namespace").perform(context)
        control_modes = LaunchConfiguration("control_modes_file").perform(context)
        platform_config = LaunchConfiguration("platform_config_file").perform(context)

        # Display launch arguments.
        print(f"namespace:        {namespace}")
        print(f"control_modes:    {control_modes}")
        print(f"platform_config:  {platform_config}")

        node = Node(
            package="as2_platform_ardupilot",
            executable="as2_platform_ardupilot_node",
            name="platform",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "control_modes_file": control_modes,
                },
                platform_config,
            ],
        )
        return node

    @staticmethod
    def generate_launch_description_with_actions() -> (
        Tuple[LaunchDescription, Dict[Text, ExecuteFunction]]
    ):
        """Generate a launch description for the ArduPilot platform."""
        launch_arguments = PlatformLaunch.generate_launch_arguments()

        action = ExecuteFunction(function=PlatformLaunch.generate_action)

        ld = LaunchDescription(
            launch_arguments
            + [
                action,
            ]
        )
        actions = {
            "as2_platform_ardupilot": action,
        }
        return ld, actions

    @staticmethod
    def generate_launch_description() -> LaunchDescription:
        """Generate a launch description."""
        ld, _ = PlatformLaunch.generate_launch_description_with_actions()
        return ld

    @staticmethod
    def generate_launch_arguments() -> List[DeclareLaunchArgument]:
        """Generate a list of launch arguments."""

        # Default control modes config file.
        control_modes = PathJoinSubstitution(
            [FindPackageShare("as2_platform_ardupilot"), "config", "control_modes.yaml"]
        )

        # Default platform config file.
        platform_config_file = PathJoinSubstitution(
            [
                FindPackageShare("as2_platform_ardupilot"),
                "config",
                "platform_config_file.yaml",
            ]
        )

        launch_args = [
            # Required launch arguments.
            DeclareLaunchArgument(
                "namespace",
                default_value="drone0",
                description="Vehicle namespace.",
            ),
            DeclareLaunchArgument(
                "control_modes_file",
                default_value=control_modes,
                description="Platform control modes file",
            ),
            DeclareLaunchArgument(
                "platform_config_file",
                default_value=platform_config_file,
                description="Platform configuration file",
            ),
        ]

        return launch_args
