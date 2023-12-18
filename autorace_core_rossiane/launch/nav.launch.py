import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    my_pkg_path = get_package_share_directory("autorace_core_rossiane")
    pkg_path = get_package_share_directory("robot_bringup")
    nav2_bringup_path = get_package_share_directory("nav2_bringup")

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_path, "launch", "bringup_launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "true",
            "map": os.path.join(my_pkg_path, "map", "map.yaml"),
            "params_file": os.path.join(my_pkg_path, "calibration", "nav_config.yaml"),
        }.items(),
    )

    return LaunchDescription([nav])