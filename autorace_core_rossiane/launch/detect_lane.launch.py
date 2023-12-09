import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('autorace_core_rossiane'),
      'calibration',
      'calibration.yaml'
      )

   return LaunchDescription([
      Node(
         package='autorace_core_rossiane',
         executable='path_tracking',
         namespace='detect',
         name='lane',
         parameters=[config]
      )
   ])