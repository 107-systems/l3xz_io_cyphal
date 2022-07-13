from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_io',
      namespace='l3xz_io',
      executable='l3xz_io_node',
      name='l3xz_io',
      output='screen',
    )
  ])
