from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_io_cyphal',
      namespace='l3xz_io_cyphal',
      executable='l3xz_io_cyphal_node',
      name='l3xz_io_cyphal',
      output='screen',
      parameters=[]
    )
  ])
