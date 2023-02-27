from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='ros2_cyphal_bridge',
      namespace='l3xz',
      executable='ros2_cyphal_bridge_node',
      name='ros2_cyphal_bridge',
      output='screen',
      emulate_tty=True,
      parameters=[
        {'can_iface' : 'vcan0'},
      ]
    )
  ])
