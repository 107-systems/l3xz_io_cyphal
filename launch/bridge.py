from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_ros_cyphal_bridge',
      namespace='l3xz_ros_cyphal_bridge',
      executable='l3xz_ros_cyphal_bridge_node',
      name='l3xz_ros_cyphal_bridge',
      output='screen',
      parameters=[]
    )
  ])
