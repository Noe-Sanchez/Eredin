import argparse
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  num_drones = int(sys.argv[4:][0].split('=')[1])
  print(f"Launching simulation with {num_drones} drones.")
  
  nodes = []

  for i in range(num_drones):
    control_node = Node(
      package="eredin_sitl",
      executable="control_node",
      name=f"control_node_{i+1}",
      output="screen",
      remappings=[("/control_1/reference/pose",   f"/control_{i+1}/reference/pose"),
                  ("/model/x500_1/odometry",      f"/model/x500_{i+1}/odometry"),
                  ("/x500_1/command/motor_speed", f"/x500_{i+1}/command/motor_speed"),],
    )
    nodes.append(control_node)

  ros_gz_bridge_node = Node(package="ros_gz_bridge",
                            executable="parameter_bridge",
                            name="ros_gz_bridge",
                            output="screen",
                            parameters=[{"config_file": os.path.join(get_package_share_directory("eredin_sitl"), "config", "bridge_config.yaml")}],
                           )

  nodes.append(ros_gz_bridge_node)

  return LaunchDescription(nodes)

if __name__ == "__main__":
  generate_launch_description()

