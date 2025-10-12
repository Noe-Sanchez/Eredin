#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist, TwistStamped, PoseArray, Pose  
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
import time
import math

class TelloReference(Node):
  def __init__(self) -> None:
    super().__init__('show_pinwheel')

    # Get num_drones from parameter
    self.declare_parameter('num_drones', 1)
    self.num_drones = self.get_parameter('num_drones').value
    
    self.leader_state_publisher    = self.create_publisher(Odometry, '/leader/state', 10)
    self.leader_state              = Odometry()

    self.formation_definition_publisher     = self.create_publisher(PoseArray, '/formation/definition', 10)
    self.formation_dot_definition_publisher = self.create_publisher(PoseArray, '/formation/velocity', 10)
    self.formation_definition               = PoseArray()
    self.formation_dot_definition           = PoseArray()

    self.timer = self.create_timer(0.01, self.timer_callback)
    self.time = 0

    self.follower_pose_list = []
    self.follower_velocity_list = []
    for i in range(self.num_drones):
      self.follower_pose_list.append(Pose())
      self.follower_velocity_list.append(Pose())

  def timer_callback(self) -> None:
    # Leader
    self.leader_state.pose.pose.position.x    = math.cos(self.time/16)
    self.leader_state.pose.pose.position.y    = math.sin(self.time/16)
    self.leader_state.pose.pose.position.z    = 1.0
    self.leader_state.pose.pose.orientation.x = 0.0
    self.leader_state.pose.pose.orientation.y = 0.0
    self.leader_state.pose.pose.orientation.z = math.sin(self.time*math.pi/(32))
    self.leader_state.pose.pose.orientation.w = math.cos(self.time*math.pi/(32))

    self.leader_state.twist.twist.linear.x    = -math.sin(self.time/16)/16
    self.leader_state.twist.twist.linear.y    = math.cos(self.time/16)/16
    self.leader_state.twist.twist.linear.z    = 0.0
    self.leader_state.twist.twist.angular.x   = 0.0
    self.leader_state.twist.twist.angular.y   = 0.0
    self.leader_state.twist.twist.angular.z   = math.pi/16

    self.leader_state.header.stamp            = self.get_clock().now().to_msg()
    self.leader_state.header.frame_id         = 'world'

    self.leader_state_publisher.publish(self.leader_state)

    # Iterate over all drones
    for i in range(self.get_parameter('num_drones').value):
      self.follower_pose_list[i].position.x = math.cos(i*math.pi/2)*(math.sin(self.time/16)/3 + 0.7)
      self.follower_pose_list[i].position.y = math.sin(i*math.pi/2)*(math.sin(self.time/16)/3 + 0.7)
      self.follower_pose_list[i].position.z = 0.0
      self.follower_pose_list[i].orientation.x = 0.0
      self.follower_pose_list[i].orientation.y = 0.0
      self.follower_pose_list[i].orientation.z = 0.0
      self.follower_pose_list[i].orientation.w = 1.0

      self.follower_velocity_list[i].position.x = math.cos(i*math.pi/2)*(math.cos(self.time/16)/48)
      self.follower_velocity_list[i].position.y = math.sin(i*math.pi/2)*(math.cos(self.time/16)/48)
      self.follower_velocity_list[i].position.z = 0.0
      self.follower_velocity_list[i].orientation.x = 0.0
      self.follower_velocity_list[i].orientation.y = 0.0
      self.follower_velocity_list[i].orientation.z = 0.0
      self.follower_velocity_list[i].orientation.w = 0.0 

    self.formation_definition.header.stamp = self.get_clock().now().to_msg()
    self.formation_definition.poses = self.follower_pose_list
    self.formation_definition.header.frame_id = 'leader'

    self.formation_definition_publisher.publish(self.formation_definition)

    self.formation_dot_definition.header.stamp = self.get_clock().now().to_msg()
    self.formation_dot_definition.poses = self.follower_velocity_list
    self.formation_dot_definition.header.frame_id = 'leader'

    self.formation_dot_definition_publisher.publish(self.formation_dot_definition)

    self.time += 0.01


def main(args=None) -> None:
    print('Starting tello reference node...')
    rclpy.init(args=args)
    tello_reference = TelloReference()
    rclpy.spin(tello_reference)
    tello_reference.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
