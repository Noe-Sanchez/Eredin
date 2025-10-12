#include <chrono>
#include <iostream>
#include <math.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class OdomFormations : public rclcpp::Node{
  public:
    OdomFormations(): Node("odom_formations_node"){
      // Drone parameters
      num_drones = this->declare_parameter("num_drones", 3);
      //follower_tfs.resize(num_drones);

      // Subscribers
      desired_formation_subscriber   = this->create_subscription<geometry_msgs::msg::PoseArray>("/formation/defintion", 10, std::bind(&OdomFormations::desired_formation_callback, this, std::placeholders::_1));
      formation_dot_subscriber   = this->create_subscription<geometry_msgs::msg::PoseArray>("/formation/velocity", 10, std::bind(&OdomFormations::formation_dot_callback, this, std::placeholders::_1));
      desired_leader_pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/leader/pose", 10, std::bind(&OdomFormations::desired_leader_pose_callback, this, std::placeholders::_1));

      // Timer 
      control_timer = this->create_wall_timer(20ms, std::bind(&OdomFormations::control_callback, this));

      // Dynamic odom publishers
      for (int i = 0; i < num_drones; i++){
	std::string topic_name = "/control_" + std::to_string(i+1) + "/reference/pose";
        follower_odom_publishers.push_back(this->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10));
        follower_odom_msgs.push_back(nav_msgs::msg::Odometry());
        follower_tfs.push_back(geometry_msgs::msg::TransformStamped());

      }

      tf_broadcaster  = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    void desired_formation_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
      for (int i = 0; i < num_drones; i++){
        follower_tfs[i].header.frame_id = "leader";
	follower_tfs[i].child_frame_id = "follower_" + std::to_string(i+1);
	follower_tfs[i].transform.translation.x = msg->poses[i].position.x;
	follower_tfs[i].transform.translation.y = msg->poses[i].position.y;
	follower_tfs[i].transform.translation.z = msg->poses[i].position.z;
	follower_tfs[i].transform.rotation = msg->poses[i].orientation;
      }
    }

    void desired_leader_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      leader_tf.header = msg->header;
      leader_tf.child_frame_id = "leader";
      leader_tf.transform.translation.x = msg->pose.pose.position.x;
      leader_tf.transform.translation.y = msg->pose.pose.position.y;
      leader_tf.transform.translation.z = msg->pose.pose.position.z;
      leader_tf.transform.rotation = msg->pose.pose.orientation;
    }

    void control_callback(){
      // Publish leader tf first
      leader_tf.header.stamp = this->now();
      leader_tf.child_frame_id = "leader";
      leader_tf.header.frame_id = "world";
      tf_broadcaster->sendTransform(leader_tf);

      for (int i = 0; i < num_drones; i++){
	// Update header
	follower_tfs[i].header.stamp = this->now();

        // Compute follower ref in world frame	
	mock_tf.header = follower_tfs[i].header;
	mock_tf.child_frame_id = follower_tfs[i].child_frame_id;

	tf2::doTransform(follower_tfs[i], mock_tf, leader_tf);

	// Publish follower tf
	tf_broadcaster->sendTransform(mock_tf);

	// Populate odom msgs
	follower_odom_msgs[i].header = mock_tf.header;
	follower_odom_msgs[i].child_frame_id = mock_tf.child_frame_id;
	follower_odom_msgs[i].pose.pose.position.x = mock_tf.transform.translation.x;
	follower_odom_msgs[i].pose.pose.position.y = mock_tf.transform.translation.y;
	follower_odom_msgs[i].pose.pose.position.z = mock_tf.transform.translation.z;
        follower_odom_msgs[i].pose.pose.orientation = mock_tf.transform.rotation;

	// Publish odom message
	follower_odom_publishers[i]->publish(follower_odom_msgs[i]);
      }
    }

  private:
    float num_drones;

    geometry_msgs::msg::TransformStamped leader_tf; 
    geometry_msgs::msg::TransformStamped mock_tf;

    std::vector<geometry_msgs::msg::TransformStamped> follower_tfs;
    std::vector<nav_msgs::msg::Odometry> follower_odom_msgs;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr     desired_formation_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           desired_leader_pose_subscriber;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> follower_odom_publishers;
    rclcpp::TimerBase::SharedPtr control_timer;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFormations>());
  rclcpp::shutdown();
  return 0;
}
