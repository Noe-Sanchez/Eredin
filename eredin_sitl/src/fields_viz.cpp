#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class VectorFViz : public rclcpp::Node{
  public:
    VectorFViz(): Node("vectorfviz_node"){
      viz_timer = this->create_wall_timer(50ms, std::bind(&VectorFViz::viz_callback, this));
      viz_publishers = this->create_publisher<visualization_msgs::msg::Marker>("vector_field", 10);

      // Declare vf parameters
      xextent = 0.5;
      yextent = 0.5;
      zextent = 0.5;
      xdivs = 7;
      ydivs = 7;
      zdivs = 7;

      markers.header.frame_id = "world";
      markers.ns = "vector_field";
      markers.id = 0;
      markers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      markers.action = visualization_msgs::msg::Marker::ADD;
      markers.color.a = 0.75;
      markers.color.r = 0.0;
      markers.color.g = 1.0;
      markers.color.b = 0.0;
      markers.pose.orientation.x = 0.0;
      markers.pose.orientation.y = 0.0;
      markers.pose.orientation.z = 0.0;
      markers.pose.orientation.w = 1.0;
      markers.pose.position.x = 0.0;
      markers.pose.position.y = 0.0;
      markers.pose.position.z = 0.0;
      //markers.scale.x = xextent/(xdivs-1);
      //markers.scale.y = xextent/((xdivs-1)*10);
      //markers.scale.z = xextent/((xdivs-1)*10);
      markers.scale.x = 0.02;
      markers.scale.y = 0.02;
      markers.scale.z = 0.02;

      geometry_msgs::msg::Point point;

      D = 0.25;
    }

    void viz_callback(){
      if(c < 20.5){
	c += 0.5;
      }else{
	c = 0;
      }

      markers.header.stamp = this->now();


      for (int j = 0; j < xdivs; j++){
	for (int k = 0; k < ydivs; k++){
	  for (int l = 0; l < zdivs; l++){
	    float xvec, yvec, zvec;
	    
	    xvec = -xextent + 2*j*xextent/(xdivs-1) + c*0.005;
	    yvec = -yextent + 2*k*yextent/(ydivs-1) + c*0.005;
	    zvec = -zextent + 2*l*zextent/(zdivs-1) + c*0.005;

	    Eigen::Vector3d vec;
	
	    vec << (-tanh((5/D)*abs(zvec))+1)*(xvec - yvec), 
		   (-tanh((5/D)*abs(zvec))+1)*(xvec + yvec),
		    zvec;

            geometry_msgs::msg::Point point;

	    point.x = vec(0); 
	    point.y = vec(1);
	    point.z = vec(2);
            markers.points.push_back(point);

	  }
	}
      }
      viz_publishers->publish(markers);
      markers.points.clear();
      
    }

  private:
    float xextent;
    float yextent;
    float zextent;
    float xdivs;
    float ydivs;
    float zdivs;
    float D;
    float d;
    float num_markers;

    // Interp param
    float c;
    visualization_msgs::msg::Marker markers;

    rclcpp::TimerBase::SharedPtr viz_timer; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_publishers;


};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VectorFViz>());
  rclcpp::shutdown();
  return 0;
}
