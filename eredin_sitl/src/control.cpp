#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "actuator_msgs/msg/actuators.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class EController : public rclcpp::Node{
  public:
    EController(): Node("control_node"){

      // Subscribers
      sim_pose_subscriber     = this->create_subscription<nav_msgs::msg::Odometry>("/sim/pose", 10, std::bind(&EController::sim_pose_callback, this, std::placeholders::_1));
      desired_pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/control/reference/pose", 10, std::bind(&EController::desired_pose_callback, this, std::placeholders::_1));

      // Publishers
      motor_publisher = this->create_publisher<actuator_msgs::msg::Actuators>("/sim/motor_speed", 10);

      // For viz
      qud_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/control/qud", 10);
      local_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/control/local_pose", 10);

      control_timer = this->create_wall_timer(10ms, std::bind(&EController::control_callback, this));

      m = 2.0;

      // x500 values for now
      Jxx = 0.021666;
      Jyy = 0.021666;
      Jzz = 0.040000;

      J << Jxx, 0,  0,
            0, Jyy, 0,
            0, 0, Jzz;

      kT = 8.54858e-6;
      //kQ = 1.6e-2;
      //kQ = 8.06428-5;
      kQ = 8064000000.28;
      l  = 0.25;

      kp_lin << 0.5, 0.5, 16.5;
      kd_lin << 0.5, 0.5, 5.5;
      kp_ang << 10.15, 10.15, 40.5;
      kd_ang << 5.0, 5.0, 15.0;
      //kd_ang << 0.0, 0.0, 0.0;

      e_lin         << 0.0, 0.0, 0.0;
      e_dot_lin     << 0.0, 0.0, 0.0;
      sim_pos       << 0.0, 0.0, 0.0;
      desired_pos   << 0.0, 0.0, 1.0;
      sim_vel       << 0.0, 0.0, 0.0;
      desired_vel   << 0.0, 0.0, 0.0;
      sim_omega     << 0.0, 0.0, 0.0;
      desired_omega << 0.0, 0.0, 0.0;
      u_lin         << 0.0, 0.0, 0.0;
      u_ang         << 0.0, 0.0, 0.0;
      fu            << 0.0, 0.0, 0.0;
      ft            << 0.0, 0.0, 1.0;
      uaux_lin      << 0.0, 0.0, 0.0;
      uaux_ang      << 0.0, 0.0, 0.0;
      g_vector      << 0.0, 0.0, m*9.81;
      e_ang         << 0.0, 0.0, 0.0;
      e_dot_ang     << 0.0, 0.0, 0.0;
      flat_outputs  << 0.0, 0.0, 0.0, 0.0;
      motor_speeds  << 0.0, 0.0, 0.0, 0.0;

      sim_quat     = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      desired_quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      u_lin_q      = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
      qud          = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      qe           = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

      // Actuation matrix
      double dd = l/sqrt(2);

      actuation <<  kT, kT, kT, kT,
      		    -dd*kT, dd*kT,  dd*kT, -dd*kT,
      		     //dd*kT, -dd*kT,  dd*kT,-dd*kT,
      		     -dd*kT, dd*kT, -dd*kT, dd*kT,
      		     //-kQ,    -kQ,   -kQ,   -kQ;
      		     //kT,    kT,   -kT,   -kT;
      		     -kT,    -kT,   kT,   kT;
      		     //0, 0, 0, 0;

      motor_speed.velocity.resize(4);
    }

    void sim_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      sim_pose = *msg;

      sim_pos <<  sim_pose.pose.pose.position.x,
                  sim_pose.pose.pose.position.y,
                  sim_pose.pose.pose.position.z;

      sim_vel <<  sim_pose.twist.twist.linear.x,
                  sim_pose.twist.twist.linear.y,
                  sim_pose.twist.twist.linear.z;

      sim_quat.w() =  sim_pose.pose.pose.orientation.w;
      sim_quat.x() =  sim_pose.pose.pose.orientation.x;
      sim_quat.y() =  sim_pose.pose.pose.orientation.y;
      sim_quat.z() =  sim_pose.pose.pose.orientation.z;

      sim_omega <<  sim_pose.twist.twist.angular.x,
	            sim_pose.twist.twist.angular.y,
		    sim_pose.twist.twist.angular.z;
    }

    void desired_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      desired_pose = *msg;

      desired_pos << desired_pose.pose.pose.position.x,
                     desired_pose.pose.pose.position.y,
                     desired_pose.pose.pose.position.z;

      desired_vel << desired_pose.twist.twist.linear.x,
                     desired_pose.twist.twist.linear.y,
                     desired_pose.twist.twist.linear.z;

      desired_quat.w() = desired_pose.pose.pose.orientation.w;
      desired_quat.x() = desired_pose.pose.pose.orientation.x;
      desired_quat.y() = desired_pose.pose.pose.orientation.y;
      desired_quat.z() = desired_pose.pose.pose.orientation.z;

      desired_omega << desired_pose.twist.twist.angular.x,
		       desired_pose.twist.twist.angular.y,
		       desired_pose.twist.twist.angular.z;
    }

    void control_callback(){
      // Compute errors
      e_lin     = desired_pos - sim_pos;
      e_dot_lin = desired_vel - sim_vel;

      // Compute linear control (angular control needs fu)
      uaux_lin = kp_lin.cwiseProduct(e_lin) + kd_lin.cwiseProduct(e_dot_lin);

      // Rotate control
      u_lin = uaux_lin;

      u_lin_q.w() = 0.0;
      u_lin_q.x() = u_lin(0);
      u_lin_q.y() = u_lin(1);
      u_lin_q.z() = u_lin(2);
      u_lin_q_rot = sim_quat.conjugate() * u_lin_q * sim_quat;

      // Desired forces
      fu(0) = -m * u_lin_q_rot.x();
      fu(1) = -m * u_lin_q_rot.y();
      fu(2) = m * u_lin_q_rot.z(); // Add gravity compensation

      // Olivas Tesis 2.51
      if ( abs(fu.normalized().dot(ft)) == 1.0 ) { 
        qud.w() = 1.0;
      } else {
	qud.w() = sqrt((1 + (fu.normalized()).dot(ft))/2.0);
      }
      if (fu.normalized().cross(ft).norm() < 0.01) {
	qud.x() = 0.0;
	qud.y() = 0.0;
	qud.z() = 0.0;
      } else {
	Eigen::Vector3d axis = (((fu.normalized()).cross(ft)).normalized())*sqrt((1 - (fu.normalized()).dot(ft))/2.0);
        qud.x() = axis(0);	
	qud.y() = axis(1);
	qud.z() = axis(2);
      }
      qud = qud * desired_quat;
      qud.normalize();

      // Publish local pose for visualization
      geometry_msgs::msg::PoseStamped local_pose_msg;
      local_pose_msg.header.stamp = this->get_clock()->now();
      local_pose_msg.header.frame_id = "map";
      local_pose_msg.pose.orientation.w = sim_quat.w();
      local_pose_msg.pose.orientation.x = sim_quat.x();
      local_pose_msg.pose.orientation.y = sim_quat.y();
      local_pose_msg.pose.orientation.z = sim_quat.z();
      local_pose_publisher->publish(local_pose_msg);

      // Compute logarithmic mapping
      qe = sim_quat.inverse() * qud;
      qe.normalize();

      // Publish qud for visualization
      geometry_msgs::msg::PoseStamped qud_msg;
      qud_msg.header.stamp = this->get_clock()->now();
      qud_msg.header.frame_id = "map";
      qud_msg.pose.orientation.w = qe.w();
      qud_msg.pose.orientation.x = qe.x();
      qud_msg.pose.orientation.y = qe.y();
      qud_msg.pose.orientation.z = qe.z();
      qud_publisher->publish(qud_msg);

      // Sanity check qe for mag = 0
      double norm = sqrt(qe.x()*qe.x() + qe.y()*qe.y() + qe.z()*qe.z());

      if (norm < 0.01) {
	e_ang = Eigen::Vector3d(0.0, 0.0, 0.0);
      } else {
	e_ang = 2 * Eigen::Vector3d(qe.x(), qe.y(), qe.z()).normalized() * (acos(qe.w()));
      }

      // Compute angular error derivative
      e_dot_ang = desired_omega - sim_omega;

      // Compute angular control
      uaux_ang = kp_ang.cwiseProduct(e_ang) + kd_ang.cwiseProduct(e_dot_ang);

      // Olivas ICUAS23 (24)
      u_ang = J * uaux_ang + (sim_omega.cross(J * sim_omega));

      // Castañeda ICUAS17 (39)
      flat_outputs << fu(2), u_ang(0), u_ang(1), u_ang(2);
      // Pseudo inverse for custom allo
      motor_speeds = actuation.completeOrthogonalDecomposition().pseudoInverse() * flat_outputs;
      
      //std::cout << "Motor speeds (squared): " << motor_speeds.transpose() << std::endl;
      
      motor_speeds = motor_speeds.cwiseSqrt();

      // Publish motor speeds
      motor_speed.header.stamp = this->get_clock()->now();
      motor_speed.header.frame_id = "sim/motor_speed";
      motor_speed.velocity[0] = std::max(0.0, std::min(2000.0, motor_speeds(0)));
      motor_speed.velocity[1] = std::max(0.0, std::min(2000.0, motor_speeds(1)));
      motor_speed.velocity[2] = std::max(0.0, std::min(2000.0, motor_speeds(2)));
      motor_speed.velocity[3] = std::max(0.0, std::min(2000.0, motor_speeds(3)));
      motor_publisher->publish(motor_speed);

      // Print control outputs for now
      //std::cout << "E_lin: " << e_lin.transpose() << " | E_dot_lin: " << e_dot_lin.transpose() << std::endl;
      std::cout << "Fu: " << fu.transpose() << " | Tau: " << u_ang.transpose() << std::endl;
      //std::cout << "E_ang: " << e_ang.transpose() << " | E_dot_ang: " << e_dot_ang.transpose() << std::endl;

    }

  private:

    nav_msgs::msg::Odometry sim_pose;
    nav_msgs::msg::Odometry desired_pose;
    actuator_msgs::msg::Actuators motor_speed;

    Eigen::Matrix3d    J;             // Inertia tensor, kg m^2
    Eigen::Vector3d    e_lin;         // Linear error
    Eigen::Vector3d    e_dot_lin;     // Linear error derivative
    Eigen::Vector3d    e_ang;         // Angular error
    Eigen::Vector3d    e_dot_ang;     // Angular error derivative
    Eigen::Vector3d    sim_pos;       // Simulated position
    Eigen::Vector3d    desired_pos;   // Desired position
    Eigen::Vector3d    sim_vel;       // Simulated velocity
    Eigen::Vector3d    desired_vel;   // Desired velocity
    Eigen::Vector3d    sim_omega;     // Simulated angular velocity
    Eigen::Vector3d    desired_omega; // Desired angular velocity
    Eigen::Vector3d    u_lin;         // Linear control output
    Eigen::Vector3d    u_ang;         // Angular control output
    Eigen::Vector3d    uaux_lin;      // Linear auxiliary control output
    Eigen::Vector3d    uaux_ang;      // Angular auxiliary control output
    Eigen::Vector3d    fu;            // Desired forces vector
    Eigen::Vector3d    ft;            // Thrust vector
    Eigen::Vector3d    kp_lin;        // Linear p gains
    Eigen::Vector3d    kd_lin;        // Linear d gains
    Eigen::Vector3d    kp_ang;        // Angular p gains
    Eigen::Vector3d    kd_ang;        // Angular d gains
    Eigen::Vector3d    g_vector;      // Gravity vector
    Eigen::Quaterniond sim_quat;     // Simulated quaternion
    Eigen::Quaterniond desired_quat; // Desired quaternion (only for yaw input)
    Eigen::Quaterniond u_lin_q;      // Linear control output (quaternion)
    Eigen::Quaterniond u_lin_q_rot;  // Rotated linear control output (quaternion)
    Eigen::Quaterniond qud;          // Internal quaternion for logarithmic mapping
    Eigen::Quaterniond qe;           // Quaternion error for logarithmic mapping
    Eigen::Matrix4d    actuation;    // Quadrotor actuation matrix
    Eigen::Vector4d    flat_outputs; // Overall flat outputs
    Eigen::Vector4d    motor_speeds; // Motor speeds for publishing

    float m;           // Mass, kg
    float Jxx;         // X-axis inertia
    float Jyy;         // Y-axis inertia
    float Jzz;         // Z-axis inertia
    float kT;          // Thrust coefficient
    float kQ;          // Torque coefficient
    float l;           // Rotor arm length, m

    rclcpp::TimerBase::SharedPtr control_timer;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sim_pose_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr desired_pose_subscriber;

    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_publisher;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr qud_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EController>());
  rclcpp::shutdown();
  return 0;
}
