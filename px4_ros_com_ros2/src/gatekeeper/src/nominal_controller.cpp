#include <chrono>
#include <functional>
#include <memory>

#include <builtin_interfaces/msg/time.hpp>
#include <dasc_msgs/msg/quad_trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "gatekeeper/visualization.hpp"

/* nominal_controller.cpp
 *
 * this file publishes one particular trajectory, for debugging.
 *
 */

using namespace std::chrono_literals;
using namespace std::placeholders;

class NominalController : public rclcpp::Node {
public:
  NominalController() : Node("nominal_controller") {

    start_time = this->get_clock()->now();

    traj_pub_ = this->create_publisher<dasc_msgs::msg::QuadTrajectory>(
        //"/committed_trajectory", 10);
	"/nominal_trajectory", 10);

    timer_ = this->create_wall_timer(
        500ms, std::bind(&NominalController::timer_callback, this));

    path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>("nominal_traj_viz", 10);

    publish_one_trajectory();
  }

private:
  // messages from tf
  double last_x = 0;
  double last_y = 0;
  double last_z = 0;
  double last_yaw = 0;

  // targets

  double target_x = 0.0;
  double target_y = 0.0;
  double target_z = 1.0;
  double target_yaw = 0.0;

  // rclcpp
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<dasc_msgs::msg::QuadTrajectory>::SharedPtr traj_pub_{
      nullptr};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_{nullptr};

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  builtin_interfaces::msg::Time last_tf_stamp;
  builtin_interfaces::msg::Time start_time;

  double clampToPi(double yaw) {
    return std::atan2(std::sin(yaw), std::cos(yaw));
    return yaw;
  }


  void append_move(dasc_msgs::msg::QuadTrajectory & msg, 
		  double start_x, double start_y, double start_z, double start_yaw,
		  double end_x,   double end_y,   double end_z,   double end_yaw,
		  double start_t, double end_t, double dt){
	  
	  int N = int(double(end_t - start_t) / dt);

	  for (int i=0; i <= N; i++){

		  double f = double(i) / double(N);

		  msg.ts.push_back( (1-f) * start_t + f * end_t);
		  msg.xs.push_back( (1-f) * start_x + f * end_x);
		  msg.ys.push_back( (1-f) * start_y + f * end_y);
		  msg.zs.push_back( (1-f) * start_z + f * end_z);
		  msg.yaws.push_back( (1-f) * start_yaw + f * end_yaw);
		  msg.vxs.push_back( 0.0);
		  msg.vys.push_back( 0.0);
		  msg.vzs.push_back( 0.0);
		  msg.axs.push_back( 0.0);
		  msg.ays.push_back( 0.0);
		  msg.azs.push_back( 0.0);

	  }

  }




  void publish_one_trajectory() {

    // construct the desired trajectory
    dasc_msgs::msg::QuadTrajectory msg = dasc_msgs::msg::QuadTrajectory();

    msg.header.stamp = start_time;
    msg.header.frame_id = "world";

    std::vector<double> key_t;
    std::vector<double> key_x;
    std::vector<double> key_y;
    std::vector<double> key_z;
    std::vector<double> key_yaw;

    // start
    key_t.push_back(0.0);
    key_x.push_back(-1.0);
    key_y.push_back(0.0);
    key_z.push_back(1.0);
    key_yaw.push_back(0.0);

    // // scan 360
    // key_t.push_back(key_t.back() + 5.0);
    // key_x.push_back(key_x.back());
    // key_y.push_back(key_y.back());
    // key_z.push_back(key_z.back());
    // key_yaw.push_back(M_PI/2);

    // key_t.push_back(key_t.back() + 5.0);
    // key_x.push_back(key_x.back());
    // key_y.push_back(key_y.back());
    // key_z.push_back(key_z.back());
    // key_yaw.push_back(M_PI);

    // key_t.push_back(key_t.back() + 5.0);
    // key_x.push_back(key_x.back());
    // key_y.push_back(key_y.back());
    // key_z.push_back(key_z.back());
    // key_yaw.push_back(M_PI*1.5);

    // key_t.push_back(key_t.back() + 5.0);
    // key_x.push_back(key_x.back());
    // key_y.push_back(key_y.back());
    // key_z.push_back(key_z.back());
    // key_yaw.push_back(0.0);

    // move forward 
    key_t.push_back(key_t.back() + 15.0);
    key_x.push_back(1.0);
    key_y.push_back(key_y.back());
    key_z.push_back(key_z.back());
    key_yaw.push_back(0.0);
    
    // move back 
    key_t.push_back(key_t.back() + 15.0);
    key_x.push_back(0.0);
    key_y.push_back(key_y.back());
    key_z.push_back(key_z.back());
    key_yaw.push_back(0.0);
    
    
    // // move down 
    // key_t.push_back(key_t.back() + 5.0);
    // key_x.push_back(key_x.back());
    // key_y.push_back(key_y.back());
    // key_z.push_back(0.5);
    // key_yaw.push_back(0.0);
    // 
    // 
    // // move forward 
    // key_t.push_back(key_t.back() + 15.0);
    // key_x.push_back(1.0);
    // key_y.push_back(key_y.back());
    // key_z.push_back(key_z.back());
    // key_yaw.push_back(0.0);




    // construct trajectory!
    for (size_t i =0; i < key_x.size() -1; i++){
	    append_move(msg, key_x[i], key_y[i], key_z[i], key_yaw[i],
			    key_x[i+1], key_y[i+1], key_z[i+1], key_yaw[i+1],
			    key_t[i], key_t[i+1], 0.2);
    }


    send(msg);

   //  double dt = 0.5;
   //  double T = 15.0;

   //  double t = 0.0;
   //  double x = 0.0;
   //  double vx = 0.5;

   //  // first T seconds spin around
   //  while (true) {
   //    double th = 2.0 * M_PI * t / T;

   //    msg.ts.push_back(t);
   //    msg.xs.push_back(0.0); // R * std::cos(th) - R);
   //    msg.ys.push_back(0.0); // R * std::sin(th) );
   //    msg.zs.push_back(1.0);
   //    msg.vxs.push_back(0.0);
   //    msg.vys.push_back(0.0);
   //    msg.vzs.push_back(0.0);
   //    msg.yaws.push_back(th + 0.5 * M_PI * (t / T));

   //    t += dt;
   //    x += vx * dt;
   //    if (t > T)
   //      break;
   //  }

   //  // next T seconds go forward
   //  while (true) {
   //    double f = (t - T) / T;

   //    msg.ts.push_back(t);
   //    msg.xs.push_back(0.0);     // R * std::cos(th) - R);
   //    msg.ys.push_back(4.0 * f); // R * std::sin(th) );
   //    msg.zs.push_back(0.5);
   //    msg.vxs.push_back(0.0);
   //    msg.vys.push_back(0.0);
   //    msg.vzs.push_back(0.0);
   //    msg.yaws.push_back(M_PI * 0.5);

   //    t += dt;
   //    x += vx * dt;

   //    if (f > 1)
   //      break;
   //  }
   //  // next 5 seconds go forward
   //  while (true) {
   //    double f = (t - 2 * T) / T;

   //    msg.ts.push_back(t);
   //    msg.xs.push_back(0.0);           // R * std::cos(th) - R);
   //    msg.ys.push_back(4 * (1.0 - f)); // R * std::sin(th) );
   //    msg.zs.push_back(0.5);
   //    msg.vxs.push_back(0.0);
   //    msg.vys.push_back(0.0);
   //    msg.vzs.push_back(0.0);
   //    msg.yaws.push_back(M_PI * 0.5);

   //    t += dt;
   //    x += vx * dt;
   //    if (f > 1)
   //      break;
   //  }

   //  send(msg);
  }

  void send(dasc_msgs::msg::QuadTrajectory msg) {

    traj_pub_->publish(msg);

    nav_msgs::msg::Path path_msg = dasc_msgs::msg::toPathMsg(msg);

    path_pub_->publish(path_msg);

    RCLCPP_INFO(this->get_logger(), "PUBLISHED");
  }

  void timer_callback() {
    publish_one_trajectory();
    return;
  }

}; // class NominalController

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<NominalController>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
