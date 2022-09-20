#include <chrono>
#include <functional>
#include <memory>

#include <builtin_interfaces/msg/time.hpp>
#include <dasc_msgs/msg/quad_setpoint.hpp>
#include <dasc_msgs/msg/quad_trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "gatekeeper/dynamics.hpp"
#include "gatekeeper/visualization.hpp"

#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/* joystick_controller.cpp
 *
 * this file subscribes to the joystick, and the quadrotors current position.
 * based on the joystick commands, it proposes flying at some speed in the
 * direction commanded by the joysticks.'
 *
 */

using namespace std::chrono_literals;
using namespace std::placeholders;

class JoystickController : public rclcpp::Node {
public:
  JoystickController() : Node("joystick_controller") {

    // rclcpp::Parameter simTime("use_sim_time", rclcpp::ParameterValue(true));
    // this->set_parameter(simTime);

    //traj_pub_ = this->create_publisher<dasc_msgs::msg::QuadTrajectory>(
    //    "/committed_trajectory", 10);
    // "/nominal_trajectory", 10);

    traj_pub_ = this->create_publisher<dasc_msgs::msg::QuadSetpoint>(
     "/target_setpoint", 10);
    
    traj_viz_pub_ =
        this->create_publisher<nav_msgs::msg::Path>("/nominal_traj_viz", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 100, std::bind(&JoystickController::joy_callback, this, _1));

    // initialize the listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
        200ms, std::bind(&JoystickController::timer_callback, this));
  }

private:
  // messages from tf
  double last_x = 0;
  double last_y = 0;
  double last_z = 0;
  double last_yaw = 0;

  double target_x = 0;
  double target_y = 0;
  double target_z = 1.0;
  double target_yaw = 0;

  double vel_x = 0.0;
  double vel_y = 0.0;
  double vel_z = 0.0;
  double vel_t = 0.0;

  // rclcpp
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_{nullptr};
  rclcpp::Publisher<dasc_msgs::msg::QuadSetpoint>::SharedPtr traj_pub_{
      nullptr};
  //rclcpp::Publisher<dasc_msgs::msg::QuadTrajectory>::SharedPtr traj_pub_{
  //    nullptr};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_viz_pub_;

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  builtin_interfaces::msg::Time start_time;
  builtin_interfaces::msg::Time last_tf_stamp;

  geometry_msgs::msg::TransformStamped transformStamped;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

    double cmd_throttle = msg->axes[1];
    // double cmd_yaw = -msg->axes[0];
    double cmd_pitch = msg->axes[3];
    double cmd_roll = msg->axes[2];

    double cmd_yaw = 0.0;
    if (msg->buttons[7] == 1)
      cmd_yaw -= 1.0;
    if (msg->buttons[6] == 1)
      cmd_yaw += 1.0;

    vel_x = cmd_pitch * std::cos(last_yaw) - cmd_roll * std::sin(last_yaw);
    vel_y = cmd_pitch * std::sin(last_yaw) + cmd_roll * std::cos(last_yaw);
    vel_z = cmd_throttle;
    vel_t = 2*cmd_yaw;

    double dt = 0.05;

    target_x += vel_x * dt;
    target_y += vel_y * dt;
    target_z += vel_z * dt;
    target_yaw += vel_t * dt;


    if (msg->buttons[1] == 1) {
      // reset target
      target_x = last_x;
      target_y = last_y;
      target_z = last_z;
      target_yaw = last_yaw;
      RCLCPP_INFO(this->get_logger(), "RESETING");
    }


    if (msg->axes[7] == 1) {target_yaw = 0.0;}
    if (msg->axes[7] == -1) {target_yaw = -M_PI;}
    if (msg->axes[6] == -1) {target_yaw = -0.5*M_PI;}
    if (msg->axes[6] == 1) {target_yaw = 0.5*M_PI;}

    return;
  }

  double saturate(double y, double x, double R) {
    if (y < x - R)
      return (x - R);
    if (y > x + R)
      return (x + R);
    return y;
  }

  void timer_callback() {


    // get the transform
    try {
      transformStamped = tf_buffer_->lookupTransform("world", "vicon/drone6",
                                                     tf2::TimePointZero, 100ms);

      last_x = transformStamped.transform.translation.x;
      last_y = transformStamped.transform.translation.y;
      last_z = transformStamped.transform.translation.z;
      last_yaw = tf2::getYaw(transformStamped.transform.rotation);

    } catch (tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  "drone6", "world", ex.what());
      return;
    }

    // saturate cmds
    target_x = saturate(target_x, last_x, 1.0);
    target_y = saturate(target_y, last_y, 1.0);
    target_z = saturate(target_z, last_z, 1.0);
    target_yaw = dyn::clampToPi(saturate(target_yaw, last_yaw, 1.0 * M_PI));

    dasc_msgs::msg::QuadSetpoint traj{};

    traj.header.stamp = transformStamped.header.stamp;
    traj.header.frame_id = "world";

    traj.x = target_x;
    traj.y = target_y;
    traj.z = target_z;
    traj.yaw = target_yaw;

    // // publish desired trajectory
    RCLCPP_INFO(this->get_logger(), "PUBLISHING! Target: (%f, %f, %f, %f)",
                target_x, target_y, target_z, target_yaw);
    traj_pub_->publish(traj);

    //nav_msgs::msg::Path path = dasc_msgs::msg::toPathMsg(traj);
    //path.header.stamp = traj.header.stamp;
    //path.header.frame_id = traj.header.frame_id;

    //traj_viz_pub_->publish(path);
  }

}; // class JoystickController

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<JoystickController>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
