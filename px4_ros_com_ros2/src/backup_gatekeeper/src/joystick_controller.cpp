#include <chrono>
#include <functional>
#include <memory>

#include <builtin_interfaces/msg/time.hpp>
#include <dasc_msgs/msg/quad_trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "gatekeeper/visualization.hpp"

#include <tf2/exceptions.h>
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
  JoystickController() : Node("nominal_controller") {

    start_time = this->get_clock()->now();

    traj_pub_ = this->create_publisher<dasc_msgs::msg::QuadTrajectory>(
        "/nominal_trajectory", 10);

    traj_viz_pub_ =
        this->create_publisher<nav_msgs::msg::Path>("/nomimal_traj_viz", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoystickController::joy_callback, this, _1));

    // initialize the listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&JoystickController::timer_callback, this));
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
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_{nullptr};
  rclcpp::Publisher<dasc_msgs::msg::QuadTrajectory>::SharedPtr traj_pub_{
      nullptr};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_viz_pub_;

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  builtin_interfaces::msg::Time start_time;
  builtin_interfaces::msg::Time last_tf_stamp;

  double clampToPi(double yaw) {
    return std::atan2(std::sin(yaw), std::cos(yaw));
    return yaw;
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

    // get the transform
    try {
      auto transformStamped =
          tf_buffer_->lookupTransform("world", "drone1", tf2::TimePointZero);

      last_x = transformStamped.transform.translation.x;
      last_y = transformStamped.transform.translation.y;
      last_z = transformStamped.transform.translation.z;
      last_tf_stamp = transformStamped.header.stamp;

    } catch (tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  "drone1", "world", ex.what());
      return;
    }

    double cmd_throttle = -msg->axes[0];
    double cmd_yaw = msg->axes[2];
    double cmd_pitch = -msg->axes[1];
    double cmd_roll = msg->axes[3];

    target_x += cmd_pitch / 5.0;
    target_y += cmd_roll / 5.0;
    target_z += cmd_throttle / 5.0;
    target_yaw += cmd_yaw / 5.0;

    // check that we are not too far
    target_x = std::max(last_x - 1.0, std::min(target_x, last_x + 1.0));
    target_y = std::max(last_y - 1.0, std::min(target_y, last_y + 1.0));
    target_z = std::max(last_z - 1.0, std::min(target_z, last_z + 1.0));
    target_yaw = clampToPi(target_yaw);

    RCLCPP_INFO(this->get_logger(), "TARGET %f, %f, %f, %f", target_x, target_y,
                target_z, target_yaw);
  }

  void timer_callback() {

    // construct the desired trajectory
    dasc_msgs::msg::QuadTrajectory msg = dasc_msgs::msg::QuadTrajectory();

    msg.header.stamp = last_tf_stamp;
    msg.header.frame_id = "world";

    double T = 1.0; // horizon

    for (int i = 0; i < 20; i++) {

      double f = (double)i / 20.0;

      msg.ts.push_back(f * T);
      msg.xs.push_back((1.0 - f) * last_x + f * target_x);
      msg.ys.push_back((1.0 - f) * last_y + f * target_y);
      msg.zs.push_back((1.0 - f) * last_z + f * target_z);
      msg.yaws.push_back((1.0 - f) * last_yaw + f * target_yaw);
      msg.vxs.push_back(0.0);
      msg.vys.push_back(0.0);
      msg.vzs.push_back(0.0);
    }

    // publish desired trajectory
    traj_pub_->publish(msg);

    nav_msgs::msg::Path path = dasc_msgs::msg::toPathMsg(msg);

    traj_viz_pub_->publish(path);
  }

}; // class JoystickController

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<JoystickController>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
