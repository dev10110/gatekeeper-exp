#pragma once
#ifndef _GATEKEEPER_
#define _GATEKEEPER_

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <dasc_msgs/msg/quad_trajectory.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


//#include <pcl/common/distances.h>
//#include <pcl/common/transforms.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "gatekeeper/dynamics.hpp"
#include "gatekeeper/transforms.hpp"
#include "gatekeeper/visualization.hpp"

using PCLPoint = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;

namespace ph = std::placeholders;

using namespace std::chrono_literals;

namespace gatekeeper {

class Gatekeeper : public rclcpp::Node {

protected:
  // rclcpp
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      m_pointCloudSub;
  rclcpp::Subscription<dasc_msgs::msg::QuadTrajectory>::SharedPtr
      m_nominalTrajSub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      m_vehicleLocalPositionSub;

  rclcpp::Publisher<dasc_msgs::msg::QuadTrajectory>::SharedPtr
      m_committedTrajPub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_committedTrajVizPub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_extTrajVizPub;

  std::shared_ptr<tf2_ros::Buffer> m_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  rclcpp::TimerBase::SharedPtr traj_timer_;

  // parameters
  double m_safety_radius_xy = 0.35;
  double m_safety_radius_z = 0.15;
  std::string m_worldFrameId = "world";


  // storage
  PCLPointCloud pc;
  pcl::KdTreeFLANN<PCLPoint> kdtree;
  rclcpp::Time last_pos_t;
  dyn::State last_quad_state{};

  // functions
  void
  nominalTraj_callback(const dasc_msgs::msg::QuadTrajectory::SharedPtr msg);

  void traj_timer_callback();
  
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void localPosition_callback(
      const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  bool isSafe(double x, double y, double z);
  bool isSafe(double x, double y, double z, double r);
  bool isSafe(dyn::Trajectory P);

  void publish_committed_traj(const dasc_msgs::msg::QuadTrajectory msg);
  void publish_extended_traj(const dasc_msgs::msg::QuadTrajectory msg);

public:
  Gatekeeper();
};

} // namespace gatekeeper

#endif // _GATEKEEPER_
