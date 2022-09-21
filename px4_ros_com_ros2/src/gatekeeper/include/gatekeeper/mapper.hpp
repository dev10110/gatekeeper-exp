#pragma once
#ifndef _MAPPER_
#define _MAPPER_

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


#include <octomap/octomap.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "gatekeeper/dynamics.hpp"
#include "gatekeeper/transforms.hpp"
#include "gatekeeper/visualization.hpp"


using PCLPoint = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTreeT = octomap::OcTree;

namespace ph = std::placeholders;

using namespace std::chrono_literals;

namespace mapper {

class Mapper : public rclcpp::Node {

protected:
  // rclcpp
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      m_pointCloudSub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      m_vehicleLocalPositionSub;

  std::shared_ptr<tf2_ros::Buffer> m_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_pcl_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr free_pcl_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unsafe_pcl_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::TimerBase::SharedPtr unsafe_pub_timer_;

  // octree
  std::shared_ptr<OcTreeT> m_octree;

  // pcl
  pcl::PassThrough<PCLPoint> pass_x, pass_y, pass_z;

  // parameters
  double m_res = 0.05;
  double m_voxel_leaf_size = 0.01;
  int m_treeDepth = 16;
  int m_maxTreeDepth = 16;
  int m_searchDepth = 16;
  double m_maxRange = 5.0;
  double probHit = 0.7;
  double probMiss = 0.4;
  double thresMin = 0.12;
  double thresMax = 0.97;
  double numMax = std::numeric_limits<double>::max();
  double numMin = -std::numeric_limits<double>::max();
  double m_zmin = 0.0;
  double m_zmax = 3.0;
  double m_safety_radius = 0.15;
  int m_keep_every_n = 1;
  std::string m_worldFrameId = "world";
  double m_free_x_min = 0.0;
  double m_free_y_min = 0.0;
  double m_free_z_min = 0.0;
  double m_free_x_max = 0.0;
  double m_free_y_max = 0.0;
  double m_free_z_max = 0.0;
  double m_publish_radius = 1.0;
  int m_publish_occupied_every = 500;
  int m_publish_unsafe_every = 500;

  // storage
  rclcpp::Time last_pos_t;
  dyn::State last_quad_state{};
  dyn::State target{};
  bool received_target = false;
  uint8_t drop_msg = 0;

  // functions
  void insertScan(const geometry_msgs::msg::Vector3 &sensorOrigin,
                  const PCLPointCloud &pc);

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void localPosition_callback(
      const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  bool isUnsafe(double x, double y, double z);
  bool isUnsafe(octomap::OcTreeKey);

  void publish_occupied_pcl();
  void publish_free_pcl();
  void publish_unsafe_pcl(const geometry_msgs::msg::Vector3 &origin);
  void assume_free_space();


public:
  Mapper();
};

} // namespace mapper

#endif // _MAPPER_
