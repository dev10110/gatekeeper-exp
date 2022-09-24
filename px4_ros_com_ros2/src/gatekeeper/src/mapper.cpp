/* mapper.cpp
 *
 * this file listens to sensor messages,
 * builds an octomap of the world
 * and publishes the occupancy info
 */

#include "gatekeeper/mapper.hpp"

namespace mapper {

Mapper::Mapper() : Node("mapper") {

  // rclcpp::Parameter simTime("use_sim_time", rclcpp::ParameterValue(true));
  // this->set_parameter(simTime);

  // parameters
  this->declare_parameter<double>("map/m_res", 0.05);
  this->declare_parameter<double>("map/m_zmin", -0.25);
  this->declare_parameter<double>("map/m_zmax", 3.0);
  this->declare_parameter<double>("map/leaf_size", 0.01);
  this->declare_parameter<int>("map/keep_every_n", 1);
  this->declare_parameter<double>("map/free_x_min", -1.0);
  this->declare_parameter<double>("map/free_y_min", -1.0);
  this->declare_parameter<double>("map/free_z_min", 0);
  this->declare_parameter<double>("map/free_x_max", 1.0);
  this->declare_parameter<double>("map/free_y_max", 1.0);
  this->declare_parameter<double>("map/free_z_max", 2.0);
  this->declare_parameter<int>("map/publish_occupied_every", 500);
  this->declare_parameter<int>("map/publish_unsafe_every", 500);
  this->declare_parameter<double>("map/publish_radius", 2.0);

  this->get_parameter("map/m_res", m_res);
  this->get_parameter("map/m_zmin", m_zmin);
  this->get_parameter("map/m_zmax", m_zmax);
  this->get_parameter("map/leaf_size", m_voxel_leaf_size);
  this->get_parameter("map/keep_every_n", m_keep_every_n);
  this->get_parameter("map/free_x_min", m_free_x_min);
  this->get_parameter("map/free_y_min", m_free_y_min);
  this->get_parameter("map/free_z_min", m_free_z_min);
  this->get_parameter("map/free_x_max", m_free_x_max);
  this->get_parameter("map/free_y_max", m_free_y_max);
  this->get_parameter("map/free_z_max", m_free_z_max);
  this->get_parameter("map/publish_radius", m_publish_radius);
  this->get_parameter("map/publish_occupied_every_ms",
                      m_publish_occupied_every);
  this->get_parameter("map/publish_unsafe_every_ms", m_publish_unsafe_every);

  // initialize pubs and subs

  m_pointCloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/color/points", 1,
      std::bind(&Mapper::cloud_callback, this, ph::_1));

  m_vehicleLocalPositionSub =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/drone4/fmu/vehicle_local_position/out", 1,
          std::bind(&Mapper::localPosition_callback, this, ph::_1));

  m_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

  m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_buffer_);

  occupied_pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/occupied_octomap", 1);

  free_pcl_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/free_octomap", 1);

  unsafe_pcl_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/unsafe_pcl", 1);

  pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(m_publish_occupied_every),
      std::bind(&Mapper::publish_occupied_pcl, this));

  // unsafe_pub_timer_ = this->create_wall_timer(
  //    std::chrono::milliseconds(m_publish_unsafe_every),
  //    std::bind(&Mapper::publish_unsafe_pcl, this));

  // initialize octomap
  m_octree = std::make_shared<OcTreeT>(m_res);
  m_octree->setProbHit(probHit);
  m_octree->setProbMiss(probMiss);
  m_octree->setClampingThresMin(thresMin);
  m_octree->setClampingThresMax(thresMax);

  m_searchDepth = 0;
  // for (int i = 0; i < 17; i++) {
  //   double s = m_octree->getNodeSize(i);
  //   if (s > 0.10) {
  //     m_searchDepth = i;
  //   } else {
  //     break;
  //   }
  // }

  // preinitialize the set of free space
  assume_free_space();

  // initialize filters
  pass_x.setFilterFieldName("x");
  pass_y.setFilterFieldName("y");
  pass_z.setFilterFieldName("z");

  pass_x.setFilterLimits(numMin, numMax);
  pass_y.setFilterLimits(numMin, numMax);
  pass_z.setFilterLimits(m_zmin, m_zmax);

  drop_msg = 0;
}

void Mapper::assume_free_space() {

  RCLCPP_INFO(this->get_logger(), "here");
  for (double x = m_free_x_min; x <= m_free_x_max; x += 0.05) {
    for (double y = m_free_y_min; y <= m_free_y_max; y += 0.05) {
      for (double z = m_free_z_min; z <= m_free_z_max; z += 0.05) {
        m_octree->updateNode(x, y, z, (float)-1.0, true);
      }
    }
  }
  m_octree->updateInnerOccupancy();

  RCLCPP_INFO(this->get_logger(), "here");
}

void Mapper::localPosition_callback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {

  // does the frame conversion here
  last_pos_t = this->get_clock()->now(); // msg->header.stamp;
  last_quad_state.x = msg->y;
  last_quad_state.y = msg->x;
  last_quad_state.z = -msg->z;
  last_quad_state.vx = msg->vy;
  last_quad_state.vy = msg->vx;
  last_quad_state.vz = -msg->vz;
  last_quad_state.yaw = dyn::clampToPi(0.5 * M_PI - msg->heading);
}

void Mapper::cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  auto start = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(), "received pc");

  if (drop_msg % m_keep_every_n != 0) {
    drop_msg++;
    return;
  }
  RCLCPP_INFO(this->get_logger(), "processing pc");

  // convert ros msg into pcl
  PCLPointCloud pc;
  pcl::fromROSMsg(*msg, pc);

  // transform pcl to the world frame
  geometry_msgs::msg::TransformStamped sensorToWorldTf;

  std::string fromFrameRel = msg->header.frame_id;
  std::string toFrameRel = "world";

  try {
    sensorToWorldTf = m_buffer_->lookupTransform(toFrameRel, fromFrameRel,
                                                 msg->header.stamp, 200ms);
  } catch (tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
  }

  Eigen::Matrix4f sensorToWorld = pcl_ros::transformAsMatrix(sensorToWorldTf);

  // directly transform to map frame:
  pcl::transformPointCloud(pc, pc, sensorToWorld);

  // filter the points
  // pass_x.setInputCloud(pc.makeShared());
  // pass_x.filter(pc);
  // pass_y.setInputCloud(pc.makeShared());
  // pass_y.filter(pc);
  pass_z.setInputCloud(pc.makeShared());
  pass_z.filter(pc);

  // naive implementation of z filter/
  // PCLPointCloud filtered;
  // for (const auto & point : pc.points){
  //  if (point.z > numMin && point.z < numMax){
  //    filtered.push_back(point);
  //  }
  //}

  // downsample pointcloud
  pcl::VoxelGrid<PCLPoint> vox;
  vox.setInputCloud(pc.makeShared());
  vox.setLeafSize(m_voxel_leaf_size, m_voxel_leaf_size, m_voxel_leaf_size);
  vox.filter(pc);

  insertScan(sensorToWorldTf.transform.translation, pc);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double, std::milli> elapsed_ms = end - start;

  RCLCPP_INFO(this->get_logger(), "insert: %f ms, size: %zu",
              elapsed_ms.count(), m_octree->size());

  publish_unsafe_pcl(sensorToWorldTf.transform.translation);
}

void Mapper::insertScan(const geometry_msgs::msg::Vector3 &sensorOrigin,
                        const PCLPointCloud &pc) {

  // make an octomap pointcloud
  octomap::Pointcloud octomap_pc;

  octomap_pc.reserve(pc.size());

  for (auto it = pc.begin(); it != pc.end(); ++it) {
    octomap_pc.push_back(it->x, it->y, it->z);
  }

  // call the octomap insert method
  octomap::point3d origin(sensorOrigin.x, sensorOrigin.y, sensorOrigin.z);

  // const std::lock_guard<std:://mutex> lock(mutex_octree);
  // mutex_octree.lock();

  auto start = std::chrono::steady_clock::now();
  m_octree->insertPointCloud(octomap_pc, origin, m_maxRange);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double, std::milli> elapsed_ms = end - start;

  // RCLCPP_INFO(this->get_logger(), "insert(): %f ms", elapsed_ms.count());

  // prune map
  m_octree->prune();

  // mutex_octree.unlock();
}

void Mapper::publish_occupied_pcl() {

  // transform pcl to the world frame
  geometry_msgs::msg::TransformStamped sensorToWorldTf;

  std::string fromFrameRel = "vicon/drone4";
  std::string toFrameRel = "world";

  try {
    sensorToWorldTf = m_buffer_->lookupTransform(toFrameRel, fromFrameRel,
                                                 tf2::TimePointZero, 100ms);
  } catch (tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
  }

  double quad_x = sensorToWorldTf.transform.translation.x;
  double quad_y = sensorToWorldTf.transform.translation.y;
  double quad_z = sensorToWorldTf.transform.translation.z;
  PCLPoint q(quad_x, quad_y, quad_z);

  // const std::lock_guard<std:://mutex> lock(mutex_octree);
  // mutex_octree.lock();
  // on a timer-callback, we collect the current octomap and republish it

  // if there is no data, dont publish
  if (m_octree->size() < 1) {
    RCLCPP_WARN(this->get_logger(),
                "No points in octree - not publishing pcl yet...");
    return;
  }

  const double R = 5.0;

  PCLPointCloud pc;
  for (auto it = m_octree->begin(m_maxTreeDepth), end = m_octree->end();
       it != end; ++it) {
    if (m_octree->isNodeOccupied(*it)) {
      PCLPoint p(it.getX(), it.getY(), it.getZ());
      if (p.x > q.x - R && p.x < q.x + R) {
        if (p.y > q.y - R && p.y < q.y + R) {
          if (p.z > q.z - R && p.z < q.z + R) {
            pc.push_back(p);
          }
        }
      }
    }
  }

  // mutex_octree.unlock();
  // convert to ROS msg
  sensor_msgs::msg::PointCloud2 msg;

  pcl::toROSMsg(pc, msg);

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "world";

  occupied_pcl_pub_->publish(msg);

  // this->publish_free_pcl();
}

void Mapper::publish_free_pcl() {

  // on a timer-callback, we collect the current octomap and republish it

  // const std::lock_guard<std:://mutex> lock(mutex_octree);
  // mutex_octree.lock();

  // if there is no data, dont publish
  if (m_octree->size() < 1) {
    RCLCPP_WARN(this->get_logger(),
                "No points in octree - not publishing pcl yet...");
    return;
  }

  PCLPointCloud pc;
  for (auto it = m_octree->begin(m_maxTreeDepth), end = m_octree->end();
       it != end; ++it) {
    if (!(m_octree->isNodeOccupied(*it))) {
      PCLPoint p(it.getX(), it.getY(), it.getZ());
      pc.push_back(p);
    }
  }
  // mutex_octree.unlock();

  // convert to ROS msg
  sensor_msgs::msg::PointCloud2 msg;

  pcl::toROSMsg(pc, msg);

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "world";

  free_pcl_pub_->publish(msg);
}

bool Mapper::isUnsafe(octomap::OcTreeKey key) {
  auto node = m_octree->search(key, m_searchDepth);
  if (node == NULL)
    return true;
  return (m_octree->isNodeOccupied(node));
}

bool Mapper::isUnsafe(double x, double y, double z) {
  auto node = m_octree->search(x, y, z, m_searchDepth);
  if (node == NULL)
    return true;
  return (m_octree->isNodeOccupied(node));
}

void Mapper::publish_unsafe_pcl(const geometry_msgs::msg::Vector3 &origin) {

  auto start = std::chrono::steady_clock::now();

  // if there is no data, dont publish
  if (m_octree->size() < 1) {
    RCLCPP_WARN(this->get_logger(),
                "No points in octree - not publishing pcl yet...");
    return;
  }

  const double R = m_publish_radius;
  const double m_theta_res = 10.0 * M_PI / 180.0; // std::atan2(0.1, R);
  const double m_phi_res = 10.0 * M_PI / 180.0;   // std::atan2(0.1, R);

  const double quad_x = origin.x;
  const double quad_y = origin.y;
  const double quad_z = origin.z;
  PCLPoint p(quad_x, quad_y, quad_z);

  PCLPointCloud pc;
  octomap::KeySet keySet;

  // search for bbx
  for (double theta = 0; theta < 2.0 * M_PI; theta += m_theta_res) {
    for (double phi = -M_PI / 2.0; phi < M_PI / 2.0; phi += m_phi_res) {

      double unit_x = std::cos(theta) * std::cos(phi);
      double unit_y = std::sin(theta) * std::cos(phi);
      double unit_z = std::sin(phi);

      for (double r = 0.05; r < R; r += 0.1) {
        PCLPoint q(quad_x + r * unit_x, quad_y + r * unit_y,
                   quad_z + r * unit_z);
        octomap::point3d oct_q(q.x, q.y, q.z);
        octomap::OcTreeKey key = m_octree->coordToKey(oct_q);
        if (keySet.find(key) != keySet.end()) {
          // exists in keyset
          continue;
        } else {
          keySet.insert(key);
          if (isUnsafe(quad_x + r * unit_x, quad_y + r * unit_y,
                       quad_z + r * unit_z)) {
            PCLPoint q(quad_x + r * unit_x, quad_y + r * unit_y,
                       quad_z + r * unit_z);
            pc.push_back(q);
            break;
          }
        }
      }
    }
  }

  // also add in known obstacles
  // for (auto it = m_octree->begin(m_maxTreeDepth), end = m_octree->end();
  //     it != end; ++it) {
  octomap::point3d min_pt(quad_x - R, quad_y - R, quad_z - R);
  octomap::point3d max_pt(quad_x + R, quad_y + R, quad_z + R);
  for (OcTreeT::leaf_bbx_iterator
           it = m_octree->begin_leafs_bbx(min_pt, max_pt),
           end = m_octree->end_leafs_bbx();
       it != end; ++it) {
    // PCLPoint q(it.getX(), it.getY(), it.getZ());
    //   if (p.x > q.x - R && p.x < q.x + R) {
    //     if (p.y > q.y - R && p.y < q.y + R) {
    //       if (p.z > q.z - R && p.z < q.z + R) {
    if (m_octree->isNodeOccupied(*it)) {
      PCLPoint q(it.getX(), it.getY(), it.getZ());
      pc.push_back(q);
    }
    //   }
    // }
    // }
  }

  if (pc.size() == 0)
    return;

  //  bool isempty = true;
  //  for (double x = quad_x - R; x < quad_x + R; x += 0.10) {
  //    for (double y = quad_y - R; y < quad_y + R; y += 0.10) {
  //      for (double z = quad_z - R; z < quad_z + R; z += 0.10) {
  //        if (isUnsafe(x, y, z)) {
  //          PCLPoint p(x, y, z);
  //          pc.push_back(p);
  //          isempty = false;
  //        }
  //      }
  //    }
  //  }

  //  if (isempty) {
  //    return;
  //  }
  //

  auto end_build = std::chrono::steady_clock::now();

  // convert to ROS msg
  sensor_msgs::msg::PointCloud2 msg;

  pcl::toROSMsg(pc, msg);

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "world";

  unsafe_pcl_pub_->publish(msg);

  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<double, std::milli> elapsed_build_ms =
      end_build - start;
  std::chrono::duration<double, std::milli> elapsed_ms = end - start;

  RCLCPP_INFO(this->get_logger(), "pub: build: %f ms, all: %f",
              elapsed_build_ms.count(), elapsed_ms.count());
}

} // namespace mapper

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<mapper::Mapper>();
  rclcpp::spin(node);

  rclcpp::shutdown();
}
