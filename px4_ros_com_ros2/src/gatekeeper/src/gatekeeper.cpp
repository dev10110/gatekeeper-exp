/* gatekeeper.cpp
 *
 * this file listens to sensor messages,
 * builds an octomap of the world
 * and listens to extended trajectories
 * if they are safe, it republishes them as committed trajectories
 */

#include "gatekeeper/gatekeeper.hpp"

namespace gatekeeper {

Gatekeeper::Gatekeeper() : Node("gatekeeper") {

  // rclcpp::Parameter simTime("use_sim_time", rclcpp::ParameterValue(true));
  // this->set_parameter(simTime);

  // parameters
  this->declare_parameter<double>("octomap/m_res", 0.05);
  this->declare_parameter<double>("octomap/m_zmin", -0.25);
  this->declare_parameter<double>("octomap/m_zmax", 3.0);
  this->declare_parameter<double>("voxel/leaf_size", 0.01);
  this->declare_parameter<double>("gatekeeper/safety_radius", 0.15);

  this->get_parameter("octomap/m_res", m_res);
  this->get_parameter("octomap/m_zmin", m_zmin);
  this->get_parameter("octomap/m_zmax", m_zmax);
  this->get_parameter("voxel/leaf_size", m_voxel_leaf_size);
  this->get_parameter("gatekeeper/safety_radius", m_safety_radius);

  // initialize pubs and subs

  m_pointCloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/color/points", 10,
      std::bind(&Gatekeeper::cloud_callback, this, ph::_1));

  m_nominalTrajSub = this->create_subscription<dasc_msgs::msg::QuadTrajectory>(
      "/nominal_trajectory", 10,
      std::bind(&Gatekeeper::nominalTraj_callback, this, ph::_1));

  m_vehicleLocalPositionSub =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/drone4/fmu/vehicle_local_position/out", 10,
          std::bind(&Gatekeeper::localPosition_callback, this, ph::_1));

  m_targetSub = this->create_subscription<dasc_msgs::msg::QuadSetpoint>(
      "/target_setpoint", 10,
      std::bind(&Gatekeeper::target_callback, this, ph::_1));

  m_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

  m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_buffer_);

  occupied_pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/occupied_octomap", 2);

  free_pcl_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/free_octomap", 2);

  m_committedTrajPub = this->create_publisher<dasc_msgs::msg::QuadTrajectory>(
      "/committed_trajectory", 10);

  m_extTrajVizPub =
      this->create_publisher<nav_msgs::msg::Path>("/ext_traj_viz", 5);

  m_committedTrajVizPub =
      this->create_publisher<nav_msgs::msg::Path>("/committed_traj_viz", 5);

  pub_timer_ = this->create_wall_timer(
      500ms, std::bind(&Gatekeeper::publish_occupied_pcl, this));

  traj_timer_ = this->create_wall_timer(
      100ms, std::bind(&Gatekeeper::traj_timer_callback, this));
  

  // initialize octomap
  m_octree = std::make_shared<OcTreeT>(m_res);
  m_octree->setProbHit(probHit);
  m_octree->setProbMiss(probMiss);
  m_octree->setClampingThresMin(thresMin);
  m_octree->setClampingThresMax(thresMax);

  m_searchDepth = 0;
  for (int i = 0; i < 17; i++) {
    double s = m_octree->getNodeSize(i);
    if (s > 0.15) {
      m_searchDepth = i;
    } else {
      break;
    }
  }

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

  // initialize target
  target.x = 0.0;
  target.y = 0.0;
  target.z = 1.0;
  target.yaw = 0.0;
  target.vx = 0.0;
  target.vy = 0.0;
  target.vz = 0.0;
  received_target = true;
}

void Gatekeeper::publish_committed_traj(
    const dasc_msgs::msg::QuadTrajectory msg) {

  m_committedTrajPub->publish(msg);

  nav_msgs::msg::Path path = dasc_msgs::msg::toPathMsg(msg);

  m_committedTrajVizPub->publish(path);
}

void Gatekeeper::publish_extended_traj(
    const dasc_msgs::msg::QuadTrajectory msg) {

  RCLCPP_INFO(this->get_logger(), "extended trajectory published");
  nav_msgs::msg::Path path = dasc_msgs::msg::toPathMsg(msg);
  m_extTrajVizPub->publish(path);
}

void Gatekeeper::assume_free_space() {

  RCLCPP_INFO(this->get_logger(), "here");
  for (double x = -2.0; x <= 0.0; x += 0.05) {
    for (double y = -1.0; y <= 1.0; y += 0.05) {
      for (double z = 0.0; z <= 2.0; z += 0.05) {
        m_octree->updateNode(x, y, z, (float)-0.9, true);
      }
    }
  }
  m_octree->updateInnerOccupancy();

  RCLCPP_INFO(this->get_logger(), "here");
}

void Gatekeeper::cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  auto start = std::chrono::steady_clock::now();

  if (drop_msg % 5 != 0) {
    drop_msg++;
    return;
  }

  // convert ros msg into pcl
  PCLPointCloud pc;
  pcl::fromROSMsg(*msg, pc);

  // transform pcl to the world frame
  geometry_msgs::msg::TransformStamped sensorToWorldTf;

  std::string fromFrameRel = msg->header.frame_id;
  std::string toFrameRel = "world";

  try {
    sensorToWorldTf =
        m_buffer_->lookupTransform(toFrameRel, fromFrameRel, msg->header.stamp, 200ms);
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
}

// check if a point is safe
bool Gatekeeper::isSafe(double x, double y, double z) {

  auto node = m_octree->search(x, y, z, m_searchDepth);
  if (node == NULL)
    return false;
  return !(m_octree->isNodeOccupied(node));
}

// check if a point, and a box around the point are safe
bool Gatekeeper::isSafe(double x, double y, double z, double r) {
  // first check the center
  if (!isSafe(x, y, z))
    return false;
  // check all the 3 cardinal directions
  if (!isSafe(x - r, y, z))
    return false;
  if (!isSafe(x + r, y, z))
    return false;
  if (!isSafe(x, y - r, z))
    return false;
  if (!isSafe(x, y + r, z))
    return false;
  if (!isSafe(x, y, z - r))
    return false;
  if (!isSafe(x, y, z + r))
    return false;
  return true;
}

// returns the largest number of steps in P that can be safe
bool Gatekeeper::isSafe(dyn::Trajectory P) {

return true;

  double R = m_safety_radius; // margin radius

  size_t N = P.ts.size();

  if (N <= 0)
    return false;

  // at least one elements in P

  // check the end point - likely to be an intersection
  if (!isSafe(P.xs[N - 1].x, P.xs[N - 1].y, P.xs[N - 1].z, 2 * R)) {
    RCLCPP_INFO(this->get_logger(), "[REJECTED] endpoint collides");
    return false;
  }

  // check that we have come to a stop at the end
  double vx = P.xs[N - 1].vx;
  double vy = P.xs[N - 1].vy;
  double vz = P.xs[N - 1].vz;
  double v = vx * vx + vy * vy + vz * vz;
  if (v > 0.01) {
    RCLCPP_INFO(this->get_logger(), "[REJECTED] too high speed");
    return false;
  } // more than 10cm/s

  // check the rest
  for (size_t i = 0; i < N - 1; i++) {
    if (!isSafe(P.xs[i].x, P.xs[i].y, P.xs[i].z, R)) {
      RCLCPP_INFO(this->get_logger(), "[REJECTED] intermediate pt collides");
      return false;
    }
  }

  return true;
}

// // returns the largest number of steps in P that can be safe
// size_t Gatekeeper::isSafe(dyn::Trajectory P) {
//
//   size_t maxI = 0;
//
//   double R = 0.15; // margin
//
//   // check if the trajectory is in free space
//   for (size_t i = 0; i < P.ts.size(); i++) {
//
//     // the last point has a stricter requirement
//     double r = ( i == P.ts.size()-1) ? (2*R) : (R);
//
//     dyn::State x = P.xs[i];
//     if ((x.x > 1-r) || (x.x < -1+r))
//       break;
//     if ((x.y > 1-r) || (x.y < -1+r))
//       break;
//     if ((x.z > 2-r) || (x.z < 0+r))
//       break;
//
//     maxI = i + 1;
//   }
//
//   return maxI;
// }

dasc_msgs::msg::QuadTrajectory direct_copy(dyn::Trajectory P, int i = -1) {

  dasc_msgs::msg::QuadTrajectory msg;

  size_t end = (i < 0 ? P.ts.size() : i);
  for (size_t i = 0; i < end; i++) {
    msg.ts.push_back(P.ts[i]);
    msg.xs.push_back(P.xs[i].x);
    msg.ys.push_back(P.xs[i].y);
    msg.zs.push_back(P.xs[i].z);
    msg.vxs.push_back(P.xs[i].vx);
    msg.vys.push_back(P.xs[i].vy);
    msg.vzs.push_back(P.xs[i].vz);
    msg.yaws.push_back(P.xs[i].yaw);
    msg.axs.push_back(P.us[i].ax);
    msg.ays.push_back(P.us[i].ay);
    msg.azs.push_back(P.us[i].az);
  }

  return msg;
}

dyn::Trajectory
direct_copy(const dasc_msgs::msg::QuadTrajectory::SharedPtr nom) {

  dyn::Trajectory P;

  for (size_t i = 0; i < nom->ts.size(); i++) {
    P.ts.push_back(nom->ts[i]);
    dyn::State x{nom->xs[i],  nom->ys[i],  nom->zs[i],  nom->vxs[i],
                 nom->vys[i], nom->vzs[i], nom->yaws[i]};
    P.xs.push_back(x);
    dyn::Input u{0, 0, 0, 0};
    P.us.push_back(u);
  }

  return P;
}

void Gatekeeper::localPosition_callback(
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


void Gatekeeper::target_callback( const dasc_msgs::msg::QuadSetpoint::SharedPtr msg) {

  target.x = msg->x;
  target.y = msg->y;
  target.z = msg->z;
  target.vx = 0.0;
  target.vy = 0.0;
  target.vz = 0.0;
  target.yaw = msg->yaw;


  received_target = true;

}

void Gatekeeper::traj_timer_callback() {

	return;


  if (!received_target) { return; }

  // forward simulate on the last quad state


  // dyn::Trajectory P_ext = simulate_target_hover(0.0, last_quad_state, target, 50, 400, 0.01);
  dyn::Trajectory P_ext = simulate_target_hover(0.0, last_quad_state, target, 50, 400, 0.01);

  // publish the extended trajectory
  dasc_msgs::msg::QuadTrajectory ext_msg = direct_copy(P_ext);
  ext_msg.header.stamp = last_pos_t;
  ext_msg.header.frame_id = "world";
  publish_extended_traj(ext_msg);

  // check if the trajectory is safe
  if (!isSafe(P_ext))
    return;

  // yes trajectory is safe, so publish
  dasc_msgs::msg::QuadTrajectory com_msg = direct_copy(P_ext);
  com_msg.header = ext_msg.header;
  publish_committed_traj(com_msg);

}

void Gatekeeper::nominalTraj_callback(
    const dasc_msgs::msg::QuadTrajectory::SharedPtr nom_msg) {

  auto now_ = this->get_clock()->now();

  double t0 = (now_ - nom_msg->header.stamp).seconds();

  // RCLCPP_INFO(this->get_logger(), "t0: %f, t_back: %f", t0,
  // nom_msg->ts.back());

  if (t0 > nom_msg->ts.back()) {
    // committed message is too old - ignore
    RCLCPP_INFO(this->get_logger(),
                "[REJECTING] nominal trajectory is too old");
    return;
  }

  double delta = (last_pos_t - rclcpp::Time(nom_msg->header.stamp)).seconds();
  // RCLCPP_INFO(this->get_logger(), "New nom delta %f", delta);

  // copy over the data that is new
  dyn::Trajectory P;
  for (size_t i = 0; i < nom_msg->ts.size(); i++) {
    if (nom_msg->ts[i] >= t0)
    {
      P.ts.push_back(nom_msg->ts[i]);
      dyn::State x{nom_msg->xs[i],
                   nom_msg->ys[i],
                   nom_msg->zs[i],
                   nom_msg->vxs[i],
                   nom_msg->vys[i],
                   nom_msg->vzs[i],
                   dyn::clampToPi(nom_msg->yaws[i])};
      // RCLCPP_INFO(this->get_logger(), "NOM YAW: %f", x.yaw);
      P.xs.push_back(x);
      dyn::Input u{0, 0, 0, 0};
      P.us.push_back(u);
    }
  }

  if (P.ts.size() == 0) { return; }


  // simulate and extend dynamics
  dyn::Trajectory P_ext =
      dyn::simulate_extend_hover(0.0, last_quad_state, P, 50, 400, 0.01);

  {
    // print the extended
   //  RCLCPP_INFO(this->get_logger(), "EXT INIT: (%f, %f, %f) (%f, %f, %f)",
   //              P_ext.xs[0].x, P_ext.xs[0].y, P_ext.xs[0].z, P_ext.xs[0].vx,
   //              P_ext.xs[0].vy, P_ext.xs[0].vz);
   //  RCLCPP_INFO(this->get_logger(), "EXT FIN: (%f, %f, %f) (%f, %f, %f)",
   //              P_ext.xs.back().x, P_ext.xs.back().y, P_ext.xs.back().z,
   //              P_ext.xs.back().vx, P_ext.xs.back().vy, P_ext.xs.back().vz);
    // for (size_t i = 0; i < P_ext.ts.size(); i++) {

    // RCLCPP_INFO(this->get_logger(), "%f: (%f, %f, %f)", P_ext.ts[i],
    //            P_ext.xs[i].x, P_ext.xs[i].y, P_ext.xs[i].z);
    // RCLCPP_INFO(this->get_logger(), "%f: %f", P_ext.ts.back(),
    //            P_ext.xs.back().yaw);
    //}
  }

  // publish the extended trajectory
  dasc_msgs::msg::QuadTrajectory ext_msg = direct_copy(P_ext);
  ext_msg.header = nom_msg->header;
  publish_extended_traj(ext_msg);

  // check if the trajectory is safe
  if (!isSafe(P_ext))
    return;

  // yes trajectory is safe, so publish
  dasc_msgs::msg::QuadTrajectory com_msg = direct_copy(P_ext);
  com_msg.header = nom_msg->header;
  // m_committedTrajPub->publish(com_msg);
  publish_committed_traj(com_msg);
}

void Gatekeeper::insertScan(const geometry_msgs::msg::Vector3 &sensorOrigin,
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

void Gatekeeper::publish_occupied_pcl() {

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

  this->publish_free_pcl();
}

void Gatekeeper::publish_free_pcl() {

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

} // namespace gatekeeper

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<gatekeeper::Gatekeeper>();
  rclcpp::spin(node);

  rclcpp::shutdown();
}
