/* gatekeeper.cpp
 *
 * this file listens to a pointcloud describing the world,
 * builds a kd tree
 * and listens to extended trajectories
 * if they are safe, it republishes them as committed trajectories
 */

#include "gatekeeper/gatekeeper.hpp"

namespace gatekeeper {

Gatekeeper::Gatekeeper() : Node("gatekeeper") {

  // rclcpp::Parameter simTime("use_sim_time", rclcpp::ParameterValue(true));
  // this->set_parameter(simTime);

  // parameters
  this->declare_parameter<double>("gatekeeper/safety_radius_xy", 0.35);
  this->declare_parameter<double>("gatekeeper/safety_radius_z", 0.15);

  this->get_parameter("gatekeeper/safety_radius_xy", m_safety_radius_xy);
  this->get_parameter("gatekeeper/safety_radius_z", m_safety_radius_z);

  // initialize pubs and subs
  m_pointCloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/occupied_pcl", 10,
      std::bind(&Gatekeeper::cloud_callback, this, ph::_1));

  m_nominalTrajSub = this->create_subscription<dasc_msgs::msg::QuadTrajectory>(
      "/nominal_trajectory", 10,
      std::bind(&Gatekeeper::nominalTraj_callback, this, ph::_1));

  m_vehicleLocalPositionSub =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/drone4/fmu/vehicle_local_position/out", 10,
          std::bind(&Gatekeeper::localPosition_callback, this, ph::_1));

  m_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

  m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_buffer_);

  m_committedTrajPub = this->create_publisher<dasc_msgs::msg::QuadTrajectory>(
      "/committed_trajectory", 10);

  m_extTrajVizPub =
      this->create_publisher<nav_msgs::msg::Path>("/ext_traj_viz", 5);

  m_committedTrajVizPub =
      this->create_publisher<nav_msgs::msg::Path>("/committed_traj_viz", 5);

  // traj_timer_ = this->create_wall_timer(
  //    100ms, std::bind(&Gatekeeper::traj_timer_callback, this));

  // initialize kdtree
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

void Gatekeeper::cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  auto start = std::chrono::steady_clock::now();

  // convert ros msg into pcl
  pcl::fromROSMsg(*msg, pc);

  // insert into kd tree
  kdtree.setInputCloud(pc.makeShared());

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double, std::milli> elapsed_ms = end - start;

  RCLCPP_INFO(this->get_logger(), "kdtree: %f ms", elapsed_ms.count());
}

// check if a point is safe
bool Gatekeeper::isSafe(double x, double y, double z) {

  PCLPoint p(x, y, z);

  int K = 1;
  std::vector<int> idx(K);
  std::vector<float> sqdist(K);

  int found = kdtree.nearestKSearch(p, K, idx, sqdist);

  if (found == 0)
    return true; // no neighhbors in unsafe area

  // check the point
  PCLPoint q = pc[idx[0]];

  if (std::abs(p.x - q.x) < m_safety_radius_xy) {
    return false;
  }
  if (std::abs(p.y - q.y) < m_safety_radius_xy) {
    return false;
  }
  if (std::abs(p.z - q.z) < m_safety_radius_z) {
    return false;
  }

  return true;
}

// check if a point, and a box around the point are safe
bool Gatekeeper::isSafe(double x, double y, double z, double r) {

  //  // first check the center
  //  if (!isSafe(x, y, z))
  //    return false;
  //  // check all the 3 cardinal directions
  //  if (!isSafe(x - r, y, z))
  //    return false;
  //  if (!isSafe(x + r, y, z))
  //    return false;
  //  if (!isSafe(x, y - r, z))
  //    return false;
  //  if (!isSafe(x, y + r, z))
  //    return false;
  //  if (!isSafe(x, y, z - r))
  //    return false;
  //  if (!isSafe(x, y, z + r))
  //    return false;
  return true;
}

// returns the largest number of steps in P that can be safe
bool Gatekeeper::isSafe(dyn::Trajectory P) { return true; }
//
//   double R = m_safety_radius; // margin radius
//
//   size_t N = P.ts.size();
//
//   if (N <= 0)
//     return false;
//
//   // at least one elements in P
//
//   // check the end point - likely to be an intersection
//   if (!isSafe(P.xs[N - 1].x, P.xs[N - 1].y, P.xs[N - 1].z, 2 * R)) {
//     RCLCPP_INFO(this->get_logger(), "[REJECTED] endpoint collides");
//     return false;
//   }
//
//   // check that we have come to a stop at the end
//   double vx = P.xs[N - 1].vx;
//   double vy = P.xs[N - 1].vy;
//   double vz = P.xs[N - 1].vz;
//   double v = vx * vx + vy * vy + vz * vz;
//   if (v > 0.01) {
//     RCLCPP_INFO(this->get_logger(), "[REJECTED] too high speed");
//     return false;
//   } // more than 10cm/s
//
//   // check the rest
//   for (size_t i = 0; i < N - 1; i++) {
//     if (!isSafe(P.xs[i].x, P.xs[i].y, P.xs[i].z, R)) {
//       RCLCPP_INFO(this->get_logger(), "[REJECTED] intermediate pt collides");
//       return false;
//     }
//   }
//
//   return true;
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

// void Gatekeeper::traj_timer_callback() {
//
// 	return;
//
//
//   if (!received_target) { return; }
//
//   // forward simulate on the last quad state
//
//
//   // dyn::Trajectory P_ext = simulate_target_hover(0.0, last_quad_state,
//   target, 50, 400, 0.01); dyn::Trajectory P_ext = simulate_target_hover(0.0,
//   last_quad_state, target, 50, 400, 0.01);
//
//   // publish the extended trajectory
//   dasc_msgs::msg::QuadTrajectory ext_msg = direct_copy(P_ext);
//   ext_msg.header.stamp = last_pos_t;
//   ext_msg.header.frame_id = "world";
//   publish_extended_traj(ext_msg);
//
//   // check if the trajectory is safe
//   if (!isSafe(P_ext))
//     return;
//
//   // yes trajectory is safe, so publish
//   dasc_msgs::msg::QuadTrajectory com_msg = direct_copy(P_ext);
//   com_msg.header = ext_msg.header;
//   publish_committed_traj(com_msg);
//
// }

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
    if (nom_msg->ts[i] >= t0) {
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

  if (P.ts.size() == 0) {
    return;
  }

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
  publish_committed_traj(com_msg);
}

} // namespace gatekeeper

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<gatekeeper::Gatekeeper>();
  rclcpp::spin(node);

  rclcpp::shutdown();
}
