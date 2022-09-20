#pragma once

#include <dasc_msgs/msg/quad_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace dasc_msgs::msg {

nav_msgs::msg::Path toPathMsg(QuadTrajectory traj) {

  nav_msgs::msg::Path path;
  path.header = traj.header;

  for (size_t i = 0; i < traj.ts.size(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = traj.header.frame_id;
    pose.header.stamp =
        traj.header.stamp; //  + rclcpp::Duration::from_seconds(traj.ts[i]);
    pose.pose.position.x = traj.xs[i];
    pose.pose.position.y = traj.ys[i];
    pose.pose.position.z = traj.zs[i];
    path.poses.push_back(pose);
  }

  return path;
}

} // namespace dasc_msgs::msg
