//
// Created by george on 1/4/22.
//

#ifndef ROS2_TRAJECTORY_UTILITIES_CHECKS_H
#define ROS2_TRAJECTORY_UTILITIES_CHECKS_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <eigen3/Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

namespace trajectory_utilities {
    double distance_between_poses(geometry_msgs::msg::Pose a, geometry_msgs::msg::Pose b);
    double angular_distance_between_poses(geometry_msgs::msg::Pose a, geometry_msgs::msg::Pose b);

    double distance_along_trajectory(std::vector<geometry_msgs::msg::PoseStamped> &trajectory);

    bool check_joint_velocities(moveit::core::RobotModelConstPtr robot_model, const std::map<std::string, double>& joint_diff);

}

#endif //ROS2_TRAJECTORY_UTILITIES_CHECKS_H
