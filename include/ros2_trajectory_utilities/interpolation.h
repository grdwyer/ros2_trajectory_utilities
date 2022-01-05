//
// Created by george on 1/4/22.
//

#ifndef ROS2_TRAJECTORY_UTILITIES_INTERPOLATION_H
#define ROS2_TRAJECTORY_UTILITIES_INTERPOLATION_H
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <eigen3/Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <ros2_trajectory_utilities/checks.h>

namespace trajectory_utilities {
// Interpolation
    geometry_msgs::msg::Pose
    interpolate_between_pose(geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose end, double factor);

    void interpolate_pose_trajectory(std::vector <geometry_msgs::msg::PoseStamped> &original, double max_distance,
                                     double max_angle, std::vector <geometry_msgs::msg::PoseStamped> &interpolated);

    void
    interpolate_pose_trajectory(std::vector <geometry_msgs::msg::Pose> &original, double max_distance, double max_angle,
                                std::vector <geometry_msgs::msg::Pose> &interpolated);
}
#endif //ROS2_TRAJECTORY_UTILITIES_INTERPOLATION_H
