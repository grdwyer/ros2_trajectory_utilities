//
// Created by george on 1/4/22.
//

#ifndef ROS2_TRAJECTORY_UTILITIES_MOVEIT_PLANS_H
#define ROS2_TRAJECTORY_UTILITIES_MOVEIT_PLANS_H
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <eigen3/Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <ros2_trajectory_utilities/checks.h>

namespace trajectory_utilities {
    bool append_plans(moveit::planning_interface::MoveGroupInterface::Plan &first,
                      moveit::planning_interface::MoveGroupInterface::Plan &second);

    bool retime_trajectory_constant_velocity(moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                             moveit::core::RobotStatePtr robot_state, double desired_velocity,
                                             std::string &jmg_name, std::string &end_effector_link,
                                             moveit::planning_interface::MoveGroupInterface::Plan &retimed_plan);

    bool retime_trajectory_trapezoidal_velocity(moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                                moveit::core::RobotStatePtr robot_state, double desired_velocity,
                                                double desired_acceleration, std::string &jmg_name,
                                                std::string &end_effector_link,
                                                moveit::planning_interface::MoveGroupInterface::Plan &retimed_plan);

}
#endif //ROS2_TRAJECTORY_UTILITIES_MOVEIT_PLANS_H
