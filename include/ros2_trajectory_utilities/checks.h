/* Author: George Dwyer */

#ifndef ROS2_TRAJECTORY_UTILITIES_CHECKS_H
#define ROS2_TRAJECTORY_UTILITIES_CHECKS_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <eigen3/Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

namespace trajectory_utilities {
    /**
     * @brief function to calculate the translational distance between two poses
     * @param a start pose
     * @param b end pose
     * @return distance between a and b in metres
     */
    double distance_between_poses(geometry_msgs::msg::Pose a, geometry_msgs::msg::Pose b);

    /**
     * @brief function to calculate the rotational distance between two poses
     * @param a start pose
     * @param b end pose
     * @return distance between a and b in radians
     */
    double angular_distance_between_poses(geometry_msgs::msg::Pose a, geometry_msgs::msg::Pose b);

    /**
     * @brief calculates the total translational distance along a trajectory
     * @param trajectory The trajectory to calculate the distance of
     * @return Distance along the trajectory in metres
     */
    double distance_along_trajectory(std::vector<geometry_msgs::msg::PoseStamped> &trajectory);

    /**
     * @brief checks the desired joint velocities of a series of joint against the limits provided in the urdf
     * @param robot_model The model of the manipulator in use
     * @param joint_diff A map of joint names and desired velocities to check
     * @return True if all joints and within their limits and false if any joint exceeds
     */
    bool check_joint_velocities(moveit::core::RobotModelConstPtr robot_model, const std::map<std::string, double>& joint_diff);

}

#endif //ROS2_TRAJECTORY_UTILITIES_CHECKS_H
