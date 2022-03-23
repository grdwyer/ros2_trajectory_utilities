/* Author: George Dwyer */

#ifndef ROS2_TRAJECTORY_UTILITIES_INTERPOLATION_H
#define ROS2_TRAJECTORY_UTILITIES_INTERPOLATION_H
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <eigen3/Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <ros2_trajectory_utilities/checks.h>

namespace trajectory_utilities {
    /**
     * @brief returns the pose at factor between the start and end
     * @param start The initial pose
     * @param end The end pose
     * @param factor The factor to interpolate by in [0;1]
     * @return The interpolated pose
     */
    geometry_msgs::msg::Pose interpolate_between_pose(geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose end, double factor);

    /**
     * @brief Interpolates across a cartesian trajectory according to a desired translation and rotation spacing between
     *        poses
     * @param original The original tracjectory to be interpolated
     * @param max_distance The maximum translational distance between poses in metres
     * @param max_angle The maximum rotational distance between poses in radians
     * @param interpolated The interpolated cartesian trajectory
     */
    void interpolate_pose_trajectory(std::vector <geometry_msgs::msg::PoseStamped> &original, double max_distance,
                                     double max_angle, std::vector <geometry_msgs::msg::PoseStamped> &interpolated);

    /**
     * @overload void interpolate_pose_trajectory(std::vector <geometry_msgs::msg::Pose> &original, double max_distance, double max_angle,
                                std::vector <geometry_msgs::msg::Pose> &interpolated)
     *  Overload to use geometry_msgs::msg::Pose instead of stamped pose
     */
    void interpolate_pose_trajectory(std::vector <geometry_msgs::msg::Pose> &original, double max_distance, double max_angle,
                                std::vector <geometry_msgs::msg::Pose> &interpolated);
}
#endif //ROS2_TRAJECTORY_UTILITIES_INTERPOLATION_H
