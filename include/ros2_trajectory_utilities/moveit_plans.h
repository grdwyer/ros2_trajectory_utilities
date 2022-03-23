/* Author: George Dwyer */

#ifndef ROS2_TRAJECTORY_UTILITIES_MOVEIT_PLANS_H
#define ROS2_TRAJECTORY_UTILITIES_MOVEIT_PLANS_H
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <eigen3/Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <ros2_trajectory_utilities/checks.h>

namespace trajectory_utilities {
    /**
     * @brief function to join two moveit plans and adjust the time from start for a continuous sequence
     * @param first The plan in the sequence to be appended (the time from start will not change) and the plan will be
     *              appended to this plan
     * @param second The second plan in the sequence to be appended (the time from start will be adjusted by the last
     *               time from start of the first plan)
     * @return bool to indicate success
     */
    bool append_plans(moveit::planning_interface::MoveGroupInterface::Plan &first,
                      moveit::planning_interface::MoveGroupInterface::Plan &second);

    /**
     * @brief Retime a moveit plan for the end effector to move at a constant translational velocity
     * @warning This does not check against the joint velocity limits in the retiming, you should use the trapezoidal
     *          retime function.
     * @param plan The plan to be retimed
     * @param robot_state The current robot state to do the FK calls and get the initial joint state
     * @param desired_velocity The desired translational velocity in m/s
     * @param jmg_name The name of the joint model group to use
     * @param end_effector_link The name of the end effector link to use to calculate the velocity
     * @param retimed_plan The retimed moveit plan
     * @return bool to indicate success
     */
    bool retime_trajectory_constant_velocity(moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                             moveit::core::RobotStatePtr robot_state, double desired_velocity,
                                             std::string &jmg_name, std::string &end_effector_link,
                                             moveit::planning_interface::MoveGroupInterface::Plan &retimed_plan);

    /**
     * @brief Retime a moveit plan for the end effector to move along a trapezoidal translation velocity profile
     * @param plan The plan to be retimed
     * @param robot_state The current robot state to do the FK calls and get the initial joint state
     * @param desired_velocity The desired translational velocity in m/s
     * @param desired_acceleration The desired translational acceleration in m/s^2
     * @param jmg_name The name of the joint model group to use
     * @param end_effector_link The name of the end effector link to use to calculate the velocity
     * @param retimed_plan The retimed moveit plan
     * @return bool to indicate success
     */
    bool retime_trajectory_trapezoidal_velocity(moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                                moveit::core::RobotStatePtr robot_state, double desired_velocity,
                                                double desired_acceleration, std::string &jmg_name,
                                                std::string &end_effector_link,
                                                moveit::planning_interface::MoveGroupInterface::Plan &retimed_plan);

}
#endif //ROS2_TRAJECTORY_UTILITIES_MOVEIT_PLANS_H
